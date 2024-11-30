#ifndef HBA_HPP
#define HBA_HPP

#include <thread>  // 处理多线程
#include <fstream> // 文件操作
#include <iomanip> // 设置输出格式
#include <Eigen/Sparse> // 稀疏矩阵
#include <Eigen/Eigenvalues> // Eigen 特征值分解
#include <Eigen/SparseCholesky> // 稀疏矩阵求解
#include <visualization_msgs/Marker.h> // 可视化标记消息
#include <visualization_msgs/MarkerArray.h> // 可视化标记数组消息

#include <gtsam/geometry/Pose3.h> // GTSAM 位姿表示
#include <gtsam/slam/PriorFactor.h> // GTSAM 先验因子
#include <gtsam/slam/BetweenFactor.h> // GTSAM 两帧之间的因子
#include <gtsam/nonlinear/Values.h> // GTSAM 非线性方程组的结果
#include <gtsam/nonlinear/ISAM2.h> // GTSAM 增量式优化

#include "mypcl.hpp"  // PCL 处理相关工具
#include "tools.hpp"  // 数学相关工具
#include "ba.hpp"     // Bundle Adjustment 相关

/**
 * @brief LAYER 类，表示 HBA 优化中的一层
 * 每一层包含了与该层相关的位姿、点云数据、Hessian 矩阵等信息。
 */
class LAYER
{
public:
  // 基本参数
  int pose_size;  // 当前层的位姿数量，根据滑动窗口的大小和上一层的处理结果确定。
  int layer_num;  // 当前层的编号，从 1 开始递增，用于区分不同层的处理。
  int max_iter;   // 当前层的最大迭代次数，用于控制每层优化时的迭代上限。
  int part_length;  // 每个线程处理的位姿数量，在多线程并行计算中用于划分任务。
  int left_size;  // 剩余未分配给线程的位姿数量，处理线程数不能整除时，剩余的位姿由 tail 线程处理。
  int left_h_size;  // 剩余的 pose 组，在多线程处理时，剩余部分的处理量，用于 Hessian 矩阵计算。
  int j_upper;  // 剩余部分的上界，用于处理多线程不能整除的剩余任务（`parallel_tail` 中使用）。
  int tail;  // 表示最后一组位姿的数量，滑动窗口不能整除时的剩余部分。
  int thread_num;  // 当前层的线程数，表示并行计算时的线程数量。
  int gap_num;  // 滑动窗口中用于计算的间隔数量，表示两帧位姿之间的跳跃间隔。
  int last_win_size;  // 最后一组滑动窗口的大小，用于处理最后一组位姿时的窗口尺寸。
  int left_gap_num;  // 剩余的 gap 数量，用于多线程处理时，剩余任务的 gap 数量。
  
  // 各种优化的参数
  double downsample_size, voxel_size, eigen_ratio, reject_ratio;
  
  // 数据路径
  std::string data_path;
  
  // 当前层所有的位姿
  vector<mypcl::pose> pose_vec;
  
  // 线程池
  std::vector<thread*> mthreads;
  
  // 存储每个线程的内存消耗
  std::vector<double> mem_costs;

  // Hessian 矩阵及其向量表示
  std::vector<VEC(6)> hessians;

  // 当前层的点云数据
  std::vector<pcl::PointCloud<PointType>::Ptr> pcds;

  /**
   * @brief LAYER 构造函数，初始化层的参数
   */
  LAYER()
  {
    pose_size = 0;
    layer_num = 1; // 默认层号为 1
    max_iter = 10; // 最大迭代次数
    downsample_size = 0.1; // 点云降采样大小
    voxel_size = 4.0; // 体素大小
    eigen_ratio = 0.1; // 特征值比率
    reject_ratio = 0.05; // 异常点剔除比率
    pose_vec.clear(); // 清空位姿向量
    mthreads.clear(); // 清空线程向量
    pcds.clear();     // 清空点云数据向量
    hessians.clear(); // 清空 Hessian 矩阵
    mem_costs.clear(); // 清空内存消耗向量
  }

  /**
   * @brief 初始化存储，分配当前层的数据存储空间
   * @param total_layer_num_ 总层数
   */
  void init_storage(int total_layer_num_)
  {
    // 分配线程池的空间
    mthreads.resize(thread_num);
    
    // 分配每个线程的内存消耗空间
    mem_costs.resize(thread_num);

    // 分配点云存储空间
    pcds.resize(pose_size);
    
    // 分配位姿存储空间
    pose_vec.resize(pose_size);

    #ifdef FULL_HESS
    // 如果当前层不是最后一层，计算 Hessian 矩阵的大小
    if(layer_num < total_layer_num_)
    {
      int hessian_size = (thread_num-1)*(WIN_SIZE-1)*WIN_SIZE/2*part_length;
      hessian_size += (WIN_SIZE-1)*WIN_SIZE/2*left_gap_num;
      if(tail > 0) hessian_size += (last_win_size-1)*last_win_size/2;
      hessians.resize(hessian_size);
      printf("hessian_size: %d\n", hessian_size);
    }
    // 如果是最后一层，计算全局 Hessian 矩阵的大小
    else
    {
      int hessian_size = pose_size*(pose_size-1)/2;
      hessians.resize(hessian_size);
      printf("hessian_size: %d\n", hessian_size);
    }
    #endif

    // 初始化每个线程的内存消耗
    for(int i = 0; i < thread_num; i++)
      mem_costs.push_back(0);
  }

  /**
   * @brief 初始化层参数，计算每一层的位姿数量、滑动窗口等信息
   * @param pose_size_ 可选参数，指定位姿数量
   */
  void init_parameter(int pose_size_ = 0)
  {
    if(layer_num == 1)
      pose_size = pose_vec.size(); // 第一层的位姿数量等于位姿向量的大小
    else
      pose_size = pose_size_; // 非第一层需要手动传入位姿数量

    // 计算尾部点的数目
    tail = (pose_size - WIN_SIZE) % GAP;

    // 计算 gap 的数量（gap 是两帧之间的差距）
    gap_num = (pose_size - WIN_SIZE) / GAP;

    // 计算最后窗口的大小
    last_win_size = pose_size - GAP * (gap_num+1);

    // 计算每个线程处理的部分长度
    part_length = ceil((gap_num+1)/double(thread_num));
    
    // 如果当前分配的部分长度不足，调整
    if(gap_num-(thread_num-1)*part_length < 0)
      part_length = floor((gap_num+1)/double(thread_num));

    // 调整线程数量，确保每个线程的负载均衡
    while(part_length == 0 || (gap_num-(thread_num-1)*part_length+1)/double(part_length) > 2)
    {
      thread_num -= 1;
      part_length = ceil((gap_num+1)/double(thread_num));
      if(gap_num-(thread_num-1)*part_length < 0)
        part_length = floor((gap_num+1)/double(thread_num));
    }

    // 计算剩余的 gap 数量
    left_gap_num = gap_num-(thread_num-1)*part_length+1;
    
    // 根据尾部点的数量调整各个参数
    if(tail == 0)
    {
      left_size = (gap_num-(thread_num-1)*part_length+1)*WIN_SIZE;
      left_h_size = (gap_num-(thread_num-1)*part_length)*GAP+WIN_SIZE-1;
      j_upper = gap_num-(thread_num-1)*part_length+1;
    }
    else
    {
      left_size = (gap_num-(thread_num-1)*part_length+1)*WIN_SIZE+GAP+tail;
      left_h_size = (gap_num-(thread_num-1)*part_length+1)*GAP+last_win_size-1;
      j_upper = gap_num-(thread_num-1)*part_length+2;
    }

    // 打印初始化参数
    printf("init parameter:\n");
    printf("layer_num %d | thread_num %d | pose_size %d | max_iter %d | part_length %d | gap_num %d | last_win_size %d | "
      "left_gap_num %d | tail %d | left_size %d | left_h_size %d | j_upper %d | "
      "downsample_size %f | voxel_size %f | eigen_ratio %f | reject_ratio %f\n",
      layer_num, thread_num, pose_size, max_iter, part_length, gap_num, last_win_size,
      left_gap_num, tail, left_size, left_h_size, j_upper,
      downsample_size, voxel_size, eigen_ratio, reject_ratio);
  }
};

/**
 * @brief HBA 类，表示整个 HBA 系统
 * HBA 系统由多层组成，每一层都包含有点云数据和姿态信息，并通过多线程进行优化。
 */
class HBA
{
public:
  int thread_num, total_layer_num; // 线程数和总层数
  std::vector<LAYER> layers; // 所有层的集合
  std::string data_path; // 数据路径

  /**
   * @brief HBA 构造函数，初始化 HBA 系统
   * @param total_layer_num_ 总层数
   * @param data_path_ 数据路径
   * @param thread_num_ 线程数
   */
  HBA(int total_layer_num_, std::string data_path_, int thread_num_)
  {
    total_layer_num = total_layer_num_; // 设置总层数
    thread_num = thread_num_; // 设置线程数
    data_path = data_path_; // 设置数据路径

    // 初始化每一层
    layers.resize(total_layer_num);
    for(int i = 0; i < total_layer_num; i++)
    {
      layers[i].layer_num = i+1; // 设置层号
      layers[i].thread_num = thread_num; // 设置每层的线程数
    }

    // 初始化第一层的位姿数据，读取姿态文件
    layers[0].data_path = data_path;
    layers[0].pose_vec = mypcl::read_pose(data_path + "pose.json");
    layers[0].init_parameter(); // 初始化第一层的参数
    layers[0].init_storage(total_layer_num); // 为第一层分配存储空间

    // 初始化后续层
    for(int i = 1; i < total_layer_num; i++)
    {
      int pose_size_ = (layers[i-1].thread_num-1)*layers[i-1].part_length;
      pose_size_ += layers[i-1].tail == 0 ? layers[i-1].left_gap_num : (layers[i-1].left_gap_num+1);
      layers[i].init_parameter(pose_size_);
      layers[i].init_storage(total_layer_num);
      layers[i].data_path = layers[i-1].data_path + "process1/"; // 设置数据路径
    }
    printf("HBA init done!\n");
  }

  /**
   * @brief HBA 构造函数，初始化 HBA 系统
   * @param total_layer_num_ 总层数
   * @param data_path_ 数据路径
   * @param thread_num_ 线程数
   * @param i_th_ 第i次迭代，从0计数
   */
  HBA(int total_layer_num_, std::string data_path_, int thread_num_, int i_th_, std::string input_json_file)
  {
    total_layer_num = total_layer_num_; // 设置总层数
    thread_num = thread_num_; // 设置线程数
    data_path = data_path_; // 设置数据路径

    // 初始化每一层
    layers.resize(total_layer_num);
    for(int i = 0; i < total_layer_num; i++)
    {
      layers[i].layer_num = i+1; // 设置层号
      layers[i].thread_num = thread_num; // 设置每层的线程数
    }

    // 初始化第一层的位姿数据，读取姿态文件
    layers[0].data_path = data_path;
    std::string filename;
    if(i_th_==0){
      filename = data_path + input_json_file;
    }else{
      filename = data_path + "pose_" + std::to_string(i_th_) + ".json";
    }

    layers[0].pose_vec = mypcl::read_pose(filename);
    layers[0].init_parameter(); // 初始化第一层的参数
    layers[0].init_storage(total_layer_num); // 为第一层分配存储空间

    // 初始化后续层
    for(int i = 1; i < total_layer_num; i++)
    {
      int pose_size_ = (layers[i-1].thread_num-1)*layers[i-1].part_length;
      pose_size_ += layers[i-1].tail == 0 ? layers[i-1].left_gap_num : (layers[i-1].left_gap_num+1);
      layers[i].init_parameter(pose_size_);
      layers[i].init_storage(total_layer_num);
      layers[i].data_path = layers[i-1].data_path + "process1/"; // 设置数据路径
    }
    printf("HBA init done!\n");
  }

  /**
   * @brief 更新下一层的位姿状态
   * @param cur_layer_num 当前层的编号
   */
  void update_next_layer_state(int cur_layer_num)
  {
    for(int i = 0; i < layers[cur_layer_num].thread_num; i++)
    {
      if(i < layers[cur_layer_num].thread_num-1)
      {
        for(int j = 0; j < layers[cur_layer_num].part_length; j++)
        {
          int index = (i * layers[cur_layer_num].part_length + j) * GAP;
          layers[cur_layer_num+1].pose_vec[i*layers[cur_layer_num].part_length+j] = layers[cur_layer_num].pose_vec[index];
        }
      }
      else
      {
        for(int j = 0; j < layers[cur_layer_num].j_upper; j++)
        {
          int index = (i * layers[cur_layer_num].part_length + j) * GAP;
          layers[cur_layer_num+1].pose_vec[i*layers[cur_layer_num].part_length+j] = layers[cur_layer_num].pose_vec[index];
        }
      }
    }
  }

/**
   * @brief 执行姿态图优化，使用 GTSAM 进行优化
   */
  void pose_graph_optimization()
  {
    std::vector<mypcl::pose> upper_pose, init_pose;
    upper_pose = layers[total_layer_num-1].pose_vec; // 获取最后一层的姿态
    init_pose = layers[0].pose_vec; // 获取第一层的姿态
    std::vector<VEC(6)> upper_cov, init_cov;
    upper_cov = layers[total_layer_num-1].hessians;
    init_cov = layers[0].hessians;

    int cnt = 0;
    gtsam::Values initial;
    gtsam::NonlinearFactorGraph graph;
    gtsam::Vector Vector6(6);
    Vector6 << 1e-6, 1e-6, 1e-6, 1e-8, 1e-8, 1e-8;

    // 设置先验模型
    gtsam::noiseModel::Diagonal::shared_ptr priorModel = gtsam::noiseModel::Diagonal::Variances(Vector6);
    initial.insert(0, gtsam::Pose3(gtsam::Rot3(init_pose[0].q.toRotationMatrix()), gtsam::Point3(init_pose[0].t)));
    graph.add(gtsam::PriorFactor<gtsam::Pose3>(0, gtsam::Pose3(gtsam::Rot3(init_pose[0].q.toRotationMatrix()),
                                                               gtsam::Point3(init_pose[0].t)), priorModel));
    
    // 遍历初始位姿，生成因子图
    for(uint i = 0; i < init_pose.size(); i++)
    {
      if(i > 0) 
        initial.insert(i, gtsam::Pose3(gtsam::Rot3(init_pose[i].q.toRotationMatrix()), gtsam::Point3(init_pose[i].t)));

      if(i % GAP == 0 && cnt < init_cov.size())
      {
        for(int j = 0; j < WIN_SIZE-1; j++)
          for(int k = j+1; k < WIN_SIZE; k++)
          {
            if(i+j+1 >= init_pose.size() || i+k >= init_pose.size()) 
              break;

            cnt++;
            if(init_cov[cnt-1].norm() < 1e-20) 
              continue;

            // 计算两帧之间的相对位姿
            Eigen::Vector3d t_ab = init_pose[i+j].t;
            Eigen::Matrix3d R_ab = init_pose[i+j].q.toRotationMatrix();
            t_ab = R_ab.transpose() * (init_pose[i+k].t - t_ab);
            R_ab = R_ab.transpose() * init_pose[i+k].q.toRotationMatrix();

            gtsam::Rot3 R_sam(R_ab);
            gtsam::Point3 t_sam(t_ab);
            
            // 设置噪声模型
            Vector6 << fabs(1.0/init_cov[cnt-1](0)), fabs(1.0/init_cov[cnt-1](1)), fabs(1.0/init_cov[cnt-1](2)),
                       fabs(1.0/init_cov[cnt-1](3)), fabs(1.0/init_cov[cnt-1](4)), fabs(1.0/init_cov[cnt-1](5));
            gtsam::noiseModel::Diagonal::shared_ptr odometryNoise = gtsam::noiseModel::Diagonal::Variances(Vector6);
            
            // 添加因子到因子图
            gtsam::NonlinearFactor::shared_ptr factor(new gtsam::BetweenFactor<gtsam::Pose3>(i+j, i+k, gtsam::Pose3(R_sam, t_sam),
                                                      odometryNoise));
            graph.push_back(factor);
          }
      }
    }

    // 添加最后一层的位姿到因子图中
    int pose_size = upper_pose.size();
    cnt = 0;
    for(int i = 0; i < pose_size-1; i++)
    {
      for(int j = i+1; j < pose_size; j++)
      {
        cnt++;
        if(upper_cov[cnt-1].norm() < 1e-20) continue;

        // 计算两帧之间的相对位姿
        Eigen::Vector3d t_ab = upper_pose[i].t;
        Eigen::Matrix3d R_ab = upper_pose[i].q.toRotationMatrix();
        t_ab = R_ab.transpose() * (upper_pose[j].t - t_ab);
        R_ab = R_ab.transpose() * upper_pose[j].q.toRotationMatrix();

        gtsam::Rot3 R_sam(R_ab);
        gtsam::Point3 t_sam(t_ab);

        // 设置噪声模型
        Vector6 << fabs(1.0/upper_cov[cnt-1](0)), fabs(1.0/upper_cov[cnt-1](1)), fabs(1.0/upper_cov[cnt-1](2)),
                   fabs(1.0/upper_cov[cnt-1](3)), fabs(1.0/upper_cov[cnt-1](4)), fabs(1.0/upper_cov[cnt-1](5));
        gtsam::noiseModel::Diagonal::shared_ptr odometryNoise = gtsam::noiseModel::Diagonal::Variances(Vector6);
        
        // 添加因子到因子图
        gtsam::NonlinearFactor::shared_ptr factor(new gtsam::BetweenFactor<gtsam::Pose3>(i*pow(GAP, total_layer_num-1),
                                                  j*pow(GAP, total_layer_num-1), gtsam::Pose3(R_sam, t_sam), odometryNoise));
        graph.push_back(factor);
      }
    }

    // 设置 ISAM2 参数
    gtsam::ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.01;
    parameters.relinearizeSkip = 1;

    // 创建 ISAM2 优化器并进行更新
    gtsam::ISAM2 isam(parameters);
    isam.update(graph, initial);
    isam.update();

    // 计算最终优化结果
    gtsam::Values results = isam.calculateEstimate();

    cout << "vertex size " << results.size() << endl;

    // 将优化后的位姿保存到 init_pose 中
    for(uint i = 0; i < results.size(); i++)
    {
      gtsam::Pose3 pose = results.at(i).cast<gtsam::Pose3>();
      assign_qt(init_pose[i].q, init_pose[i].t, Eigen::Quaterniond(pose.rotation().matrix()), pose.translation());
    }

    // 保存优化后的位姿
    mypcl::write_pose(init_pose, data_path);
    printf("pgo complete\n");
  }

  /**
   * @brief 执行姿态图优化，使用 GTSAM 进行优化
   */
  void pose_graph_optimization(int i_th)
  {
    std::vector<mypcl::pose> upper_pose, init_pose;
    upper_pose = layers[total_layer_num-1].pose_vec; // 获取最后一层的姿态
    init_pose = layers[0].pose_vec; // 获取第一层的姿态
    std::vector<VEC(6)> upper_cov, init_cov;
    upper_cov = layers[total_layer_num-1].hessians;
    init_cov = layers[0].hessians;

    int cnt = 0;
    gtsam::Values initial;
    gtsam::NonlinearFactorGraph graph;
    gtsam::Vector Vector6(6);
    Vector6 << 1e-6, 1e-6, 1e-6, 1e-8, 1e-8, 1e-8;

    // 设置先验模型
    gtsam::noiseModel::Diagonal::shared_ptr priorModel = gtsam::noiseModel::Diagonal::Variances(Vector6);
    initial.insert(0, gtsam::Pose3(gtsam::Rot3(init_pose[0].q.toRotationMatrix()), gtsam::Point3(init_pose[0].t)));
    graph.add(gtsam::PriorFactor<gtsam::Pose3>(0, gtsam::Pose3(gtsam::Rot3(init_pose[0].q.toRotationMatrix()),
                                                               gtsam::Point3(init_pose[0].t)), priorModel));
    
    // 遍历初始位姿，生成因子图
    for(uint i = 0; i < init_pose.size(); i++)
    {
      if(i > 0) 
        initial.insert(i, gtsam::Pose3(gtsam::Rot3(init_pose[i].q.toRotationMatrix()), gtsam::Point3(init_pose[i].t)));

      if(i % GAP == 0 && cnt < init_cov.size())
      {
        for(int j = 0; j < WIN_SIZE-1; j++)
          for(int k = j+1; k < WIN_SIZE; k++)
          {
            if(i+j+1 >= init_pose.size() || i+k >= init_pose.size()) 
              break;

            cnt++;
            if(init_cov[cnt-1].norm() < 1e-20) 
              continue;

            // 计算两帧之间的相对位姿
            Eigen::Vector3d t_ab = init_pose[i+j].t;
            Eigen::Matrix3d R_ab = init_pose[i+j].q.toRotationMatrix();
            t_ab = R_ab.transpose() * (init_pose[i+k].t - t_ab);
            R_ab = R_ab.transpose() * init_pose[i+k].q.toRotationMatrix();

            gtsam::Rot3 R_sam(R_ab);
            gtsam::Point3 t_sam(t_ab);
            
            // 设置噪声模型
            Vector6 << fabs(1.0/init_cov[cnt-1](0)), fabs(1.0/init_cov[cnt-1](1)), fabs(1.0/init_cov[cnt-1](2)),
                       fabs(1.0/init_cov[cnt-1](3)), fabs(1.0/init_cov[cnt-1](4)), fabs(1.0/init_cov[cnt-1](5));
            gtsam::noiseModel::Diagonal::shared_ptr odometryNoise = gtsam::noiseModel::Diagonal::Variances(Vector6);
            
            // 添加因子到因子图
            gtsam::NonlinearFactor::shared_ptr factor(new gtsam::BetweenFactor<gtsam::Pose3>(i+j, i+k, gtsam::Pose3(R_sam, t_sam),
                                                      odometryNoise));
            graph.push_back(factor);
          }
      }
    }

    // 添加最后一层的位姿到因子图中
    int pose_size = upper_pose.size();
    cnt = 0;
    for(int i = 0; i < pose_size-1; i++)
    {
      for(int j = i+1; j < pose_size; j++)
      {
        cnt++;
        if(upper_cov[cnt-1].norm() < 1e-20) continue;

        // 计算两帧之间的相对位姿
        Eigen::Vector3d t_ab = upper_pose[i].t;
        Eigen::Matrix3d R_ab = upper_pose[i].q.toRotationMatrix();
        t_ab = R_ab.transpose() * (upper_pose[j].t - t_ab);
        R_ab = R_ab.transpose() * upper_pose[j].q.toRotationMatrix();

        gtsam::Rot3 R_sam(R_ab);
        gtsam::Point3 t_sam(t_ab);

        // 设置噪声模型
        Vector6 << fabs(1.0/upper_cov[cnt-1](0)), fabs(1.0/upper_cov[cnt-1](1)), fabs(1.0/upper_cov[cnt-1](2)),
                   fabs(1.0/upper_cov[cnt-1](3)), fabs(1.0/upper_cov[cnt-1](4)), fabs(1.0/upper_cov[cnt-1](5));
        gtsam::noiseModel::Diagonal::shared_ptr odometryNoise = gtsam::noiseModel::Diagonal::Variances(Vector6);
        
        // 添加因子到因子图
        gtsam::NonlinearFactor::shared_ptr factor(new gtsam::BetweenFactor<gtsam::Pose3>(i*pow(GAP, total_layer_num-1),
                                                  j*pow(GAP, total_layer_num-1), gtsam::Pose3(R_sam, t_sam), odometryNoise));
        graph.push_back(factor);
      }
    }

    // 设置 ISAM2 参数
    gtsam::ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.01;
    parameters.relinearizeSkip = 1;

    // 创建 ISAM2 优化器并进行更新
    gtsam::ISAM2 isam(parameters);
    isam.update(graph, initial);
    isam.update();

    // 计算最终优化结果
    gtsam::Values results = isam.calculateEstimate();

    cout << "vertex size " << results.size() << endl;

    // 将优化后的位姿保存到 init_pose 中
    for(uint i = 0; i < results.size(); i++)
    {
      gtsam::Pose3 pose = results.at(i).cast<gtsam::Pose3>();
      assign_qt(init_pose[i].q, init_pose[i].t, Eigen::Quaterniond(pose.rotation().matrix()), pose.translation());
    }

    // 保存优化后的位姿
    mypcl::write_pose(init_pose, data_path, i_th);
    printf("pgo complete\n");
  }
};



#endif