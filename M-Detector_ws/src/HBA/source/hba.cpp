/*
 hba.cpp 是系统的主要执行入口，负责启动并管理多层次的 BA（Bundle Adjustment）过程。
 该文件通过 ROS 节点初始化，读取参数并调用 HBA 类进行多层次优化处理。主要功能包括：

  1. 点云数据的降采样、体素化处理。
  2. 并行点云处理（多线程）。
  3. 多层次 BA 优化。
  4. 使用 GTSAM 进行姿态图优化。
  5. 通过 ROS 打印和发布处理结果。
*/

// 标准库
#include <iostream>   // 标准输入输出
#include <iomanip>    // 设置输出格式
#include <fstream>    // 文件操作
#include <string>     // 字符串操作
#include <stdio.h>    // C 标准输入输出
#include <mutex>      // 互斥锁
#include <assert.h>   // 断言
#include <math.h>     // 数学库

// ROS 库
#include <ros/ros.h>  // ROS 基本功能
#include <rosbag/bag.h>  // ROS bag 文件操作
#include <sensor_msgs/Imu.h>         // IMU 传感器消息
#include <sensor_msgs/PointCloud2.h> // 点云消息
#include <geometry_msgs/PoseArray.h> // 位姿数组消息
#include <tf/transform_broadcaster.h> // TF 坐标变换广播
#include <visualization_msgs/Marker.h>       // 可视化标记
#include <visualization_msgs/MarkerArray.h>  // 可视化标记数组
#include <std_msgs/Int32.h>

// PCL 库
#include <pcl/point_cloud.h>              // PCL 点云
#include <pcl/point_types.h>              // PCL 点类型
#include <pcl/io/pcd_io.h>                // PCD 点云文件 I/O
#include <pcl/filters/voxel_grid.h>       // 点云体素滤波
#include <pcl/filters/passthrough.h>      // 直通滤波器
#include <pcl/filters/statistical_outlier_removal.h> // 统计滤波器
#include <pcl/kdtree/kdtree_flann.h>      // KD 树
#include <pcl_conversions/pcl_conversions.h> // PCL 与 ROS 消息转换

// Eigen 库
#include <Eigen/Dense>   // Eigen 矩阵和向量
#include <Eigen/StdVector> // Eigen 向量标准库扩展

// 其他库
#include <ceres/ceres.h>  // Ceres 库，用于优化

// 项目自定义头文件
#include "ba.hpp"   // BA 优化相关
#include "hba.hpp"  // HBA 类定义
#include "tools.hpp" // 工具函数
#include "mypcl.hpp" // PCL 相关工具函数

using namespace std;
using namespace Eigen;

int pcd_name_fill_num = 0; // 保存 PCD 文件名中的填充值

/**
 * @brief 体素化处理函数，将点云分割成体素，并存储在 map 中
 * @param feat_map 存储体素的 map
 * @param feat_pt 输入的点云数据
 * @param q 当前帧的旋转四元数
 * @param t 当前帧的平移向量
 * @param fnum 当前帧序号
 * @param voxel_size 体素的大小
 * @param window_size 滑动窗口大小
 * @param eigen_ratio 特征值比率，用于判断平面性
 */
void cut_voxel(unordered_map<VOXEL_LOC, OCTO_TREE_ROOT*>& feat_map,
                pcl::PointCloud<PointType>& feat_pt,
                Eigen::Quaterniond q, Eigen::Vector3d t, int fnum,
                double voxel_size, int window_size, float eigen_ratio)
{
  float loc_xyz[3]; // 存储点的体素坐标
  // 遍历点云中的每个点
  for(PointType& p_c: feat_pt.points)
  {
    // 将点从局部坐标系变换到全局坐标系
    Eigen::Vector3d pvec_orig(p_c.x, p_c.y, p_c.z); // 原始点坐标
    Eigen::Vector3d pvec_tran = q * pvec_orig + t; // 变换后的点坐标

    for(int j = 0; j < 3; j++)
    {
      // 计算点的体素位置
      loc_xyz[j] = pvec_tran[j] / voxel_size;
      if(loc_xyz[j] < 0) loc_xyz[j] -= 1.0; // 处理负坐标
    }

    // 体素位置，使用整数表示
    VOXEL_LOC position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1], (int64_t)loc_xyz[2]);
    auto iter = feat_map.find(position);
    if(iter != feat_map.end())
    {
      // 如果体素已经存在，则将点加入到该体素中
      iter->second->vec_orig[fnum].push_back(pvec_orig);
      iter->second->vec_tran[fnum].push_back(pvec_tran);

      iter->second->sig_orig[fnum].push(pvec_orig);
      iter->second->sig_tran[fnum].push(pvec_tran);
    }
    else
    {
      // 如果体素不存在，则新建一个体素节点
      OCTO_TREE_ROOT* ot = new OCTO_TREE_ROOT(window_size, eigen_ratio);
      ot->vec_orig[fnum].push_back(pvec_orig);
      ot->vec_tran[fnum].push_back(pvec_tran);
      ot->sig_orig[fnum].push(pvec_orig);
      ot->sig_tran[fnum].push(pvec_tran);

      // 设置体素的中心点和四分之一的长度
      ot->voxel_center[0] = (0.5+position.x) * voxel_size;
      ot->voxel_center[1] = (0.5+position.y) * voxel_size;
      ot->voxel_center[2] = (0.5+position.z) * voxel_size;
      ot->quater_length = voxel_size / 4.0;
      ot->layer = 0;
      feat_map[position] = ot;
    }
  }
}
/**
 * @brief 并行计算函数，用于处理某一层的点云数据
 * @param layer 当前层
 * @param thread_id 当前线程 ID
 * @param next_layer 下一层，用于存储本层处理后的结果
 */
void parallel_comp(LAYER& layer, int thread_id, LAYER& next_layer)
{
  int& part_length = layer.part_length; // 每个线程处理的部分长度
  int& layer_num = layer.layer_num; // 当前层的编号
  for(int i = thread_id*part_length; i < (thread_id+1)*part_length; i++)
  {
    vector<pcl::PointCloud<PointType>::Ptr> src_pc, raw_pc; // 存储源点云和原始点云
    src_pc.resize(WIN_SIZE); raw_pc.resize(WIN_SIZE);

    double residual_cur = 0, residual_pre = 0; // 当前迭代和前一次迭代的残差
    vector<IMUST> x_buf(WIN_SIZE); // 存储滑动窗口中的 IMU 状态
    for(int j = 0; j < WIN_SIZE; j++)
    {
      // 将当前姿态的旋转矩阵和位置信息存储到缓冲区中
      x_buf[j].R = layer.pose_vec[i*GAP+j].q.toRotationMatrix();
      x_buf[j].p = layer.pose_vec[i*GAP+j].t;
    }

    if(layer_num != 1)
      for(int j = i*GAP; j < i*GAP+WIN_SIZE; j++)
        src_pc[j-i*GAP] = (*layer.pcds[j]).makeShared(); // 共享当前层的点云数据

    size_t mem_cost = 0; // 存储内存消耗
    for(int loop = 0; loop < layer.max_iter; loop++)
    {
      if(layer_num == 1)
        for(int j = i*GAP; j < i*GAP+WIN_SIZE; j++)
        {
          // 如果是第一层，加载点云数据
          if(loop == 0)
          {
            pcl::PointCloud<PointType>::Ptr pc(new pcl::PointCloud<PointType>);
            mypcl::loadPCD(layer.data_path, pcd_name_fill_num, pc, j, "pcd/");
            raw_pc[j-i*GAP] = pc;
          }
          src_pc[j-i*GAP] = (*raw_pc[j-i*GAP]).makeShared();
        }

      // 创建体素 map
      unordered_map<VOXEL_LOC, OCTO_TREE_ROOT*> surf_map;
      
      // 遍历滑动窗口内的每个点云帧
      for(size_t j = 0; j < WIN_SIZE; j++)
      {
        // 如果需要降采样，则对点云进行体素降采样
        if(layer.downsample_size > 0) downsample_voxel(*src_pc[j], layer.downsample_size);
        // 进行体素化处理
        cut_voxel(surf_map, *src_pc[j], Eigen::Quaterniond(x_buf[j].R), x_buf[j].p,
                  j, layer.voxel_size, WIN_SIZE, layer.eigen_ratio);
      }
      // 对体素进行递归切分
      for(auto iter = surf_map.begin(); iter != surf_map.end(); ++iter)
        iter->second->recut();

      // 计算 Hessian 矩阵
      VOX_HESS voxhess(WIN_SIZE);
      for(auto iter = surf_map.begin(); iter != surf_map.end(); iter++)
        iter->second->tras_opt(voxhess);

      VOX_OPTIMIZER opt_lsv(WIN_SIZE);
      // 移除异常点
      opt_lsv.remove_outlier(x_buf, voxhess, layer.reject_ratio);
      PLV(6) hess_vec;
      // 进行阻尼迭代优化
      opt_lsv.damping_iter(x_buf, voxhess, residual_cur, hess_vec, mem_cost);

      // 删除体素
      for(auto iter = surf_map.begin(); iter != surf_map.end(); ++iter)
        delete iter->second;
      
      // 判断是否收敛
      if(loop > 0 && abs(residual_pre-residual_cur)/abs(residual_cur) < 0.05 || loop == layer.max_iter-1)
      {
        if(layer.mem_costs[thread_id] < mem_cost) layer.mem_costs[thread_id] = mem_cost;
        
        for(int j = 0; j < WIN_SIZE*(WIN_SIZE-1)/2; j++)
          layer.hessians[i*(WIN_SIZE-1)*WIN_SIZE/2+j] = hess_vec[j];
        
        break;
      }
      residual_pre = residual_cur;
    }

    // 生成关键帧点云
    pcl::PointCloud<PointType>::Ptr pc_keyframe(new pcl::PointCloud<PointType>);
    for(size_t j = 0; j < WIN_SIZE; j++)
    {
      // 计算关键帧之间的位姿变换
      Eigen::Quaterniond q_tmp;
      Eigen::Vector3d t_tmp;
      assign_qt(q_tmp, t_tmp, Quaterniond(x_buf[0].R.inverse() * x_buf[j].R),
                x_buf[0].R.inverse() * (x_buf[j].p - x_buf[0].p));

      pcl::PointCloud<PointType>::Ptr pc_oneframe(new pcl::PointCloud<PointType>);
      mypcl::transform_pointcloud(*src_pc[j], *pc_oneframe, t_tmp, q_tmp);
      pc_keyframe = mypcl::append_cloud(pc_keyframe, *pc_oneframe);
    }
    // 对关键帧降采样(这个操作占了大约70%的时间)
    downsample_voxel(*pc_keyframe, 0.05);
    next_layer.pcds[i] = pc_keyframe;
  }
}

/**
 * @brief 并行处理中处理线程剩余的部分，一般用于处理线程不能整除的剩余部分
 * @param layer 当前层
 * @param thread_id 当前线程 ID
 * @param next_layer 下一层，用于存储本层处理后的结果
 */
void parallel_tail(LAYER& layer, int thread_id, LAYER& next_layer)
{
  int& part_length = layer.part_length; // 每个线程处理的部分长度
  int& layer_num = layer.layer_num; // 当前层的编号
  int& left_gap_num = layer.left_gap_num; // 剩余的 gap 数量

  double load_t = 0, undis_t = 0, dsp_t = 0, cut_t = 0, recut_t = 0, total_t = 0,
    tran_t = 0, sol_t = 0, save_t = 0; // 统计各阶段的时间

  // 验证剩余的 gap 数量是否正确
  if(layer.gap_num-(layer.thread_num-1)*part_length+1 != left_gap_num) 
    printf("THIS IS WRONG!\n");

  // 处理线程剩余的部分
  for(uint i = thread_id*part_length; i < thread_id*part_length+left_gap_num; i++)
  {
    printf("parallel computing %d\n", i);
    double t0, t1;
    double t_begin = ros::Time::now().toSec(); // 记录开始时间
    
    vector<pcl::PointCloud<PointType>::Ptr> src_pc, raw_pc; // 存储源点云和原始点云
    src_pc.resize(WIN_SIZE); raw_pc.resize(WIN_SIZE);

    double residual_cur = 0, residual_pre = 0; // 当前迭代和前一次迭代的残差
    vector<IMUST> x_buf(WIN_SIZE); // 存储滑动窗口中的 IMU 状态
    for(int j = 0; j < WIN_SIZE; j++)
    {
      // 将当前姿态的旋转矩阵和位置信息存储到缓冲区中
      x_buf[j].R = layer.pose_vec[i*GAP+j].q.toRotationMatrix();
      x_buf[j].p = layer.pose_vec[i*GAP+j].t;
    }
    
    if(layer_num != 1)
    {
      t0 = ros::Time::now().toSec(); // 记录加载点云的时间
      for(int j = i*GAP; j < i*GAP+WIN_SIZE; j++)
        src_pc[j-i*GAP] = (*layer.pcds[j]).makeShared(); // 共享当前层的点云数据
      load_t += ros::Time::now().toSec()-t0;
    }

    size_t mem_cost = 0; // 存储内存消耗

    // 迭代优化，最大迭代次数为 max_iter
    for(int loop = 0; loop < layer.max_iter; loop++)
    {
      if(layer_num == 1)
      {
        t0 = ros::Time::now().toSec(); // 记录加载原始点云数据的时间
        for(int j = i*GAP; j < i*GAP+WIN_SIZE; j++)
        {
          if(loop == 0)
          {
            pcl::PointCloud<PointType>::Ptr pc(new pcl::PointCloud<PointType>);
            // 加载当前帧点云数据
            mypcl::loadPCD(layer.data_path, pcd_name_fill_num, pc, j, "pcd/");
            raw_pc[j-i*GAP] = pc;
          }
          src_pc[j-i*GAP] = (*raw_pc[j-i*GAP]).makeShared(); // 共享原始点云数据
        }
        load_t += ros::Time::now().toSec()-t0;
      }

      // 创建体素 map
      unordered_map<VOXEL_LOC, OCTO_TREE_ROOT*> surf_map;

      // 遍历滑动窗口内的每个点云帧
      for(size_t j = 0; j < WIN_SIZE; j++)
      {
        t0 = ros::Time::now().toSec(); // 记录降采样的时间
        if(layer.downsample_size > 0) downsample_voxel(*src_pc[j], layer.downsample_size); // 对点云进行体素降采样
        dsp_t += ros::Time::now().toSec()-t0;

        t0 = ros::Time::now().toSec(); // 记录体素化处理的时间
        // 进行体素化处理
        cut_voxel(surf_map, *src_pc[j], Quaterniond(x_buf[j].R), x_buf[j].p,
                  j, layer.voxel_size, WIN_SIZE, layer.eigen_ratio);
        cut_t += ros::Time::now().toSec()-t0;
      }

      t0 = ros::Time::now().toSec(); // 记录体素递归切分的时间
      // 对体素进行递归切分
      for(auto iter = surf_map.begin(); iter != surf_map.end(); ++iter)
        iter->second->recut();
      recut_t += ros::Time::now().toSec()-t0;

      t0 = ros::Time::now().toSec(); // 记录 Hessian 矩阵计算的时间
      VOX_HESS voxhess(WIN_SIZE); // 创建体素 Hessian 矩阵对象
      for(auto iter = surf_map.begin(); iter != surf_map.end(); iter++)
        iter->second->tras_opt(voxhess); // 计算体素 Hessian 矩阵
      tran_t += ros::Time::now().toSec()-t0;

      VOX_OPTIMIZER opt_lsv(WIN_SIZE); // 创建优化器
      t0 = ros::Time::now().toSec(); // 记录去除异常点时间
      // 移除异常点
      opt_lsv.remove_outlier(x_buf, voxhess, layer.reject_ratio);
      PLV(6) hess_vec; // Hessian 向量
      // 阻尼迭代优化
      opt_lsv.damping_iter(x_buf, voxhess, residual_cur, hess_vec, mem_cost);
      sol_t += ros::Time::now().toSec()-t0;

      // 删除体素
      for(auto iter = surf_map.begin(); iter != surf_map.end(); ++iter)
        delete iter->second;
      
      // 判断残差是否收敛
      if(loop > 0 && abs(residual_pre-residual_cur)/abs(residual_cur) < 0.05 || loop == layer.max_iter-1)
      {
        if(layer.mem_costs[thread_id] < mem_cost) layer.mem_costs[thread_id] = mem_cost;

        if(i < thread_id*part_length+left_gap_num)
          for(int j = 0; j < WIN_SIZE*(WIN_SIZE-1)/2; j++)
            layer.hessians[i*(WIN_SIZE-1)*WIN_SIZE/2+j] = hess_vec[j];

        break;
      }
      residual_pre = residual_cur;
    }

    // 生成关键帧点云
    pcl::PointCloud<PointType>::Ptr pc_keyframe(new pcl::PointCloud<PointType>);
    for(size_t j = 0; j < WIN_SIZE; j++)
    {
      t1 = ros::Time::now().toSec(); // 记录保存点云的时间
      // 计算关键帧之间的位姿变换
      Eigen::Quaterniond q_tmp;
      Eigen::Vector3d t_tmp;
      assign_qt(q_tmp, t_tmp, Quaterniond(x_buf[0].R.inverse() * x_buf[j].R),
                x_buf[0].R.inverse() * (x_buf[j].p - x_buf[0].p));

      pcl::PointCloud<PointType>::Ptr pc_oneframe(new pcl::PointCloud<PointType>);
      // 对点云进行坐标变换
      mypcl::transform_pointcloud(*src_pc[j], *pc_oneframe, t_tmp, q_tmp);
      // 将变换后的点云拼接到关键帧点云中
      pc_keyframe = mypcl::append_cloud(pc_keyframe, *pc_oneframe);
      save_t += ros::Time::now().toSec()-t1;
    }
    t0 = ros::Time::now().toSec(); // 记录降采样时间
    // 对关键帧降采样
    downsample_voxel(*pc_keyframe, 0.05);
    dsp_t += ros::Time::now().toSec()-t0;

    t0 = ros::Time::now().toSec(); // 记录保存时间
    // 将关键帧点云保存到下一层
    next_layer.pcds[i] = pc_keyframe;
    save_t += ros::Time::now().toSec()-t0;
    
    // 记录整个过程的总时间
    total_t += ros::Time::now().toSec()-t_begin;
  }

  // 如果还有未处理的尾帧
  if(layer.tail > 0)
  {
    int i = thread_id*part_length+left_gap_num;

    vector<pcl::PointCloud<PointType>::Ptr> src_pc, raw_pc; // 存储点云
    src_pc.resize(layer.last_win_size); raw_pc.resize(layer.last_win_size);

    double residual_cur = 0, residual_pre = 0; // 当前迭代和前一次迭代的残差
    vector<IMUST> x_buf(layer.last_win_size); // 存储 IMU 状态
    for(int j = 0; j < layer.last_win_size; j++)
    {
      // 将当前姿态的旋转矩阵和位置信息存储到缓冲区中
      x_buf[j].R = layer.pose_vec[i*GAP+j].q.toRotationMatrix();
      x_buf[j].p = layer.pose_vec[i*GAP+j].t;
    }

    if(layer_num != 1)
    {
      // 获取点云数据
      for(int j = i*GAP; j < i*GAP+layer.last_win_size; j++)
        src_pc[j-i*GAP] = (*layer.pcds[j]).makeShared();
    }

    size_t mem_cost = 0; // 内存消耗

    // 迭代优化
    for(int loop = 0; loop < layer.max_iter; loop++)
    {
      if(layer_num == 1)
        for(int j = i*GAP; j < i*GAP+layer.last_win_size; j++)
        {
          if(loop == 0)
          {
            // 加载原始点云数据
            pcl::PointCloud<PointType>::Ptr pc(new pcl::PointCloud<PointType>);
            mypcl::loadPCD(layer.data_path, pcd_name_fill_num, pc, j, "pcd/");
            raw_pc[j-i*GAP] = pc;
          }
          src_pc[j-i*GAP] = (*raw_pc[j-i*GAP]).makeShared();
        }

      // 创建体素 map
      unordered_map<VOXEL_LOC, OCTO_TREE_ROOT*> surf_map;

      // 遍历滑动窗口内的每个点云帧
      for(size_t j = 0; j < layer.last_win_size; j++)
      {
        // 如果需要降采样，则对点云进行体素降采样
        if(layer.downsample_size > 0) downsample_voxel(*src_pc[j], layer.downsample_size);
        // 进行体素化处理
        cut_voxel(surf_map, *src_pc[j], Quaterniond(x_buf[j].R), x_buf[j].p,
                  j, layer.voxel_size, layer.last_win_size, layer.eigen_ratio);
      }
      // 对体素进行递归切分
      for(auto iter = surf_map.begin(); iter != surf_map.end(); ++iter)
        iter->second->recut();
      
      // 计算 Hessian 矩阵
      VOX_HESS voxhess(layer.last_win_size);
      for(auto iter = surf_map.begin(); iter != surf_map.end(); iter++)
        iter->second->tras_opt(voxhess);

      VOX_OPTIMIZER opt_lsv(layer.last_win_size); // 创建优化器
      // 移除异常点
      opt_lsv.remove_outlier(x_buf, voxhess, layer.reject_ratio);
      PLV(6) hess_vec; // Hessian 向量
      // 阻尼迭代优化
      opt_lsv.damping_iter(x_buf, voxhess, residual_cur, hess_vec, mem_cost);

      // 删除体素
      for(auto iter = surf_map.begin(); iter != surf_map.end(); ++iter)
        delete iter->second;

      // 判断残差是否收敛
      if(loop > 0 && abs(residual_pre-residual_cur)/abs(residual_cur) < 0.05 || loop == layer.max_iter-1)
      {
        if(layer.mem_costs[thread_id] < mem_cost) layer.mem_costs[thread_id] = mem_cost;

        // 更新 Hessian
        for(int j = 0; j < layer.last_win_size*(layer.last_win_size-1)/2; j++)
          layer.hessians[i*(WIN_SIZE-1)*WIN_SIZE/2+j] = hess_vec[j];
        
        break;
      }
      residual_pre = residual_cur;
    }

    // 生成关键帧点云
    pcl::PointCloud<PointType>::Ptr pc_keyframe(new pcl::PointCloud<PointType>);
    for(size_t j = 0; j < layer.last_win_size; j++)
    {
      // 计算关键帧之间的位姿变换
      Eigen::Quaterniond q_tmp;
      Eigen::Vector3d t_tmp;
      assign_qt(q_tmp, t_tmp, Quaterniond(x_buf[0].R.inverse() * x_buf[j].R),
                x_buf[0].R.inverse() * (x_buf[j].p - x_buf[0].p));

      pcl::PointCloud<PointType>::Ptr pc_oneframe(new pcl::PointCloud<PointType>);
      // 对点云进行坐标变换
      mypcl::transform_pointcloud(*src_pc[j], *pc_oneframe, t_tmp, q_tmp);
      // 将变换后的点云拼接到关键帧点云中
      pc_keyframe = mypcl::append_cloud(pc_keyframe, *pc_oneframe);
    }
    // 对关键帧降采样
    downsample_voxel(*pc_keyframe, 0.05);
    // 将关键帧点云保存到下一层
    next_layer.pcds[i] = pc_keyframe;
  }

  // 打印各阶段的时间消耗
  printf("total time: %.2fs\n", total_t);
  printf("load pcd %.2fs %.2f%% | undistort pcd %.2fs %.2f%% | "
   "downsample %.2fs %.2f%% | cut voxel %.2fs %.2f%% | recut %.2fs %.2f%% | trans %.2fs %.2f%% | solve %.2fs %.2f%% | "
   "save pcd %.2fs %.2f%%\n",
    load_t, load_t/total_t*100, undis_t, undis_t/total_t*100,
    dsp_t, dsp_t/total_t*100, cut_t, cut_t/total_t*100, recut_t, recut_t/total_t*100, tran_t, tran_t/total_t*100,
    sol_t, sol_t/total_t*100, save_t, save_t/total_t*100);
}

/**
 * @brief 全局 Bundle Adjustment 优化
 * @param layer 最后一层的层数据
 */
void global_ba(LAYER& layer)
{
  int window_size = layer.pose_vec.size(); // 滑动窗口的大小
  vector<IMUST> x_buf(window_size); // 存储优化后的 IMU 状态
  for(int i = 0; i < window_size; i++)
  {
    // 将当前姿态的旋转矩阵和位置信息存储到缓冲区中
    x_buf[i].R = layer.pose_vec[i].q.toRotationMatrix();
    x_buf[i].p = layer.pose_vec[i].t;
  }

  vector<pcl::PointCloud<PointType>::Ptr> src_pc; // 源点云数据
  src_pc.resize(window_size); // 分配空间
  for(int i = 0; i < window_size; i++)
    src_pc[i] = (*layer.pcds[i]).makeShared(); // 共享点云数据

  double residual_cur = 0, residual_pre = 0; // 当前和前一次的残差
  size_t mem_cost = 0, max_mem = 0; // 内存消耗
  double dsp_t = 0, cut_t = 0, recut_t = 0, tran_t = 0, sol_t = 0, t0; // 记录各个阶段的时间
  // 迭代优化
  for(int loop = 0; loop < layer.max_iter; loop++)
  {
    std::cout<<"---------------------"<<std::endl;
    std::cout<<"Iteration "<<loop<<std::endl;

    unordered_map<VOXEL_LOC, OCTO_TREE_ROOT*> surf_map; // 创建体素 map

    for(int i = 0; i < window_size; i++)
    {
      t0 = ros::Time::now().toSec(); // 记录降采样时间
      if(layer.downsample_size > 0) downsample_voxel(*src_pc[i], layer.downsample_size); // 对点云进行降采样
      dsp_t += ros::Time::now().toSec() - t0;
      t0 = ros::Time::now().toSec(); // 记录体素化处理时间
      cut_voxel(surf_map, *src_pc[i], Quaterniond(x_buf[i].R), x_buf[i].p, i,
                layer.voxel_size, window_size, layer.eigen_ratio*2); // 体素化处理
      cut_t += ros::Time::now().toSec() - t0;
    }
    t0 = ros::Time::now().toSec(); // 记录体素递归切分时间
    for(auto iter = surf_map.begin(); iter != surf_map.end(); ++iter)
      iter->second->recut(); // 递归切分体素
    recut_t += ros::Time::now().toSec() - t0;
    
    t0 = ros::Time::now().toSec(); // 记录 Hessian 矩阵计算时间
    VOX_HESS voxhess(window_size); // 创建 Hessian 矩阵
    for(auto iter = surf_map.begin(); iter != surf_map.end(); ++iter)
      iter->second->tras_opt(voxhess); // 计算 Hessian 矩阵
    tran_t += ros::Time::now().toSec() - t0;
    
    t0 = ros::Time::now().toSec(); // 记录阻尼迭代优化时间
    VOX_OPTIMIZER opt_lsv(window_size); // 创建优化器
    opt_lsv.remove_outlier(x_buf, voxhess, layer.reject_ratio); // 移除异常点
    PLV(6) hess_vec; // Hessian 向量
    opt_lsv.damping_iter(x_buf, voxhess, residual_cur, hess_vec, mem_cost); // 阻尼迭代优化
    sol_t += ros::Time::now().toSec() - t0;

    // 删除体素
    for(auto iter = surf_map.begin(); iter != surf_map.end(); ++iter)
      delete iter->second;
    
    // 打印残差
    cout<<"Residual absolute: "<<abs(residual_pre-residual_cur)<<" | "
      <<"percentage: "<<abs(residual_pre-residual_cur)/abs(residual_cur)<<endl;
    
    // 判断是否收敛
    if(loop > 0 && abs(residual_pre-residual_cur)/abs(residual_cur) < 0.05 || loop == layer.max_iter-1)
    {
      if(max_mem < mem_cost) max_mem = mem_cost;
      #ifdef FULL_HESS
      for(int i = 0; i < window_size*(window_size-1)/2; i++)
        layer.hessians[i] = hess_vec[i]; // 保存 Hessian 矩阵
      #else
      // 简化 Hessian 矩阵输出
      for(int i = 0; i < window_size-1; i++)
      {
        Matrix6d hess = Hess_cur.block(6*i, 6*i+6, 6, 6);
        for(int row = 0; row < 6; row++)
          for(int col = 0; col < 6; col++)
            hessFile << hess(row, col) << ((row*col==25)?"":" ");
        if(i < window_size-2) hessFile << "\n";
      }
      #endif
      break;
    }
    residual_pre = residual_cur;
  }
  // 更新姿态
  for(int i = 0; i < window_size; i++)
  {
    layer.pose_vec[i].q = Quaterniond(x_buf[i].R);
    layer.pose_vec[i].t = x_buf[i].p;
  }
  // 打印各阶段的时间消耗
  printf("Downsample: %f, Cut: %f, Recut: %f, Tras: %f, Sol: %f\n", dsp_t, cut_t, recut_t, tran_t, sol_t);
}

/**
 * @brief 分配线程并执行并行计算
 * @param layer 当前层数据
 * @param next_layer 下一层数据，用于存储当前层处理后的结果
 */
void distribute_thread(LAYER& layer, LAYER& next_layer)
{
  int& thread_num = layer.thread_num;  // 获取当前层的线程数
  double t0 = ros::Time::now().toSec(); // 记录线程分配的开始时间

  // 为每个线程分配并行任务
  for(int i = 0; i < thread_num; i++)
  {
    // 如果不是最后一个线程，启动 `parallel_comp` 进行并行计算
    if(i < thread_num-1)
      layer.mthreads[i] = new thread(parallel_comp, ref(layer), i, ref(next_layer));
    else
      // 如果是最后一个线程，启动 `parallel_tail` 处理剩余的部分
      layer.mthreads[i] = new thread(parallel_tail, ref(layer), i, ref(next_layer));
  }
  // 此处可以打印线程分配时间
  // printf("Thread distribution time: %f\n", ros::Time::now().toSec()-t0);

  t0 = ros::Time::now().toSec(); // 记录线程 join 的开始时间

  // 等待所有线程完成计算
  for(int i = 0; i < thread_num; i++)
  {
    layer.mthreads[i]->join(); // 主线程等待子线程完成
    delete layer.mthreads[i];  // 释放线程对象
  }
  // 此处可以打印线程结束时间
  // printf("Thread join time: %f\n", ros::Time::now().toSec()-t0);
}

/**
 * @brief 主函数，初始化 ROS 节点并执行 HBA 优化流程
 */
int main1(int argc, char** argv)
{
  // 初始化 ROS 节点，节点名称为 "hba"
  ros::init(argc, argv, "hba");
  ros::NodeHandle nh("~"); // 获取 ROS 节点句柄，"~" 表示私有参数命名空间

  int total_layer_num, thread_num;  // 总层数和线程数
  string data_path;  // 数据路径

  // 从 ROS 参数服务器获取参数
  nh.getParam("total_layer_num", total_layer_num); // 获取总层数
  nh.getParam("pcd_name_fill_num", pcd_name_fill_num); // 获取 PCD 文件名的填充值
  nh.getParam("data_path", data_path); // 获取数据路径
  nh.getParam("thread_num", thread_num); // 获取线程数

  // 初始化 HBA 类，传入总层数、数据路径和线程数
  HBA hba(total_layer_num, data_path, thread_num);

  // 对每一层进行处理，直到最后一层
  for(int i = 0; i < total_layer_num-1; i++)
  {
    std::cout << "---------------------" << std::endl;
    // 为当前层和下一层分配线程，并行处理
    distribute_thread(hba.layers[i], hba.layers[i+1]);
    // 更新下一层的状态
    hba.update_next_layer_state(i);
  }

  // 对最后一层执行全局 BA 优化
  global_ba(hba.layers[total_layer_num-1]);

  // 执行位姿图优化
  hba.pose_graph_optimization();

  // 打印优化完成消息
  printf("iteration complete\n");

  // 可视化
  cout<<"push enter to view"<<endl;
  getchar();
  
  return 0;
}

// 封装点云和位姿发布的函数
void publishPointCloudAndPose(
    string data_path,
    ros::Publisher& pub_map,
    ros::Publisher& pub_pose,
    ros::Publisher& pub_trajectory,
    ros::Publisher& pub_pose_number,
    double downsample_size,
    double marker_size,
    int i_th,
    std::string input_json_file)
{
    sensor_msgs::PointCloud2 cloudMsg;
    geometry_msgs::PoseArray parray;
    parray.header.frame_id = "map";
    parray.header.stamp = ros::Time::now();
    std::cout << "Jack" <<std::endl;
    // 读取最新的位姿信息
    std::string filename;
    if(i_th==0){
      filename = data_path + input_json_file;
    }else{
      filename = data_path + "pose_" + std::to_string(i_th) + ".json";
    }
    std::vector<mypcl::pose> pose_vec = mypcl::read_pose(filename);
    pcl::PointCloud<PointType>::Ptr pc_surf(new pcl::PointCloud<PointType>);

    visualization_msgs::MarkerArray markerArray;

    size_t pose_size = pose_vec.size();
    ros::Time cur_t = ros::Time::now();

    for (size_t i = 0; i < pose_size; i++)
    {
        mypcl::loadPCD(data_path + "pcd/", pcd_name_fill_num, pc_surf, i);

        pcl::PointCloud<PointType>::Ptr pc_filtered(new pcl::PointCloud<PointType>);
        pc_filtered->resize(pc_surf->points.size());
        int cnt = 0;
        for(size_t j = 0; j < pc_surf->points.size(); j++)
        {
          pc_filtered->points[cnt] = pc_surf->points[j];
          cnt++;
        }
        pc_filtered->resize(cnt);
        
        mypcl::transform_pointcloud(*pc_filtered, *pc_filtered, pose_vec[i].t, pose_vec[i].q);
        downsample_voxel(*pc_filtered, downsample_size);

        pcl::toROSMsg(*pc_filtered, cloudMsg);
        cloudMsg.header.frame_id = "map";
        cloudMsg.header.stamp = cur_t;
        pub_map.publish(cloudMsg);

        geometry_msgs::Pose apose;
        apose.orientation.w = pose_vec[i].q.w();
        apose.orientation.x = pose_vec[i].q.x();
        apose.orientation.y = pose_vec[i].q.y();
        apose.orientation.z = pose_vec[i].q.z();
        apose.position.x = pose_vec[i].t(0);
        apose.position.y = pose_vec[i].t(1);
        apose.position.z = pose_vec[i].t(2);
        parray.poses.push_back(apose);
        pub_pose.publish(parray);

        // // 发布轨迹
        // visualization_msgs::Marker marker;
        // marker.header.frame_id = "map";
        // marker.header.stamp = cur_t;
        // marker.ns = "basic_shapes";
        // marker.id = i;
        // marker.type = visualization_msgs::Marker::SPHERE;
        // marker.pose.position.x = pose_vec[i].t(0);
        // marker.pose.position.y = pose_vec[i].t(1);
        // marker.pose.position.z = pose_vec[i].t(2);
        // pose_vec[i].q.normalize();
        // marker.pose.orientation.x = pose_vec[i].q.x();
        // marker.pose.orientation.y = pose_vec[i].q.y();
        // marker.pose.orientation.z = pose_vec[i].q.x();
        // marker.pose.orientation.w = pose_vec[i].q.w();
        // marker.scale.x = marker_size;
        // marker.scale.y = marker_size;
        // marker.scale.z = marker_size;
        // marker.color.r = float(1 - float(i) / pose_size);
        // marker.color.g = float(float(i) / pose_size);
        // marker.color.b = float(float(i) / pose_size);
        // marker.color.a = 1.0;
        // marker.lifetime = ros::Duration();
        // pub_trajectory.publish(marker);

        // // 发布位姿编号
        // visualization_msgs::Marker marker_txt;
        // marker_txt.header.frame_id = "map";
        // marker_txt.header.stamp = cur_t;
        // marker_txt.ns = "marker_txt";
        // marker_txt.id = i;
        // marker_txt.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        // std::ostringstream str;
        // str << i;
        // marker_txt.text = str.str();
        // marker_txt.action = visualization_msgs::Marker::ADD;
        // marker_txt.pose.position.x = pose_vec[i].t(0) + marker_size;
        // marker_txt.pose.position.y = pose_vec[i].t(1) + marker_size;
        // marker_txt.pose.position.z = pose_vec[i].t(2);
        // marker_txt.pose.orientation.x = pose_vec[i].q.x();
        // marker_txt.pose.orientation.y = pose_vec[i].q.y();
        // marker_txt.pose.orientation.z = pose_vec[i].q.x();
        // marker_txt.pose.orientation.w = 1.0;
        // marker_txt.scale.x = marker_size;
        // marker_txt.scale.y = marker_size;
        // marker_txt.scale.z = marker_size;
        // marker_txt.color.r = 1.0f;
        // marker_txt.color.g = 1.0f;
        // marker_txt.color.b = 1.0f;
        // marker_txt.color.a = 1.0;
        // marker_txt.lifetime = ros::Duration();
        // if (i % GAP == 0)
        //     markerArray.markers.push_back(marker_txt);
        // pub_pose_number.publish(markerArray);

        // ros::Duration(0.000001).sleep();
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "hba_iterative");
    ros::NodeHandle nh("~");

    // 从参数服务器获取参数
    std::string data_path, input_json_file;
    double downsample_size_vis, marker_size, downsample_size_BA;
    int total_layer_num, thread_num, stride, win_size;
    int iter_num = 0;

    nh.getParam("data_path", data_path);
    nh.getParam("total_layer_num", total_layer_num);
    nh.getParam("thread_num", thread_num);
    nh.getParam("downsample_size_BA", downsample_size_BA);  // 降采样
    nh.getParam("downsample_size_vis", downsample_size_vis);
    nh.getParam("marker_size", marker_size);
    nh.getParam("iter_num", iter_num);
    nh.getParam("input_json_file", input_json_file);  // 输入的json，如果前面有pgo的话，应当以pgo的输出json为输入

    // 声明ROS发布器
    ros::Publisher pub_map;
    ros::Publisher pub_pose;
    ros::Publisher pub_trajectory;
    ros::Publisher pub_pose_number;
    ros::Publisher json_index_pub = nh.advertise<std_msgs::Int32>("/json_index", 10);

    char user_input = '\n';  // 用于接收用户输入
    int iteration_count = 0;

    // 发布json的index，用于另一节点的可视化
    std_msgs::Int32 indexmsg; // 创建消息对象
    indexmsg.data = 0; // 随机生成一个整数（0-99）
    json_index_pub.publish(indexmsg); // 发布消息
    
    // 开始循环，处理不同的迭代逻辑
    while (ros::ok())
    {
      std::cout << "==========================" << std::endl;
      std::cout << "Iteration: " << iteration_count << std::endl;

      ros::Time start = ros::Time::now();
      ros::Time time0;

      // 初始化 HBA 类，加载点云和位姿数据
      time0 = ros::Time::now();
      HBA hba(total_layer_num, data_path, thread_num, iteration_count, input_json_file);
      std::cout << "[hba]: total:" << (ros::Time::now() - start).toSec() << " s" << "\t\tStep:" << (ros::Time::now() - time0).toSec() << " s" << std::endl;

      // 对每一层进行处理，直到最后一层
      for (int i = 0; i < total_layer_num - 1; i++)
      {
          std::cout << "Layer " << i << " processing..." << std::endl;
          time0 = ros::Time::now();
          distribute_thread(hba.layers[i], hba.layers[i + 1]);
          hba.update_next_layer_state(i);
          std::cout << "[layer]: total:" << (ros::Time::now() - start).toSec() << " s" << "\t\tStep:" << (ros::Time::now() - time0).toSec() << " s" << std::endl;
      }

      // 对最后一层执行全局 BA 优化
      time0 = ros::Time::now();
      global_ba(hba.layers[total_layer_num - 1]);
      std::cout << "[global]: total:" << (ros::Time::now() - start).toSec() << " s" << "\t\tStep:" << (ros::Time::now() - time0).toSec() << " s" << std::endl;

      // 执行位姿图优化
      time0 = ros::Time::now();
      hba.pose_graph_optimization(iteration_count+1); //+1指的是，存储json的序号和优化次数是一样多的
      std::cout << "[PGO]: total:" << (ros::Time::now() - start).toSec() << " s" << "\t\tStep:" << (ros::Time::now() - time0).toSec() << " s" << std::endl;

      // 发布json的index，用于另一节点的可视化
      indexmsg.data = iteration_count+1; // 随机生成一个整数（0-99）
      json_index_pub.publish(indexmsg); // 发布消息

      // 检查是否达到 iter_num 次数
      if (iter_num > 0 && iteration_count > iter_num)
      {
          std::cout << "Reached maximum iterations (" << iter_num << "). Exiting." << std::endl;
          break;
      }

      // iter_num 为 0 时，等待用户输入决定是否继续
      if (iter_num == 0)
      {
          std::cout << "Press 'Enter' to continue to next iteration, or 'q' to quit: ";
          user_input = std::getchar();
          if (user_input == 'q' || user_input == 'Q')
          {
              std::cout << "Exiting iterative optimization." << std::endl;
              break;
          }
      }

      iteration_count++;
    }

    return 0;
}