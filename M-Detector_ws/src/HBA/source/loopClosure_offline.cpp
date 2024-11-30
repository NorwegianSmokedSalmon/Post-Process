#include <ros/ros.h>
#include "mypcl.hpp"
#include "tools.hpp"

#include <gtsam/geometry/Pose3.h> // GTSAM 位姿表示
#include <gtsam/slam/PriorFactor.h> // GTSAM 先验因子
#include <gtsam/slam/BetweenFactor.h> // GTSAM 两帧之间的因子
#include <gtsam/nonlinear/Values.h> // GTSAM 非线性方程组的结果
#include <gtsam/nonlinear/ISAM2.h> // GTSAM 增量式优化

#include <pcl/registration/gicp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>  // Include for TF broadcasting
#include <pcl/filters/voxel_grid.h>
#include <std_msgs/Int32.h>

using namespace std;
using namespace mypcl;
using PointType = pcl::PointXYZ;

void pose_graph_optimization(vector<pose> vec_pose, gtsam::Pose3 T_start_end, string data_path)
{
    // 设置图
    gtsam::Values initial;
    gtsam::NonlinearFactorGraph graph;
    // 设置先验置信度
    gtsam::Vector Prior_Vector6(6);
    Prior_Vector6 << 1e-6, 1e-6, 1e-6, 1e-8, 1e-8, 1e-8;
    gtsam::noiseModel::Diagonal::shared_ptr priorModel = gtsam::noiseModel::Diagonal::Variances(Prior_Vector6);
    // 设置帧间估计置信度
    gtsam::Vector odom_Vector6(6);
    odom_Vector6 << 1e-6, 1e-6, 1e-6, 1e-8, 1e-8, 1e-8;
    gtsam::noiseModel::Diagonal::shared_ptr odomModel = gtsam::noiseModel::Diagonal::Variances(odom_Vector6);
    // 设置回环置信度
    gtsam::Vector loopClosure_Vector6(6);
    loopClosure_Vector6 << 1e-6, 1e-6, 1e-6, 1e-8, 1e-8, 1e-8;
    gtsam::noiseModel::Diagonal::shared_ptr loopClosureModel = gtsam::noiseModel::Diagonal::Variances(loopClosure_Vector6);

    for(uint i = 0; i < vec_pose.size(); i++)
    {
        initial.insert(i, gtsam::Pose3(gtsam::Rot3(vec_pose[i].q.toRotationMatrix()), gtsam::Point3(vec_pose[i].t)));

        if(i == 0)
        {
            graph.add(gtsam::PriorFactor<gtsam::Pose3>(i, gtsam::Pose3(gtsam::Rot3(vec_pose[i].q.toRotationMatrix()),gtsam::Point3(vec_pose[i].t)), priorModel));   // 固定第一帧位置不优化
            continue;  // 遍历到第一帧时不需要加约束
        }

        // 计算两帧之间的相对位姿
        gtsam::Pose3 newPose(gtsam::Rot3(vec_pose[i].q.toRotationMatrix()), gtsam::Point3(vec_pose[i].t));
        gtsam::Pose3 oldPose(gtsam::Rot3(vec_pose[i-1].q.toRotationMatrix()), gtsam::Point3(vec_pose[i-1].t));
        gtsam::Pose3 T_new_old = oldPose.between(newPose);

        // 添加因子到因子图
        gtsam::NonlinearFactor::shared_ptr factor(new gtsam::BetweenFactor<gtsam::Pose3>(i-1, i, T_new_old, odomModel));
        graph.push_back(factor);
    }

    // 加入回环因子（T_start_end是用gicp算出来的）
    gtsam::NonlinearFactor::shared_ptr factor(new gtsam::BetweenFactor<gtsam::Pose3>(0, vec_pose.size()-1, T_start_end, loopClosureModel));
    graph.push_back(factor);

    // 设置 ISAM2 参数
    gtsam::ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.01;
    parameters.relinearizeSkip = 1;

    // 创建 ISAM2 优化器并进行更新
    gtsam::ISAM2 isam(parameters);
    isam.update(graph, initial);

    // 计算最终优化结果
    gtsam::Values results = isam.calculateEstimate();

    cout << "vertex size " << results.size() << endl;

    // 将优化后的位姿保存到 vec_pose_opt 中
    vector<pose> vec_pose_opt;
    vec_pose_opt.resize(vec_pose.size());
    for(uint i = 0; i < results.size(); i++)
    {
        gtsam::Pose3 pose = results.at(i).cast<gtsam::Pose3>();
        assign_qt(vec_pose_opt[i].q, vec_pose_opt[i].t, Eigen::Quaterniond(pose.rotation().matrix()), pose.translation());
    }

    // 保存优化后的位姿
    mypcl::write_pose(vec_pose_opt, data_path, -1);
    printf("pgo complete\n");
}

// Function to create a 4x4 transformation matrix from translation and quaternion
Eigen::Matrix4d createTransformationMatrix(const Eigen::Vector3d &translation, const Eigen::Quaterniond &quaternion) {
    Eigen::Matrix4d transformation = Eigen::Matrix4d::Identity();
    transformation.block<3, 3>(0, 0) = quaternion.toRotationMatrix();
    transformation.block<3, 1>(0, 3) = translation;
    return transformation;
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "loopClosure_offline");
    ros::NodeHandle nh("~");

    int stack_size;
    std::string data_path;
    double leaf_size;

    nh.getParam("data_path", data_path);
    nh.getParam("stack_size", stack_size);
    nh.getParam("leaf_size", leaf_size);

    // Publishers for point clouds
    ros::Publisher pub_start = nh.advertise<sensor_msgs::PointCloud2>("start_frame_stack", 1);
    ros::Publisher pub_end = nh.advertise<sensor_msgs::PointCloud2>("end_frame_stack", 1);
    ros::Publisher pub_aligned = nh.advertise<sensor_msgs::PointCloud2>("aligned_cloud", 1);
    ros::Publisher json_index_pub = nh.advertise<std_msgs::Int32>("/json_index", 10);

    // TF broadcaster for publishing frame transforms
    tf::TransformBroadcaster tf_broadcaster;

    // Load pose vector
    std::string filename = data_path + "pose.json";
    std::vector<mypcl::pose> pose_vec = mypcl::read_pose(filename);
    size_t pose_size = pose_vec.size();
    std::cout << "[Debug] pose_vec loaded! => Size: " << pose_size << std::endl;

    // Initialize point cloud pointers
    pcl::PointCloud<PointType>::Ptr pc_tmp(new pcl::PointCloud<PointType>);
    pcl::PointCloud<PointType>::Ptr start_frame_stack(new pcl::PointCloud<PointType>);
    pcl::PointCloud<PointType>::Ptr end_frame_stack(new pcl::PointCloud<PointType>);

    for (size_t i = 0; i < stack_size; i++) {
        // Stack to start_frame_stack
        pc_tmp->clear();
        mypcl::loadPCD(data_path + "pcd/", pc_tmp, i);
        mypcl::transform_pointcloud(*pc_tmp, *pc_tmp, pose_vec[i].t, pose_vec[i].q);
        *start_frame_stack += *pc_tmp;

        // Stack to end_frame_stack
        pc_tmp->clear();
        int index = pose_size - 1 - i;
        mypcl::loadPCD(data_path + "pcd/", pc_tmp, index);
        mypcl::transform_pointcloud(*pc_tmp, *pc_tmp, pose_vec[index].t, pose_vec[index].q);
        *end_frame_stack += *pc_tmp;
    }

    downsample_voxel_ckx(*start_frame_stack, leaf_size);
    downsample_voxel_ckx(*end_frame_stack, leaf_size);


    // Transform to local coordinate frames
    Eigen::Matrix4d start_transform = createTransformationMatrix(pose_vec[0].t, pose_vec[0].q).inverse();
    Eigen::Matrix4d end_transform = createTransformationMatrix(pose_vec[pose_size - 1].t, pose_vec[pose_size - 1].q).inverse();

    pcl::transformPointCloud(*start_frame_stack, *start_frame_stack, start_transform);
    pcl::transformPointCloud(*end_frame_stack, *end_frame_stack, end_transform);

    // Perform GICP alignment to get rotation and translation
    pcl::GeneralizedIterativeClosestPoint<PointType, PointType> gicp;
    gicp.setInputSource(start_frame_stack);
    gicp.setInputTarget(end_frame_stack);

    pcl::PointCloud<PointType> aligned;
    gicp.align(aligned);

    Eigen::Matrix3d rotation;
    Eigen::Vector3d translation;

    if (gicp.hasConverged()) {
        std::cout << "GICP converged." << std::endl;

        // Get the transformation matrix
        Eigen::Matrix4d transformation = gicp.getFinalTransformation().cast<double>();
        rotation = transformation.block<3, 3>(0, 0);
        translation = transformation.block<3, 1>(0, 3);
        std::cout << "Rotation:\n" << rotation << std::endl;
        std::cout << "Translation:\n" << translation.transpose() << std::endl;
    } else {
        std::cout << "GICP did not converge. Exit!" << std::endl;
        return 0;
    }

    gtsam::Pose3 T_start_end = gtsam::Pose3(gtsam::Rot3(rotation),gtsam::Point3(translation)).inverse();
    pose_graph_optimization(pose_vec, T_start_end, data_path);
    // 发布json的index，用于另一节点的可视化
    std_msgs::Int32 indexmsg; // 创建消息对象
    indexmsg.data = 0; // 随机生成一个整数（0-99）
    json_index_pub.publish(indexmsg); // 发布消息
    return 0;
}