#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <boost/bind.hpp>
#include <fstream>
#include <sys/stat.h>  // 用于检查和创建文件夹
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_listener.h>
#include <vector>
#include <string>
#include <dirent.h>  // 用于读取文件夹内容
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "std_msgs/Bool.h"
#include "std_msgs/Int32.h"
#include <pcl/filters/voxel_grid.h>
#include <chrono>
#include <thread>
#include <std_msgs/String.h>

typedef pcl::PointXYZ PointType;
int Data_volume_ = 0;

ros::Publisher cloud_pub_;
ros::Subscriber new_iteration_sub_, vis_iteration_sub_;
std::string pcd_file_;

std::vector<Eigen::Vector3f> odom_pos_vec_;
std::vector<Eigen::Quaternionf> odom_rot_vec_;
pcl::PointCloud<PointType>::Ptr transformed_cloud(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr merged_cloud(new pcl::PointCloud<PointType>());
std::vector<pcl::PointCloud<PointType>::Ptr> cloud_vec;
std::vector<pcl::PointCloud<PointType>::Ptr> cloud_trans_vec;
int big_cloud_;
int vis_iteration_ = -1;
int Cycle_limit = -1;

std::string json_file;
std::string json_name;

void newIterationCallback(const std_msgs::Int32::ConstPtr& msg)
{
    ROS_WARN("Get new iteration: %d", msg->data);
}

void visIterationCallback(const std_msgs::String::ConstPtr& msg)
{
    vis_iteration_ = vis_iteration_ + 1;
    ROS_WARN("Vis iteration: %d", msg->data);
    ROS_INFO("Received: %s", msg->data.c_str());  // 打印接收到的字符串
    ROS_INFO("Received: %s", msg->data.c_str());  // 打印接收到的字符串
    ROS_INFO("Received: %s", msg->data.c_str());  // 打印接收到的字符串
    ROS_INFO("Received: %s", msg->data.c_str());  // 打印接收到的字符串
    ROS_INFO("Received: %s", msg->data.c_str());  // 打印接收到的字符串
    json_name = msg->data;
    std::cout << "Project root directory: " << ROOT_DIR << std::endl;
    std::cout << "Project root directory: " << ROOT_DIR << std::endl;
    
}

// 读取 Odom 数据
bool readOdomFromTxt(const std::string &file_path, int &num) {
    std::ifstream file(file_path);
    num = 0;
    if (!file.is_open()) {
        ROS_ERROR("Could not open file: %s", file_path.c_str());
        return false;
    }
    std::string line;
    while (std::getline(file, line)) {
        num++;
        std::istringstream iss(line);
        float tx, ty, tz, qw, qx, qy, qz;
        if (iss >> tx >> ty >> tz >> qw >> qx >> qy >> qz) {
            // 创建平移向量
            Eigen::Vector3f translation(tx, ty, tz);

            // 创建四元数
            Eigen::Quaternionf rotation(qw, qx, qy, qz);

            odom_pos_vec_.push_back(translation);
            odom_rot_vec_.push_back(rotation);

        } else {
            ROS_WARN("Invalid line format in file: %s", line.c_str());
        }
    }
    return true;
}


// 读取 PCD 文件并返回点云
bool readPointCloud(const std::string &file_path, pcl::PointCloud<PointType>::Ptr &cloud) {
    if (pcl::io::loadPCDFile<PointType>(file_path, *cloud) == -1) {
        ROS_ERROR("Couldn't read file %s", file_path.c_str());
        return false;
    }
    return true;
}

// 将点云转换到世界坐标系
void transformPointCloud(pcl::PointCloud<PointType>::Ptr &cloud,int id) {

    Eigen::Vector3f pos = odom_pos_vec_[id];
    Eigen::Quaternionf rot = odom_rot_vec_[id];
    // std::cout << pos.transpose() << std::endl;
    transformed_cloud->points.resize(cloud->points.size());
    for(int i=0; i < cloud->points.size(); i++)
    {
        PointType point = cloud->points[i];
        Eigen::Vector3f e_p{point.x, point.y, point.z};
        e_p = rot * e_p + pos;
        point.x = e_p[0];
        point.y = e_p[1];
        point.z = e_p[2];
        transformed_cloud->points[i] = point;
    }
}

void publishPointCloud(ros::Publisher &pub, const pcl::PointCloud<PointType>::Ptr &cloud) {

        sensor_msgs::PointCloud2 cloud_msg;
        cloud_msg.data.clear();  // 清空点云数据
        pcl::toROSMsg(*cloud, cloud_msg);
        cloud_msg.header.stamp = ros::Time::now();
        cloud_msg.header.frame_id = "world";  // 或者其他适合的坐标系名称
        // ROS_WARN("PUB_cloud");
        // 发布点云
        pub.publish(cloud_msg);
        ros::Duration(0.01).sleep();
}

void publishPointClouds(ros::Publisher &pub, const std::vector<pcl::PointCloud<PointType>::Ptr> &cloud_vec) {
    // for (size_t i = 0; i < cloud_vec.size(); i++) {
        // 创建 PointCloud2 消息
        sensor_msgs::PointCloud2 cloud_msg;
        cloud_msg.data.clear();  // 清空点云数据
        pcl::toROSMsg(*cloud_vec[big_cloud_], cloud_msg);
        cloud_msg.header.stamp = ros::Time::now();
        cloud_msg.header.frame_id = "world";  // 或者其他适合的坐标系名称
        ROS_WARN("PUB_big_cloud");
        // 发布点云
        pub.publish(cloud_msg);

    // }
}

bool mergePointClouds(pcl::PointCloud<PointType>::Ptr &clouds, pcl::PointCloud<PointType>::Ptr &merged_cloud, size_t max_size = 1000000) {
    // 检查 merged_cloud 和 clouds 的总点云数量是否超过 max_size
    if (merged_cloud->points.size() + clouds->points.size() > max_size) {
        // 如果合并后超过了最大点云数量，返回 true
        return true;
    }

    // 如果没有超过最大大小，则将 clouds 点云添加到 merged_cloud 中
    *merged_cloud += *clouds;
    // ROS_INFO("LOADING");

    // 返回 ，false表示没有超过最大大小
    return false;
}