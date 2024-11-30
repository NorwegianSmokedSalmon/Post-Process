#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <boost/filesystem.hpp>
#include <sstream>
#include <iostream>
#include <deque>
#include <cmath>
#include <cstdlib>  // for exit()
#include <iomanip>

#define BLUE_TEXT "\033[34m"
#define RESET_COLOR "\033[0m"

// 定义时间戳匹配阈值，单位秒
#define TIME_THRESHOLD 0.000001  // 时间戳差异阈值，默认为0.05秒

// 定义最大 PCD 文件数量，设为 0 表示没有限制
#define MAX_PCD_FILES 0   // 设置为0表示无限制，其他数字表示限制的最大数量

std::string trans; // 用于存储 trans 文件夹路径
std::string bag_file;
std::string cloud;
std::string Odometry;

//wlj add, for TUM with timestamp
bool saveTum = false;


int main(int argc, char** argv)
{
    setlocale(LC_ALL, "en_US.UTF-8");

    // 初始化 ROS 节点
    ros::init(argc, argv, "bag2pcd");
    ros::NodeHandle nh;

    // 获取 trans 文件夹的名称（相对路径）
    if (!nh.hasParam("trans_name")) {
        ROS_ERROR(BLUE_TEXT "请指定 trans 文件夹路径。" RESET_COLOR);
        return 1;
    }
    nh.getParam("trans_name", trans);  // 获取 trans 文件夹的名称
    trans = "../../../" + trans; 
    // 检查是否提供了 .bag 文件的路径
    if (!nh.hasParam("bag_file"))
    {
        if (argc < 2) {
            ROS_ERROR(BLUE_TEXT "请指定一个 .bag 文件路径。" RESET_COLOR);
            return 1;
        }
        bag_file = argv[1];
    } else {
        nh.getParam("bag_file", bag_file);
    }

    // 打开 .bag 文件
    rosbag::Bag bag;
    try {
        bag.open(bag_file, rosbag::bagmode::Read);
    } catch (const rosbag::BagIOException& e) {
        ROS_ERROR(BLUE_TEXT "无法打开 bag 文件: %s" RESET_COLOR, e.what());
        return 1;
    }

    std::vector<std::string> topics; 
    if (!nh.hasParam("cloud_name") || !nh.hasParam("odom_name"))
    {
        topics.push_back(std::string("/cloud_registered_body"));         
        topics.push_back(std::string("/Wxx_Odometry"));        
    } else {
        nh.getParam("cloud_name", cloud);
        nh.getParam("odom_name", Odometry);
        topics.push_back(cloud);         
        topics.push_back(Odometry);    
    }

    // 声明保存点云和里程计数据的队列
    std::deque<sensor_msgs::PointCloud2::ConstPtr> pointcloud_queue;
    std::deque<nav_msgs::Odometry::ConstPtr> odometry_queue;

    // 创建 view，用于读取 bag 中的 topic
    rosbag::View view(bag, rosbag::TopicQuery(topics));
    rosbag::View::iterator it;
    int pcd_counter = 0;

    // 创建 trans 文件夹及其内部文件夹
    boost::filesystem::path trans_folder(trans);  // 获取 trans 文件夹路径（相对路径）
    boost::filesystem::path pcd_folder = std::string(ROOT_DIR) / trans_folder / "pcd";  // 创建 pcd 子文件夹
    boost::filesystem::path pose_file_path = std::string(ROOT_DIR) / trans_folder / "pose.json";  // pose.json 文件路径

    // 确保 trans 文件夹存在
    if (!boost::filesystem::exists(trans_folder)) {
        try {
            boost::filesystem::create_directory(std::string(ROOT_DIR) / trans_folder);  // 如果 trans 文件夹不存在，创建它
            ROS_INFO("Created directory: %s", trans_folder.string().c_str());
        } catch (const boost::filesystem::filesystem_error& e) {
            ROS_ERROR("Failed to create trans folder: %s", e.what());
            return 1;
        }
    }

    // 确保 pcd 文件夹存在
    if (!boost::filesystem::exists(pcd_folder)) {
        try {
            boost::filesystem::create_directory(pcd_folder);  // 如果 pcd 文件夹不存在，创建它
            ROS_INFO("Created directory: %s", pcd_folder.string().c_str());
        } catch (const boost::filesystem::filesystem_error& e) {
            ROS_ERROR("Failed to create pcd folder: %s", e.what());
            return 1;
        }
    }

    // 打开 pose.json 文件用于保存里程计数据
    std::ofstream pose_file(pose_file_path.string());
    if (!pose_file.is_open()) {
        ROS_ERROR("Failed to open pose.json for writing.");
        return 1;
    }

    // 使用迭代器的方式遍历，注意：每一个迭代器为一帧数据
    for (it = view.begin(); it != view.end(); it++) {
        auto msg = *it;
        std::string topic = msg.getTopic();
       
        if (topic == cloud) {
            // 处理点云数据
            sensor_msgs::PointCloud2::ConstPtr cloud_msg = msg.instantiate<sensor_msgs::PointCloud2>();
            if (cloud_msg != NULL) {
                ROS_INFO("Received PointCloud2 message");

                // 保存点云数据到队列
                pointcloud_queue.push_back(cloud_msg);
            }
        }
        if (topic == Odometry) {
            // 处理里程计数据
            nav_msgs::Odometry::ConstPtr odom_msg = msg.instantiate<nav_msgs::Odometry>();
            if (odom_msg != NULL) {
                ROS_INFO("Received Odometry message");

                // 保存里程计数据到队列
                odometry_queue.push_back(odom_msg);
            }
        }

        // 如果队列中有点云数据和里程计数据，则进行时间戳匹配
        while (!pointcloud_queue.empty() && !odometry_queue.empty() && ros::ok()) {
            sensor_msgs::PointCloud2::ConstPtr cloud_msg = pointcloud_queue.front();
            nav_msgs::Odometry::ConstPtr odom_msg = odometry_queue.front();

            // 计算时间戳差异
            double time_diff = std::fabs((cloud_msg->header.stamp - odom_msg->header.stamp).toSec());
            ROS_INFO("time_diff: %f", time_diff);

            // 如果时间戳差异小于阈值，认为是匹配的消息
            if (time_diff <= TIME_THRESHOLD) {
                ROS_INFO("Synchronizing messages: PointCloud2 and Odometry");

                // 保存点云数据为 PCD 文件
                std::stringstream ss;
                ss << pcd_folder.string() << "/" << pcd_counter++ << ".pcd";
                std::string filename = ss.str();

                pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
                pcl::fromROSMsg(*cloud_msg, *cloud);

                if (pcl::io::savePCDFileASCII(filename, *cloud) == -1) {
                    ROS_ERROR("Failed to save PCD file");
                } else {
                    ROS_INFO("Saved PCD file: %s", filename.c_str());
                }
                // std::cout << "Original timestamp: " << odom_msg->header.stamp << std::endl;
                // 输出里程计数据到 pose.json
                if(saveTum){
                    pose_file << std::fixed << std::setprecision(9)
                          <<  odom_msg->header.stamp.toSec() << " "
                          << odom_msg->pose.pose.position.x << " "
                          << odom_msg->pose.pose.position.y << " "
                          << odom_msg->pose.pose.position.z << " "
                          << odom_msg->pose.pose.orientation.w << " "
                          << odom_msg->pose.pose.orientation.x << " "
                          << odom_msg->pose.pose.orientation.y << " "
                          << odom_msg->pose.pose.orientation.z << std::endl;
                }
                // else{
                //     pose_file << odom_msg->pose.pose.position.x << " "
                //           << odom_msg->pose.pose.position.y << " "
                //           << odom_msg->pose.pose.position.z << " "
                //           << odom_msg->pose.pose.orientation.w << " "
                //           << odom_msg->pose.pose.orientation.x << " "
                //           << odom_msg->pose.pose.orientation.y << " "
                //           << odom_msg->pose.pose.orientation.z << std::endl;
                // }
                else{
                    pose_file << odom_msg->pose.pose.position.x << " "
                          << odom_msg->pose.pose.position.y << " "
                          << odom_msg->pose.pose.position.z << " "
                          << odom_msg->pose.pose.orientation.w << " "
                          << odom_msg->pose.pose.orientation.x << " "
                          << odom_msg->pose.pose.orientation.y << " "
                          << odom_msg->pose.pose.orientation.z << std::endl;
                }

                

                // 删除已处理的消息
                pointcloud_queue.pop_front();
                odometry_queue.pop_front();

                // 限制最大 PCD 文件数量
                if (MAX_PCD_FILES > 0 && pcd_counter >= MAX_PCD_FILES) {
                    ROS_INFO("Reached maximum PCD file limit");
                    exit(1);  // 达到最大文件数，退出
                }
            }
            else if (cloud_msg->header.stamp < odom_msg->header.stamp) {
                // 如果点云消息的时间戳早于里程计消息，移除点云消息
                pointcloud_queue.pop_front();
            }
            else {
                // 如果里程计消息的时间戳早于点云消息，移除里程计消息
                odometry_queue.pop_front();
            }
        }
    }

    // 关闭 pose.json 文件
    pose_file.close();

    // 关闭 .bag 文件
    bag.close();
    ROS_WARN("！！！任务完成！！！");
    ROS_WARN("！！！任务完成！！！");
    ROS_WARN("！！！任务完成！！！");
    return 0;
}
