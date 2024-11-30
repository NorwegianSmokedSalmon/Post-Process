#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <boost/bind.hpp>
#include <fstream>
#include <sys/stat.h>  // 用于检查和创建文件夹

// 定义同步策略
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry> SyncPolicyImageOdom;

class GridMap {
public:
    GridMap() : nh_() {
        // 从参数服务器获取点云话题、里程计话题和目标文件夹名称
        std::string cloud_topic, odom_topic;
        nh_.param("cloud_name", cloud_topic, std::string("/cloud_registered_body"));  // 设置默认值
        nh_.param("odom_name", odom_topic, std::string("/Wxx_Odometry"));  // 设置默认值
        nh_.param("trans_name", trans_folder_name_, std::string("trans1"));  // 设置默认值

        // 订阅/同步点云和里程计数据
        point_cloud_sub_.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh_, cloud_topic, 20000));
        odom_sub_.reset(new message_filters::Subscriber<nav_msgs::Odometry>(nh_, odom_topic, 20000));

        // 设置同步策略
        sync_image_odom_.reset(new message_filters::Synchronizer<SyncPolicyImageOdom>(
            SyncPolicyImageOdom(100), *point_cloud_sub_, *odom_sub_));

        sync_image_odom_->registerCallback(boost::bind(&GridMap::depthOdomCallback, this, _1, _2));

        debug_world_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("debug_world_cloud", 100000);

        // 创建目标文件夹（如果不存在的话）
        createDirectory(std::string(ROOT_DIR) + "/" + trans_folder_name_);

        // 创建PCD文件夹（在目标文件夹内）
        createPCDDirectory(std::string(ROOT_DIR) + "/" + trans_folder_name_);

        // 打开JSON文件用于记录Odometry数据
        json_file_.open(std::string(ROOT_DIR) + trans_folder_name_ + "/pose.json", std::ios::out | std::ios::trunc);
    }

    ~GridMap() {
        if (json_file_.is_open()) {
            json_file_.close();
        }
    }

    void depthOdomCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg,
                           const nav_msgs::OdometryConstPtr& odom_msg) {
        // 保存当前点云图为PCD文件
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(*cloud_msg, cloud);  // 将ROS消息转为PCL格式

        // 使用递增的数字命名文件，例如 1.pcd, 2.pcd, ...
        std::string pcd_filename = std::string(ROOT_DIR) + "/" + trans_folder_name_ + "/pcd/" + std::to_string(frame_count_) + ".pcd";  // 存储在目标文件夹下的pcd子文件夹内
        pcl::io::savePCDFile(pcd_filename, cloud);
        ROS_INFO("Saved point cloud to %s", pcd_filename.c_str());

        // 记录Odometry数据（使用空格分隔的格式）
        json_file_ << std::fixed << std::setprecision(8)  // 设置浮点数精度
                   << odom_msg->pose.pose.position.x << " "
                   << odom_msg->pose.pose.position.y << " "
                   << odom_msg->pose.pose.position.z << " "
                   << odom_msg->pose.pose.orientation.w << " "
                   << odom_msg->pose.pose.orientation.x << " "
                   << odom_msg->pose.pose.orientation.y << " "
                   << odom_msg->pose.pose.orientation.z << std::endl;

        Eigen::Vector3f p{odom_msg->pose.pose.position.x, odom_msg->pose.pose.position.y, odom_msg->pose.pose.position.z};
        Eigen::Quaternionf rotation(odom_msg->pose.pose.orientation.w, odom_msg->pose.pose.orientation.x, odom_msg->pose.pose.orientation.y, odom_msg->pose.pose.orientation.z);

        for (int i = 0; i < cloud.points.size(); i++) {
            pcl::PointXYZ point = cloud.points[i];
            Eigen::Vector3f e_p{point.x, point.y, point.z};
            e_p = rotation * e_p + p;
            point.x = e_p[0];
            point.y = e_p[1];
            point.z = e_p[2];
            cloud.points[i] = point;
        }

        sensor_msgs::PointCloud2 laserCloudmsg;
        pcl::toROSMsg(cloud, laserCloudmsg);
        laserCloudmsg.header.stamp = ros::Time().now();
        laserCloudmsg.header.frame_id = "world";
        debug_world_cloud_pub_.publish(laserCloudmsg);

        frame_count_++;
    }

private:
    ros::NodeHandle nh_;
    boost::shared_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> point_cloud_sub_;
    boost::shared_ptr<message_filters::Subscriber<nav_msgs::Odometry>> odom_sub_;
    boost::shared_ptr<message_filters::Synchronizer<SyncPolicyImageOdom>> sync_image_odom_;
    ros::Publisher debug_world_cloud_pub_;

    std::ofstream json_file_;  // 用于记录Odometry数据
    int frame_count_ = 0;
    std::string trans_folder_name_;  // 存储文件夹名称

    // 创建目标文件夹（如果不存在的话）
    void createDirectory(const std::string& folder_name) {
        struct stat info;
        if (stat(folder_name.c_str(), &info) != 0) {
            // 如果文件夹不存在，则创建
            ROS_INFO("Directory %s does not exist. Creating it now...", folder_name.c_str());
            if (mkdir(folder_name.c_str(), 0777) == -1) {
                ROS_ERROR("Failed to create directory %s", folder_name.c_str());
            } else {
                ROS_INFO("Directory %s created.", folder_name.c_str());
            }
        }
    }

    // 创建PCD文件夹（如果不存在的话）
    void createPCDDirectory(const std::string& folder_name) {
        struct stat info;
        std::string pcd_folder = folder_name + "/pcd";
        if (stat(pcd_folder.c_str(), &info) != 0) {
            // 如果文件夹不存在，则创建
            ROS_INFO("PCD directory does not exist. Creating it now...");
            if (mkdir(pcd_folder.c_str(), 0777) == -1) {
                ROS_ERROR("Failed to create directory %s", pcd_folder.c_str());
            } else {
                ROS_INFO("PCD directory created.");
            }
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "grid_map_sync");
    setlocale(LC_ALL, "en_US.UTF-8");
    GridMap grid_map;
    ros::spin();
    return 0;
}
