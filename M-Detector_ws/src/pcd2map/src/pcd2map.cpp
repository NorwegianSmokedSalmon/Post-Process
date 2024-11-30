#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <Eigen/Dense>
#include <boost/format.hpp>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <ikd-Tree/ikd_Tree.h>
#include <pcl_conversions/pcl_conversions.h>
#define INFO_MSG_RED(str) do {std::cout << "\033[31m" << str << "\033[0m" << std::endl; } while(false)
#define INFO_MSG_RED_HIGHLIGHT(str) do {std::cout << "\033[1;31m" << str << "\033[0m" << std::endl; } while(false)
#define INFO_MSG_GREEN(str) do {std::cout << "\033[32m" << str << "\033[0m" << std::endl; } while(false)
#define INFO_MSG_GREEN_HIGHLIGHT(str) do {std::cout << "\033[1;32m" << str << "\033[0m" << std::endl; } while(false)

using namespace std;
using PointVector = std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ>>;

ros::Publisher merged_cloud_pub_, ikd_tree_pub_;
KD_TREE<pcl::PointXYZ> ikdtree_;
// param
double Downsample_Leaf_Size_for_Vis_;
double Downsample_Leaf_Size_for_Kdtree_;

struct Pose {
    Eigen::Vector3d position;
    Eigen::Quaterniond orientation;
};

bool readPoses(const std::string& filename, std::vector<Pose>& poses) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Fail to open: " << filename << std::endl;
        return false;
    }

    poses.clear();
    std::string line;
    double x, y, z, qw, qx, qy, qz;

    while (std::getline(file, line)) {
        std::istringstream iss(line);
        if (!(iss >> x >> y >> z >> qw >> qx >> qy >> qz)) {
            std::cerr << "Fail to parse data！" << std::endl;
            return false;
        }
        
        Pose pose;
        pose.position = Eigen::Vector3d(x, y, z);
        pose.orientation = Eigen::Quaterniond(qw, qx, qy, qz).normalized();
        poses.push_back(pose);
    }

    return true;
}

bool readPCDFiles(const std::string& folder_path, 
                 int pcd_num, 
                 std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& clouds) {
    clouds.clear();

    for (int i = 1; i <= pcd_num; ++i) {
        std::string pcd_file = folder_path + "/" + 
                             (boost::format("%d.pcd") % i).str();

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

        if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file, *cloud) == -1) {
            std::cerr << "Fail to load file: " << pcd_file << std::endl;
            return false;
        }

        clouds.push_back(cloud);
    }

    return true;
}

void downsampleCloud( const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, double leaf_size) {
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    voxel_filter.setInputCloud(cloud);
    voxel_filter.setLeafSize(leaf_size, leaf_size, leaf_size);
    voxel_filter.filter(*cloud);
}

void publishPointCloud(ros::Publisher &pub, const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) {
    sensor_msgs::PointCloud2 cloud_msg;
    cloud_msg.data.clear();  // 清空点云数据
    pcl::toROSMsg(*cloud, cloud_msg);
    cloud_msg.header.stamp = ros::Time::now();
    cloud_msg.header.frame_id = "world";  // 或者其他适合的坐标系名称
    pub.publish(cloud_msg);
    ros::Duration(0.01).sleep();
}

void mergeClouds(
    const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& clouds,
    const std::vector<Pose>& poses,
    KD_TREE<pcl::PointXYZ> &ikdtree,
    pcl::PointCloud<pcl::PointXYZ>::Ptr &merged_cloud) {
    
    for (size_t i = 0; i < clouds.size(); ++i) {
        Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
        transform.block<3,3>(0,0) = poses[i].orientation.cast<float>().toRotationMatrix();
        transform.block<3,1>(0,3) = poses[i].position.cast<float>();

        pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::transformPointCloud(*clouds[i], *transformed_cloud, transform);

        if(ikdtree.Root_Node == nullptr) {
            ikdtree.set_downsample_param(Downsample_Leaf_Size_for_Kdtree_);
            ikdtree.Build(transformed_cloud->points);
        }
        else {
          PointVector PointToAdd;
          for( int j = 0; j < transformed_cloud->points.size(); j++ )
            PointToAdd.push_back(transformed_cloud->points[j]);
          ikdtree.Add_Points(PointToAdd, true);
        }

        *merged_cloud += *transformed_cloud;
        downsampleCloud(merged_cloud, Downsample_Leaf_Size_for_Vis_);
        publishPointCloud(merged_cloud_pub_, merged_cloud);
    }
}

void saveIkdTree(
    KD_TREE<pcl::PointXYZ> &ikdtree,
    pcl::PointCloud<pcl::PointXYZ>::Ptr &ikdtree_cloud,
    std::string file_name_map_kdtree) {
    
    // save the points from the kd-tree
    PointVector ().swap(ikdtree.PCL_Storage);
    ikdtree.flatten(ikdtree.Root_Node, ikdtree.PCL_Storage, NOT_RECORD);
    ikdtree_cloud->clear();
    ikdtree_cloud->points = ikdtree.PCL_Storage;
    std::string all_points_dir_map_kdtree(std::string(std::string(Pcd2map_ROOT_DIR) + "PCD/") + file_name_map_kdtree);
    pcl::PCDWriter pcd_writer;
    cout << "\033[1;32mthe new map kd-tree saved to /PCD/" << file_name_map_kdtree << "\033[0m" << endl;
    pcd_writer.writeBinary(all_points_dir_map_kdtree, *ikdtree_cloud);
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "pcd2map");
    ros::NodeHandle nh;

    merged_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("vis_merged_cloud", 1);
    ikd_tree_pub_ = nh.advertise<sensor_msgs::PointCloud2>("ikd_tree", 1);
    
    std::string pcd_folder, pose_file, file_name_map_kdtree;

    nh.param<std::string>("pcd_folder",pcd_folder,"");
    nh.param<std::string>("pose_file", pose_file,"");
    nh.param<std::string>("output_ikdtree_file_name", file_name_map_kdtree,"wxx.pcd");
    nh.param<double>("downsample_leaf_size_for_vis", Downsample_Leaf_Size_for_Vis_, 0.05);
    nh.param<double>("downsample_leaf_size_for_kdtree", Downsample_Leaf_Size_for_Kdtree_, 0.05);

    std::vector<Pose> poses;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds;

    // load pose
    INFO_MSG_RED_HIGHLIGHT("Load pose file ...");
    if(!readPoses(pose_file, poses)) {
      ROS_ERROR("readPoses Fail!");
      return -1;
    }
    INFO_MSG_GREEN_HIGHLIGHT("Load pose file successed!");
    INFO_MSG_GREEN_HIGHLIGHT("");

    // load pcd
    INFO_MSG_RED_HIGHLIGHT("Load pcd files ...");
    if(!readPCDFiles(pcd_folder, poses.size(), clouds)) {
        ROS_ERROR("readPCDFiles Fail!");
        return -1;
    }
    INFO_MSG_GREEN_HIGHLIGHT("Load pcd files successed!");
    INFO_MSG_GREEN_HIGHLIGHT("");

    // merge pcd
    INFO_MSG_RED_HIGHLIGHT("Merge pcd files ...");
    pcl::PointCloud<pcl::PointXYZ>::Ptr vis_merged_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    mergeClouds(clouds, poses, ikdtree_, vis_merged_cloud);
    INFO_MSG_GREEN_HIGHLIGHT("Merge pcd files successed!");
    INFO_MSG_GREEN_HIGHLIGHT("");

    // save ikd-tree pcd
    INFO_MSG_RED_HIGHLIGHT("Save ikd-tree pcd ...");
    pcl::PointCloud<pcl::PointXYZ>::Ptr vis_ikd_tree(new pcl::PointCloud<pcl::PointXYZ>);
    saveIkdTree(ikdtree_, vis_ikd_tree, file_name_map_kdtree);
    INFO_MSG_GREEN_HIGHLIGHT("Save ikd-tree pcd successed!");
    INFO_MSG_GREEN_HIGHLIGHT("");

    sensor_msgs::PointCloud2 vis_merged_cloud_msg;
    pcl::toROSMsg(*vis_merged_cloud, vis_merged_cloud_msg);
    vis_merged_cloud_msg.header.frame_id = "world";

    sensor_msgs::PointCloud2 ikdtree_cloud_msg;
    pcl::toROSMsg(*vis_ikd_tree, ikdtree_cloud_msg);
    ikdtree_cloud_msg.header.frame_id = "world";

    ros::Rate rate(10);  // 10Hz
    while (ros::ok()) {
        vis_merged_cloud_msg.header.stamp = ros::Time::now();
        merged_cloud_pub_.publish(vis_merged_cloud_msg);
        ikdtree_cloud_msg.header.stamp = ros::Time::now();
        ikd_tree_pub_.publish(ikdtree_cloud_msg);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}