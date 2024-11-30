#include <ros/ros.h>
#include "bag2pcd/view_pcd.h"

// 获取pcd存入容器，拿出转换并存入vec后，直接发布

int main(int argc, char** argv) {
    ros::init(argc, argv, "point_cloud_merger");
    ros::NodeHandle nh;
    ros::Rate loop_rate(100);  // 10 Hz
    setlocale(LC_ALL, "en_US.UTF-8");

    // 点云发布器
    cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("merged_point_cloud", 1);
    new_iteration_sub_ = nh.subscribe("/new_iteration", 1000, newIterationCallback);
    vis_iteration_sub_ = nh.subscribe("/json_index", 1000, visIterationCallback);

    // 获取launch文件中传入的参数
    std::string trans_name;
    if (!nh.getParam("trans_name", trans_name)) {
        ROS_ERROR("Failed to get param 'trans_name'");
        return -1;
    }

    // 拼接文件夹路径
    std::string current_dir = std::string(ROOT_DIR) + trans_name + "/pcd";  // pcd文件夹路径
    json_file = std::string(ROOT_DIR) + trans_name + "/pose.json"; // pose.json文件路径

    int data_size;

    auto start_time = std::chrono::steady_clock::now();

    //读取json文件 存储到vector中
    if (!readOdomFromTxt(json_file, data_size)) {
        ROS_ERROR("Failed to read Odom data from %s", json_file.c_str());
        return -1;
    }
    Data_volume_ = data_size;

    auto json_end_time = std::chrono::steady_clock::now();

    ROS_INFO("START LOADING PCD");
    for (int i = 0; i < Data_volume_; i++)
    {
        //定义pcd_file名字,并获取对应点云
        pcd_file_ = current_dir + "/" + std::to_string(i+1) + ".pcd";
        pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>());
        readPointCloud(pcd_file_, cloud);
        // std::cout << "current_file_index: " << i << ", " << pcd_file_ << std::endl;
        // cloud_vec.push_back(pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>(*cloud)));
        cloud_vec.push_back(cloud);
    }
    ROS_INFO("LOADING PCD OK");

    auto pcd_end_time = std::chrono::steady_clock::now();

    while(ros::ok())
    {
            Cycle_limit = vis_iteration_;
            // 清空这两个向量
            odom_pos_vec_.clear();
            odom_rot_vec_.clear();
            if (vis_iteration_ == -1)
            {
                json_file = std::string(ROOT_DIR) + trans_name +"/pose.json"; // 假设文件名是 pose.json
            }else{
                // json_file = "./" + trans_name + "/pose_" + std::to_string(vis_iteration_) + ".json"; // 假设文件名是 pose.json
                json_file = std::string(ROOT_DIR) + trans_name + "/" + json_name + ".json"; // 假设文件名是 pose.json
            }

            if (!readOdomFromTxt(json_file, data_size)) 
            {
                ROS_ERROR("Failed to read Odom data from %s", json_file.c_str());
                return -1;
            }
        
        for (int i = 0; i < cloud_vec.size(); i++)
        {
            // i++;
            Eigen::Vector3f pos = odom_pos_vec_[i];
            Eigen::Quaternionf rot = odom_rot_vec_[i];

            transformed_cloud->points.resize(cloud_vec[i]->points.size());
            for(int j=0; j < cloud_vec[i]->points.size(); j++)
            {
                PointType point = cloud_vec[i]->points[j];
                Eigen::Vector3f e_p{point.x, point.y, point.z};
                e_p = rot * e_p + pos;
                point.x = e_p[0];
                point.y = e_p[1];
                point.z = e_p[2];
                transformed_cloud->points[j] = point;
            }

            bool too_large = mergePointClouds(transformed_cloud, merged_cloud, 100000);
            if (too_large) {
                ROS_WARN("too_large");
                ROS_WARN("%d", i);
                // 如果点云过大，则将merged_cloud放入cloud_vec中，并清空merged_cloud，继续合并
                // cloud_vec.push_back(merged_cloud);
                // cloud_trans_vec.push_back(pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>(*merged_cloud)));
                publishPointCloud(cloud_pub_, merged_cloud);
                merged_cloud->clear();
                mergePointClouds(transformed_cloud, merged_cloud, 100000);
            }
            // if (cloud_trans_vec.size() !=0 )
            // {
            //     publishPointCloud(cloud_pub_, cloud_trans_vec.back());
            // }
        }

        auto trans_end_time = std::chrono::steady_clock::now();

        std::chrono::duration<double> json_duration = json_end_time - start_time;
        std::chrono::duration<double> pcd_duration = pcd_end_time - json_end_time;
        std::chrono::duration<double> trans_duration = trans_end_time - pcd_end_time;
        ROS_INFO("json_duration: %f 秒", json_duration.count());
        ROS_INFO("pcd_duration: %f 秒", pcd_duration.count());
        ROS_INFO("trans_duration: %f 秒", trans_duration.count());

        while (ros::ok)
        {
            if(vis_iteration_ != Cycle_limit)
                break;
            // ROS_INFO("NO break");
            
            ros::spinOnce();
            loop_rate.sleep();
        }
    }
    
    return 0;

}