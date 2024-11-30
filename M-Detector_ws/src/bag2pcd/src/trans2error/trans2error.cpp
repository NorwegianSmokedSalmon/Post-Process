#include <ros/ros.h>
#include <fstream>
#include <iostream>
#include <vector>
#include <string>
#include <sstream>
#include <iomanip>  // 用于设置输出精度
#include <Eigen/Core>
#include <Eigen/Geometry>  // 包含四元数支持

// 定义一个结构体用于保存点的平移和四元数
struct Pose {
    Eigen::Vector3d translation;
    Eigen::Quaterniond rotation;

    Pose(double tx, double ty, double tz, double qw, double qx, double qy, double qz) {
        translation = Eigen::Vector3d(tx, ty, tz);
        rotation = Eigen::Quaterniond(qw, qx, qy, qz);
    }

    // 通过平移量和欧拉角来修改当前位姿
    void applyTransformation(double tx, double ty, double tz, double yaw, double pitch, double roll) {
        // 创建平移向量
        Eigen::Vector3d delta_translation(tx, ty, tz);

        // 创建旋转四元数，从欧拉角（yaw, pitch, roll）计算
        Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
        
        Eigen::Quaterniond delta_rotation = yawAngle * pitchAngle * rollAngle;

        // 更新平移和旋转
        translation += rotation * delta_translation; // 先旋转再平移
        rotation = delta_rotation * rotation; // 先旋转再更新旋转
    }
};

struct train {
    int num;
    double tx, ty, tz;
    double yaw, pitch, roll;
};

// 计算两点之间的相对变换
Pose computeRelativeTransform(const Pose& prev_pose, const Pose& current_pose) {
    // 计算旋转部分
    Eigen::Quaterniond q_rel = prev_pose.rotation.inverse() * current_pose.rotation;

    // 计算平移部分
    Eigen::Vector3d t_rel = prev_pose.rotation.inverse() * (current_pose.translation - prev_pose.translation);

    // 返回新的Pose（平移和旋转）
    return Pose(t_rel.x(), t_rel.y(), t_rel.z(), q_rel.w(), q_rel.x(), q_rel.y(), q_rel.z());
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "relative_transform");
    ros::NodeHandle nh;
    setlocale(LC_ALL, "en_US.UTF-8");

    // 获取launch文件中传入的参数
    std::string trans_file_name;
    std::string trans_file_out_name;
    std::string trans_name;
    if (!nh.getParam("trans_file_name", trans_file_name)) {
        ROS_ERROR("Failed to get param 'trans_file_name'");
        return -1;
    }
    if (!nh.getParam("trans_file_out_name", trans_file_out_name)) {
        ROS_ERROR("Failed to get param 'trans_file_out_name'");
        return -1;
    }
    if (!nh.getParam("trans_name", trans_name)) {
        ROS_ERROR("Failed to get param 'trans_name'");
        return -1;
    }
    std::ifstream file(std::string(ROOT_DIR) + "/" + trans_name + "/" + trans_file_name);

    if (!file.is_open()) {
        ROS_ERROR("Failed to open file: %s", trans_file_name);
        return -1;
    }

    std::vector<Pose> poses;  // 存储所有的Pose

    // 读取文件中的数据，并将其转换为Pose对象
    std::string line;
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        double tx, ty, tz, qw, qx, qy, qz;
        ss >> tx >> ty >> tz >> qw >> qx >> qy >> qz;

        // 创建 Pose 对象并存储
        poses.push_back(Pose(tx, ty, tz, qw, qx, qy, qz));
    }
    file.close();

    // 获取launch文件中传入的参数
    std::string trans_config_file;
    if (!nh.getParam("trans_config_file", trans_config_file)) {
        ROS_ERROR("Failed to get param 'trans_config_file'");
        return -1;
    }

    std::ifstream file2(std::string(ROOT_DIR) + "/" + trans_config_file);
    if (!file2.is_open()) {
        std::cerr << "无法打开文件!" << std::endl;
        return -1;
    }

    std::string line2;
    std::vector<train> trains;  // 用于存储每个点的数据
    
    // 逐行读取文件
    while (std::getline(file2, line2)) {
        std::stringstream ss(line2);
        train train;
        
        // 解析每行的数据 (以逗号分隔)
        char comma;  // 用于读取逗号
        
        ss >> train.num >> comma
           >> train.tx >> comma
           >> train.ty >> comma
           >> train.tz >> comma
           >> train.yaw >> comma
           >> train.pitch >> comma
           >> train.roll;
        
        // 将点加入容器
        trains.push_back(train);
    }
    file2.close();








    // 用于存储每个点相对于上一个点的位姿
    std::vector<Pose> relative_poses;

    // 从第二个点开始，计算每个点相对于上一个点的变换
    for (size_t i = 1; i < poses.size(); ++i) {
        ROS_INFO("Processing pose %zu", i);

        // 计算当前点相对于上一个点的变换
        Pose relative_pose = computeRelativeTransform(poses[i - 1], poses[i]);

        // 检查是否需要应用变化：如果i等于某个train.num，则应用变换
        for (const train& tr : trains) {
            if (i == tr.num) {
                ROS_INFO("Applying transformation for train %d", tr.num);
                // 应用位移变化
                relative_pose.applyTransformation(tr.tx, tr.ty, tr.tz, tr.yaw, tr.pitch, tr.roll);
                break;  // 找到对应的train并应用变换后，跳出循环
            }
        }

        // 将当前点相对于上一个点的位姿存入容器
        relative_poses.push_back(relative_pose);
    }

    // 通过relative_poses和第一个点来计算相对原点的位置
    std::vector<Pose> updated_poses;
    updated_poses.push_back(poses[0]);  // 将第一个点加入到新的结果中

    // 计算后续点的相对原点变换
    for (size_t i = 0; i < poses.size()-1; ++i) {
        // 累积位姿变化
        Pose cumulative_pose = updated_poses[i];

        // cumulative_pose.translation += relative_poses[i].rotation * relative_poses[i].translation;
        // cumulative_pose.rotation = relative_poses[i].rotation * cumulative_pose.rotation;

        cumulative_pose.translation += cumulative_pose.rotation * relative_poses[i].translation;
        cumulative_pose.rotation =  cumulative_pose.rotation * relative_poses[i].rotation;

        // 将计算的相对原点的位姿添加到容器
        updated_poses.push_back(cumulative_pose);
    }

    // 输出所有计算得到的相对原点变换到文件
    std::ofstream output_file(std::string(ROOT_DIR) + "/" + trans_name + "/" + trans_file_out_name);  // 输出文件
    if (!output_file.is_open()) {
        ROS_ERROR("Failed to open output file");
        return -1;
    }
// 【修改】设置输出的精度为9位小数
    output_file << std::fixed << std::setprecision(9);
    for (const Pose& updated_pose : updated_poses) {
        output_file << updated_pose.translation.x() << " "
                    << updated_pose.translation.y() << " "
                    << updated_pose.translation.z() << " "
                    << updated_pose.rotation.w() << " "
                    << updated_pose.rotation.x() << " "
                    << updated_pose.rotation.y() << " "
                    << updated_pose.rotation.z() << std::endl;
    }

    output_file.close();  // 关闭文件

    ros::spin();
    return 0;
}
