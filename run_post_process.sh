#!/bin/zsh

# 启动 mapping
gnome-terminal -- zsh -c "
cd M-Detector_ws/
source devel/setup.zsh
roslaunch fast_lio mapping_mid360.launch use_rviz:=false
exec zsh
"
sleep 5

# 启动 detector
gnome-terminal -- zsh -c "
cd M-Detector_ws/
source devel/setup.zsh
roslaunch m_detector detector_mid360.launch
exec zsh
"
sleep 5

# 录制 rosbag 数据
gnome-terminal -- zsh -c "
cd data/
rosbag record /Odometry /m_detector/std_points -O static.bag
exec zsh
"
sleep 5

# 播放 rosbag 文件
gnome-terminal -- zsh -c "
cd data/
rosbag play 4.bag --topics /livox/lidar /livox/imu
exec zsh
"

# 等待 rosbag 播放结束并关闭前四个终端
sleep 240
sleep 5  # 等待 rosbag 完全播放并停留 5 秒

# 关闭前四个终端
# pkill -f "gnome-terminal.*zsh"

# 转换 bag 到 PCD
gnome-terminal -- zsh -c "
cd M-Detector_ws/
source devel/setup.zsh
roslaunch bag2pcd bag2pcd.launch 
exec zsh
"
sleep 100



# 转换到body系
gnome-terminal -- zsh -c "
cd M-Detector_ws/
source devel/setup.zsh
roslaunch m_detector transToBodyFrame.launch
exec zsh
"
sleep 200

cp data/pose.json data/body_pcd
sleep 5

# 激活新环境
# 启动 loopClosure_offline

gnome-terminal -- zsh -c "
cd cd M-Detector_ws/
source devel/setup.zsh
roslaunch hba loopClosure_offline.launch
exec zsh
"
sleep 70


# 启动 hba
gnome-terminal -- zsh -c "
cd cd M-Detector_ws/
source devel/setup.zsh
roslaunch hba hba.launch
exec zsh
"
sleep 300

# 确保所有进程结束
wait

