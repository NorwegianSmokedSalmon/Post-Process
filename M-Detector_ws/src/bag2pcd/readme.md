bag2pcd使用：
    roslaunch bag2pcd  bag2pcd.launch
    //trans_name 是 包含 pcd文件夹 和 json 的文件夹名字，生成的pcd文件夹和json会往里面放
    //cloud_name ：要订阅的点云话题(body)
    //odom_name ： 要订阅的odom

topic2pcd
    roslaunch bag2pcd  topic2pcd.launch
    //trans_name 是 包含 pcd文件夹 和 json 的文件夹名字，生成的pcd文件夹和json会往里面放
    //cloud_name ：要订阅的点云话题(body)
    //odom_name ： 要订阅的odom
    
view_pcd
    roslaunch bag2pcd  view_pcd.launch
    //trans_name 是 包含 pcd文件夹 和 json 的文件夹名字，会从里面读取pcd文件夹和json，用rivz显示

trans2error
    roslaunch bag2pcd  trans2error.launch
    //trans_config_file ：配置文件的路径 内容：偏置的点的编号, X, Y, Z, YAW, PITCH, ROLL
    //trans_file_name ：要转换的json文件的路径 转换出来的文件名称：pose_updated.json