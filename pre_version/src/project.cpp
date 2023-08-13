#include <iostream>
#include "../include/lidar_camera.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vins");
    ros::NodeHandle n;

    Init("/home/oem/Vscode/color_cloud/camera.yaml");

    ROS_INFO("begin");
    ros::Subscriber sub_imu = n.subscribe("/odometry/imu_incremental", 200, odoLidar);
    ros::Subscriber sub_lidar = n.subscribe("/liorf/mapping/cloud_registered_raw", 10, lidar_callback);
    ros::Subscriber sub_camera = n.subscribe("/cam/left/gray", 10, img_callback);
    ros::spin();
    return 0;
}