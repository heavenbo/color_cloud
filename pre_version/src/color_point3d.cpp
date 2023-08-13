#include <iostream>
#include "../include/colorMap_copy.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vins");
    ros::NodeHandle n;

    Init("/home/oem/Vscode/color_cloud/camera.yaml");
    initLog("/home/oem/Vscode/color_cloud/logs/");
    LOG(INFO) << "Initializing......";
    ROS_INFO("begin");
    pub_colorCloud = n.advertise<sensor_msgs::PointCloud2>("/colored_point", 1000);
    ros::Subscriber sub_imu = n.subscribe("/odometry/imu", 200, odoLidar);
    ros::Subscriber sub_lidar = n.subscribe("/liorf/mapping/cloud_registered_raw", 10, lidar_callback);
    ros::Subscriber sub_camera = n.subscribe("/cam/left/bgr", 10, img_callback);
    ros::spin();
    return 0;
}