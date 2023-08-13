#include <iostream>
#include "../include/colorMap_sync.h"
int main(int argc, char **argv)
{
    ros::init(argc, argv, "vins");
    ros::NodeHandle nh;
    ros_interface_t RosInterface(0.3, "/home/oem/tool_ws/src/color_cloud/");

    ROS_INFO("\033[1;32m----> Visual Feature Tracker Started.\033[0m");
    RosInterface.sub_img_left = nh.subscribe<sensor_msgs::Image>("/cam/left/bgr_gamma", 200,
                                                                 boost::bind(&ros_interface_t::LeftCallback, &RosInterface, _1));
    RosInterface.sub_img_right = nh.subscribe<sensor_msgs::Image>("/cam/right/bgr_gamma", 200,
                                                                  boost::bind(&ros_interface_t::RightCallback, &RosInterface, _1));
    RosInterface.sub_odom = nh.subscribe<nav_msgs::Odometry>("/Odometry", 100,
                                                             boost::bind(&ros_interface_t::odom_callback, &RosInterface, _1));
    RosInterface.sub_lidar = nh.subscribe<sensor_msgs::PointCloud2>("/cloud_registered", 100,
                                                                    boost::bind(&ros_interface_t::LidarCallback, &RosInterface, _1));
    RosInterface.pub_colorCloud = nh.advertise<sensor_msgs::PointCloud2>("/colored_point", 1000);
    // 0.1s运行一次callback2
    ros::Timer timer1 = nh.createTimer(ros::Duration(0.1), boost::bind(&ros_interface_t::handle_timer,
                                                                       &RosInterface, _1));
    // ros::Timer timer1 = nh.createTimer(ros::Duration(1.0), callback1); // 0.1s运行一次callback1
    ros::spin();
    return 0;
}