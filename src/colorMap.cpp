#include <iostream>
#include "../include/colorMap.hpp"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
int main(int argc, char **argv)
{
    ros::init(argc, argv, "vins");
    ros::NodeHandle nh;

    param = parameter("/home/oem/tool_ws/src/color_cloud/colorCloud.yaml");
    param.initLog("/home/oem/tool_ws/src/color_cloud/logs/");
    LOG(INFO) << "Initializing......";
    ROS_INFO("begin");
    pub_colorCloud = nh.advertise<sensor_msgs::PointCloud2>("/colored_point", 1000);
    message_filters::Subscriber<sensor_msgs::Image> right_sub(nh, "/cam/right/bgr_gamma", 100);
    message_filters::Subscriber<sensor_msgs::Image> left_sub(nh, "/cam/left/bgr_gamma", 100);
    message_filters::Subscriber<sensor_msgs::PointCloud2> pcl_sub(nh, "/cloud_registered", 50);
    message_filters::Subscriber<nav_msgs::Odometry> odom_sub(nh, "/Odometry", 50);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
                                                            sensor_msgs::Image,
                                                            sensor_msgs::PointCloud2,
                                                            nav_msgs::Odometry>
        MySyncPolicy;
    // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), left_sub, right_sub,
                                                     pcl_sub, odom_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2, _3, _4));

    ros::spin();

    return 0;
}