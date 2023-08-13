#include <tf/transform_broadcaster.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/ChannelFloat32.h>

#include <opencv2/opencv.hpp>
#include <nav_msgs/Odometry.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tbb/concurrent_queue.h>
#include <tbb/parallel_for.h>

#include "../camera_models/CataCamera.h"
#include "parameter.h"

typedef pcl::PointXYZI PointType;

class ros_interface_t
{
private:
    parameter param;
    bool isfirstImu;
    nav_msgs::Odometry position_first;
    tbb::concurrent_bounded_queue<nav_msgs::Odometry> odom_q;
    tbb::concurrent_bounded_queue<sensor_msgs::Image> left_q, right_q;
    tbb::concurrent_bounded_queue<sensor_msgs::PointCloud2> lidar_q;

    sensor_msgs::Image left_msg, right_msg;
    sensor_msgs::PointCloud2 lidar_msg;
    nav_msgs::Odometry odom_msg;

    double sync_time;      // 时间同步限制时间

public:
    ros::Subscriber sub_img_left, sub_img_right; // 订阅图像
    ros::Subscriber sub_odom;                    // 订阅位姿
    ros::Subscriber sub_lidar;                   // 订阅雷达点云
    ros::Publisher pub_colorCloud;               // 发布上色点云
    ros_interface_t();
    // 是否开启同步，同步限制时间，文件位置
    ros_interface_t(double sync_time, std::string file);
    // 判断时间戳最小
    int maxStamp(std::vector<double> time_q);
    // 图像话题回调函数
    void LeftCallback(const sensor_msgs::ImageConstPtr &img_msg);
    void RightCallback(const sensor_msgs::ImageConstPtr &img_msg);
    // 位姿话题回调函数
    void odom_callback(const nav_msgs::OdometryConstPtr &odom_msg);
    // 雷达点云话题回调函数
    void LidarCallback(const sensor_msgs::PointCloud2ConstPtr &laser_msg);
    // 进行时间同步，保证队列首位时间戳对齐
    bool sync();
    // 定时处理函数
    void handle_timer(const ros::TimerEvent &time_e);
};