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
#include <mutex>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <yaml-cpp/yaml.h>
#include <Eigen/Geometry>
#include <tf/transform_broadcaster.h>
typedef pcl::PointXYZI PointType;

// sensor_msgs::Image depth_img;

std::mutex imu_mutex; // 锁imu坐标系
bool isfirstImu;
nav_msgs::Odometry imu_now;
geometry_msgs::Point position_first;
std::mutex lidar_mutex; // 锁雷达点云

std::deque<pcl::PointCloud<PointType>> cloudQueue;
pcl::PointCloud<PointType>::Ptr depthCloud(new pcl::PointCloud<PointType>());
std::deque<double> timeQueue;

double left_intrinsics[4], right_intrinsics[4];
Eigen::Affine3f tran_imu2camera;
Eigen::Matrix3d R_camera_imu; // 旋转矩阵
Eigen::Matrix4d imu2camera, imu2camera_inv;
Eigen::Vector3d T_imu_world, T_camera_imu, t_imu_cam;
// Eigen::Transform a;
int width, height;
float time_;

void Init(std::string file_path)
{

    YAML::Node yaml_node = YAML::LoadFile(file_path);
    width = yaml_node["Width"].as<double>();
    height = yaml_node["Height"].as<double>();
    for (int i = 0; i < 4; i++)
    {
        left_intrinsics[i] = yaml_node["Left_cam"]["intrinsics"][i].as<double>();
        right_intrinsics[i] = yaml_node["Right_cam"]["intrinsics"][i].as<double>();
    }
    for (int i = 0; i < 16; i++)
    {
        imu2camera(i / 4, i % 4) = yaml_node["Left_cam"]["T_cam_imu"][i / 4][i % 4].as<double>();
    }
    time_ = yaml_node["Time"].as<float>();

    R_camera_imu << imu2camera(0, 0), imu2camera(0, 1), imu2camera(0, 2),
        imu2camera(1, 0), imu2camera(1, 1), imu2camera(1, 2),
        imu2camera(2, 0), imu2camera(2, 1), imu2camera(2, 2); // world坐标到imu坐标的转换R
    t_imu_cam = -R_camera_imu.transpose() * Eigen::Vector3d(imu2camera(0, 3), imu2camera(1, 3), imu2camera(2, 3));
    // 得到imu-camera坐标的转换
    T_camera_imu = R_camera_imu.eulerAngles(2, 1, 0);
    tran_imu2camera = pcl::getTransformation(imu2camera(0, 3), imu2camera(1, 3), imu2camera(2, 3),
                                             T_camera_imu[2], T_camera_imu[1], T_camera_imu[0]);
}
// 实时更新imu位姿
void odometry_imu(const nav_msgs::OdometryConstPtr &odometry)
{
    if (!isfirstImu)
    {
        position_first = odometry->pose.pose.position;
        isfirstImu = true;
    }
    imu_mutex.lock();
    imu_now = *odometry;
    imu_now.pose.pose.position.x = imu_now.pose.pose.position.x - position_first.x;
    imu_now.pose.pose.position.y = imu_now.pose.pose.position.y - position_first.y;
    imu_now.pose.pose.position.z = imu_now.pose.pose.position.z - position_first.z;
    imu_mutex.unlock();
}
void lidar_callback(const sensor_msgs::PointCloud2ConstPtr &laser_msg)
{
    // 1. convert laser cloud message to pcl
    pcl::PointCloud<PointType>::Ptr laser_cloud_in(new pcl::PointCloud<PointType>());
    pcl::fromROSMsg(*laser_msg, *laser_cloud_in);

    // 2. downsample new cloud (save memory)
    pcl::PointCloud<PointType>::Ptr laser_cloud_in_ds(new pcl::PointCloud<PointType>());
    static pcl::VoxelGrid<PointType> downSizeFilter;
    downSizeFilter.setLeafSize(0.3, 0.3, 0.3);
    downSizeFilter.setInputCloud(laser_cloud_in);
    downSizeFilter.filter(*laser_cloud_in_ds);
    *laser_cloud_in = *laser_cloud_in_ds;

    // save new cloud
    double timeScanCur = laser_msg->header.stamp.toSec();
    cloudQueue.push_back(*laser_cloud_in);
    timeQueue.push_back(timeScanCur);

    // pop old cloud
    while (!timeQueue.empty())
    {
        if (timeScanCur - timeQueue.front() > time_)
        {
            cloudQueue.pop_front();
            timeQueue.pop_front();
        }
        else
        {
            break;
        }
    }
    // 8. fuse cloud
    std::lock_guard<std::mutex> lock(lidar_mutex);
    depthCloud->clear();
    for (int i = 0; i < (int)cloudQueue.size(); ++i)
    {
        *depthCloud += cloudQueue[i];
    }
}
void img_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    static tf::TransformBroadcaster broadcaster;
    // 得到世界坐标系-imu的转换
    cv::Mat cam = cv::Mat(img_msg->height, img_msg->width, CV_8UC1, const_cast<uchar *>(&img_msg->data[0]), img_msg->step);
    imu_mutex.lock();
    nav_msgs::Odometry imu_now_ = imu_now;
    imu_mutex.unlock();
    Eigen::Quaterniond quaternion4(imu_now_.pose.pose.orientation.w,
                                   imu_now_.pose.pose.orientation.x,
                                   imu_now_.pose.pose.orientation.y,
                                   imu_now_.pose.pose.orientation.z); // imu坐标系在world坐标系的旋转表示
    Eigen::Matrix3d a = quaternion4.matrix().transpose();             // world坐标到imu坐标的转换R
    T_imu_world = -a * Eigen::Vector3d(imu_now_.pose.pose.position.x,
                                       imu_now_.pose.pose.position.y,
                                       imu_now_.pose.pose.position.z); // world坐标到imu坐标的转换T
    Eigen::Vector3d world2imu_r = a.eulerAngles(2, 1, 0);
    Eigen::Affine3f tran_world2imu = pcl::getTransformation(T_imu_world[0], T_imu_world[1], T_imu_world[2],
                                                            world2imu_r[2], world2imu_r[1], world2imu_r[0]);

    // 1.lidar本体坐标系下的点云，转到imu坐标系下表示
    // 发布world-imu坐标
    broadcaster.sendTransform(
        tf::StampedTransform(
            tf::Transform(tf::Quaternion(imu_now_.pose.pose.orientation.x,
                                         imu_now_.pose.pose.orientation.y,
                                         imu_now_.pose.pose.orientation.z,
                                         imu_now_.pose.pose.orientation.w),
                          tf::Vector3(imu_now_.pose.pose.position.x,
                                      imu_now_.pose.pose.position.y,
                                      imu_now_.pose.pose.position.z)),
            img_msg->header.stamp, "world", "imu"));
    lidar_mutex.lock();
    pcl::PointCloud<PointType>::Ptr laser_cloud_offset(new pcl::PointCloud<PointType>());
    pcl::transformPointCloud(*depthCloud, *laser_cloud_offset, tran_world2imu);
    lidar_mutex.unlock();

    // 3.  imu坐标系下的点云，转到vins的world系下表示
    // 发布camera-imu转换
    Eigen::Quaterniond quaternion2(R_camera_imu.transpose());
    broadcaster.sendTransform(
        tf::StampedTransform(
            tf::Transform(tf::Quaternion(quaternion2.x(), quaternion2.y(),
                                         quaternion2.z(), quaternion2.w()),
                          tf::Vector3(t_imu_cam[0], t_imu_cam[1], t_imu_cam[2])),
            ros::Time::now(), "imu", "camera"));
    pcl::PointCloud<PointType>::Ptr laser_cloud_global(new pcl::PointCloud<PointType>());
    pcl::transformPointCloud(*laser_cloud_offset, *laser_cloud_global, tran_imu2camera);
    // 4.三维点映射为像素，并绘制depth图
    int point = 0;
    cv::Mat depth_img = cv::Mat::zeros(height, width, CV_8UC1);
    for (int i = 0; i < (int)laser_cloud_global->size(); ++i)
    {
        if (laser_cloud_global->points[i].z > 0)
        {
            int u = left_intrinsics[0] * laser_cloud_global->points[i].x / laser_cloud_global->points[i].z + left_intrinsics[2];
            int v = left_intrinsics[1] * laser_cloud_global->points[i].y / laser_cloud_global->points[i].z + left_intrinsics[3];
            if (u >= width || v >= height || u < 0 || v < 0)
            {
                continue;
            }
            if (depth_img.at<uchar>(v, u) == 0 || depth_img.at<uchar>(v, u) > (int)(laser_cloud_global->points[i].z * 20))
            {
                point++;
                depth_img.at<uchar>(v, u) = (int)(laser_cloud_global->points[i].z * 20);
                // std::cout << (int)depth_img.at<uchar>(v, u) << std::endl;
            }
        }
    }
    cv::Mat in_color;
    cv::applyColorMap(depth_img, in_color, cv::COLORMAP_HOT);
    cv::cvtColor(cam, cam, cv::COLOR_GRAY2BGR);
    for (int u = 0; u < width; u++)
    {
        for (int v = 0; v < height; v++)
        {
            if (depth_img.at<uchar>(v, u) != 0)
            {
                cam.at<cv::Vec3b>(v, u) = in_color.at<cv::Vec3b>(v, u);
            }
        }
    }
    ROS_INFO("3d to 2d:%d", point);
    // cv::imshow("depth", in_color);
    cv::imshow("cam", cam);
    cv::waitKey(1);
}
