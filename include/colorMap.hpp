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
using namespace std;
typedef pcl::PointXYZI PointType;

ros::Publisher pub_colorCloud;
bool isfirstImu;
geometry_msgs::Point position_first;

sensor_msgs::PointCloud2 color_msg;

pcl::PointCloud<PointType>::Ptr depthCloud(new pcl::PointCloud<PointType>());
pcl::PointCloud<pcl::PointXYZRGB>::Ptr colorCloud(new pcl::PointCloud<pcl::PointXYZRGB>());

// tbb::concurrent_bounded_queue<nav_msgs::Odometry> poseQueue;
// tbb::concurrent_bounded_queue<sensor_msgs::Image> photo_left_q, photo_right_q;
tbb::concurrent_bounded_queue<pcl::PointXYZRGB> point_q;

parameter param;
// 获取imu的坐标系
// void odoLidar(const nav_msgs::OdometryConstPtr &odometry)
// {
//     if (!isfirstImu)
//     {
//         position_first = odometry->pose.pose.position;
//         isfirstImu = true;
//     }
//     nav_msgs::Odometry imu_now = *odometry;
//     imu_now.pose.pose.position.x = imu_now.pose.pose.position.x - position_first.x;
//     imu_now.pose.pose.position.y = imu_now.pose.pose.position.y - position_first.y;
//     imu_now.pose.pose.position.z = imu_now.pose.pose.position.z - position_first.z;
//     poseQueue.push(imu_now);
// }
// // 获取左图
// void LeftCallback(const sensor_msgs::ImageConstPtr &img_msg)
// {
//     photo_left_q.push(*img_msg);
// }
// // 获取右图
// void RightCallback(const sensor_msgs::ImageConstPtr &img_msg)
// {
//     photo_right_q.push(*img_msg);
// }
// 获取lidar点云并开始处理
void callback(const sensor_msgs::ImageConstPtr &left_msg, const sensor_msgs::ImageConstPtr &right_msg,
              const sensor_msgs::PointCloud2ConstPtr &laser_msg, const nav_msgs::OdometryConstPtr &odom_msg)
{
    LOG(INFO) << "************";
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

    // 3.匹配坐标系
    nav_msgs::Odometry curPose = *odom_msg;
    // bool _sync = false;
    // while (poseQueue.try_pop(curPose))
    // {
    //     if (fabs((laser_msg->header.stamp - curPose.header.stamp).toSec()) < 1.0 / param.imu_fre)
    //     {
    //         _sync = true;
    //         break;
    //     }
    // }
    ROS_INFO("sync!");
    // 4.计算变换矩阵ss
    Eigen::Quaterniond quaternion4(curPose.pose.pose.orientation.w,
                                   curPose.pose.pose.orientation.x,
                                   curPose.pose.pose.orientation.y,
                                   curPose.pose.pose.orientation.z);
    Eigen::Matrix3d r = quaternion4.matrix().transpose();
    LOG(INFO) << "imu pose:(" << curPose.pose.pose.position.x << ","
              << curPose.pose.pose.position.y << ","
              << curPose.pose.pose.position.z << ")";
    Eigen::Vector3d t = -r * Eigen::Vector3d(curPose.pose.pose.position.x,
                                             curPose.pose.pose.position.y,
                                             curPose.pose.pose.position.z);
    Eigen::Matrix4d world2imu, world2left, world2right;
    world2imu << r(0, 0), r(0, 1), r(0, 2), t(0),
        r(1, 0), r(1, 1), r(1, 2), t(1),
        r(2, 0), r(2, 1), r(2, 2), t(2),
        0, 0, 0, 1;
    world2left = param.imu2left * world2imu;
    world2right = param.imu2right * world2imu;
    LOG(INFO) << "imu2left:\n"
              << param.imu2left;
    LOG(INFO) << "imu2right:\n"
              << param.imu2right;
    LOG(INFO) << "world2imu:\n"
              << world2imu;
    LOG(INFO) << "world2left:\n"
              << world2left;
    LOG(INFO) << "world2right:\n"
              << world2right;
    // sensor_msgs::Image left_im, right_im;
    // right_im = *img_msg;
    // while (photo_left_q.try_pop(left_im))
    // {
    //     if (fabs((laser_msg->header.stamp - left_im.header.stamp).toSec()) > 1.0 / 20)
    //     {
    //         break;
    //     }
    // }
    // while (photo_right_q.try_pop(right_im))
    // {
    //     if (fabs((laser_msg->header.stamp - right_im.header.stamp).toSec()) > 1.0 / 20)
    //     {
    //         break;
    //     }
    // }
    // 5.进行坐标变换并着色
    for (int i = 0; i < laser_cloud_in->size(); i++)
    {
        Eigen::Vector4d worldCoord, cameraCoord_right, cameraCoord_left;
        worldCoord << laser_cloud_in->points[i].x,
            laser_cloud_in->points[i].y,
            laser_cloud_in->points[i].z,
            1;
        // 转换坐标
        cameraCoord_right = world2right * worldCoord;
        cameraCoord_left = world2left * worldCoord;
        Eigen::Vector2d point2d_left, point2d_right;
        Eigen::Vector3d point3d;
        if (cameraCoord_right[2] > 0.4 && cameraCoord_right[2] < 50)
        {
            point3d << cameraCoord_right(0), cameraCoord_right(1), cameraCoord_right(2);
            param.cam_right.spaceToPlane(point3d, point2d_right);
            if (point2d_right.x() < param.width && point2d_right.y() < param.height &&
                point2d_right.x() > 0 && point2d_right.y() > 0)
            {
                pcl::PointXYZRGB colorPoint;
                colorPoint.x = laser_cloud_in->points[i].x;
                colorPoint.y = laser_cloud_in->points[i].y;
                colorPoint.z = laser_cloud_in->points[i].z;
                colorPoint.b = right_msg->data[(int)point2d_right.y() * 640 * 3 + (int)point2d_right.x() * 3];
                colorPoint.g = right_msg->data[(int)point2d_right.y() * 640 * 3 + (int)point2d_right.x() * 3 + 1];
                colorPoint.r = right_msg->data[(int)point2d_right.y() * 640 * 3 + (int)point2d_right.x() * 3 + 2];
                // int contrast = 1;
                // colorPoint.b = (int)((float)colorPoint.b + (float)(colorPoint.b - 127) * contrast / 255);
                // colorPoint.g = (int)((float)colorPoint.g + (float)(colorPoint.b - 127) * contrast / 255);
                // colorPoint.r = (int)((float)colorPoint.r + (float)(colorPoint.b - 127) * contrast / 255);
                if (colorPoint.b + colorPoint.g + colorPoint.r > 30)
                    point_q.push(colorPoint);
            }
        }
        else if (cameraCoord_left[2] > 0.4 && cameraCoord_left[2] < 50)
        {
            point3d << cameraCoord_left(0), cameraCoord_left(1), cameraCoord_left(2);
            param.cam_left.spaceToPlane(point3d, point2d_left);
            // LOG(INFO) << "campoint3d:(" << cameraCoord(0) << "," << cameraCoord(1)
            //           << "," << cameraCoord(2) << ")";
            // LOG(INFO) << "campoint2d:(" << point2d_left.x() << "," << point2d_left.y() << ")";
            // 着色
            // uchar right_gamma[640 * 512 * 3];
            // param.GammaCorrectiom(reinterpret_cast<uchar *>(&right_im.data[0]), param.width,
            //                       param.height, right_gamma);
            if (point2d_left.x() < param.width && point2d_left.y() < param.height &&
                point2d_left.x() > 0 && point2d_left.y() > 0)
            {
                pcl::PointXYZRGB colorPoint;
                colorPoint.x = laser_cloud_in->points[i].x;
                colorPoint.y = laser_cloud_in->points[i].y;
                colorPoint.z = laser_cloud_in->points[i].z;
                colorPoint.b = left_msg->data[(int)point2d_left.y() * 640 * 3 + (int)point2d_left.x() * 3];
                colorPoint.g = left_msg->data[(int)point2d_left.y() * 640 * 3 + (int)point2d_left.x() * 3 + 1];
                colorPoint.r = left_msg->data[(int)point2d_left.y() * 640 * 3 + (int)point2d_left.x() * 3 + 2];
                // int contrast = 1;
                // colorPoint.b = (int)((float)colorPoint.b + (float)(colorPoint.b - 127) * contrast / 255);
                // colorPoint.g = (int)((float)colorPoint.g + (float)(colorPoint.b - 127) * contrast / 255);
                // colorPoint.r = (int)((float)colorPoint.r + (float)(colorPoint.b - 127) * contrast / 255);
                if (colorPoint.b + colorPoint.g + colorPoint.r > 30 )
                    point_q.push(colorPoint);
            }
        }
    }
    LOG(INFO) << "point_q size:" << point_q.size();
    while (!point_q.empty())
    {
        pcl::PointXYZRGB colorPoint;
        point_q.pop(colorPoint);
        colorCloud->push_back(colorPoint);
    }
    pcl::toROSMsg(*colorCloud, color_msg); // 将点云转化为消息才能发布
    colorCloud->clear();
    color_msg.header.frame_id = "map";
    pub_colorCloud.publish(color_msg);
}