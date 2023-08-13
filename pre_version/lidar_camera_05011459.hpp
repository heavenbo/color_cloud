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

double left_intrinsics[4], left_dist[4];
Eigen::Affine3f tran_imu2camera, tran_lidar2imu;
Eigen::Matrix3d R_camera_imu, R_imu_lidar; // 旋转矩阵
Eigen::Matrix4d imu2camera, imu2camera_inv, lidar2imu;
Eigen::Vector3d T_imu_world;
// Eigen::Transform a;
int width, height;
float time_;
cv::Mat map1, map2;
void Init(std::string file_path)
{
    YAML::Node yaml_node = YAML::LoadFile(file_path);
    width = yaml_node["Width"].as<double>();
    height = yaml_node["Height"].as<double>();
    for (int i = 0; i < 4; i++)
    {
        left_intrinsics[i] = yaml_node["Left_cam"]["intrinsics"][i].as<double>();
        left_dist[i] = yaml_node["Left_cam"]["distortion"][i].as<double>();
    }
    for (int i = 0; i < 16; i++)
    {
        imu2camera(i / 4, i % 4) = yaml_node["Left_cam"]["T_cam_imu"][i / 4][i % 4].as<double>();
        lidar2imu(i / 4, i % 4) = yaml_node["T_imu_lidar"][i / 4][i % 4].as<double>();
    }
    time_ = yaml_node["Time"].as<float>();

    std::cout << (lidar2imu * imu2camera).inverse() << std::endl;
    R_camera_imu << imu2camera(0, 0), imu2camera(0, 1), imu2camera(0, 2),
        imu2camera(1, 0), imu2camera(1, 1), imu2camera(1, 2),
        imu2camera(2, 0), imu2camera(2, 1), imu2camera(2, 2); // imu坐标到cam坐标的转换R

    R_imu_lidar << lidar2imu(0, 0), lidar2imu(0, 1), lidar2imu(0, 2),
        lidar2imu(1, 0), lidar2imu(1, 1), lidar2imu(1, 2),
        lidar2imu(2, 0), lidar2imu(2, 1), lidar2imu(2, 2); // lidar坐标到imu坐标的转换R
    // 得到imu-camera坐标的转换
    Eigen::Vector3d T_camera_imu = R_camera_imu.eulerAngles(2, 1, 0);
    tran_imu2camera = pcl::getTransformation(imu2camera(0, 3), imu2camera(1, 3), imu2camera(2, 3),
                                             T_camera_imu[2], T_camera_imu[1], T_camera_imu[0]);
    // Eigen::Vector3d t_imu_lidar = Eigen::Vector3d(lidar2imu(0, 3), lidar2imu(1, 3),
    //                                               lidar2imu(2, 3));
    // Eigen::Vector3d ang_imu_lidar = R_imu_lidar.eulerAngles(2, 1, 0);
    Eigen::Vector3d t_imu_lidar = -R_imu_lidar.transpose() * Eigen::Vector3d(lidar2imu(0, 3), lidar2imu(1, 3),
                                                                             lidar2imu(2, 3));
    Eigen::Vector3d ang_imu_lidar = R_imu_lidar.transpose().eulerAngles(2, 1, 0);
    tran_lidar2imu = pcl::getTransformation(t_imu_lidar[0], t_imu_lidar[1], t_imu_lidar[2],
                                            ang_imu_lidar[2], ang_imu_lidar[1], ang_imu_lidar[0]);
}
void undiost_Init()
{
    const cv::Mat K = (cv::Mat_<double>(3, 3) << left_intrinsics[0], 0, left_intrinsics[2],
                       0, left_intrinsics[1], left_intrinsics[3],
                       0, 0, 1);
    const cv::Mat D = (cv::Mat_<double>(5, 1) << left_dist[0], left_dist[1], 0.0,
                       left_dist[2], left_dist[3]);

    cv::Size imageSize(width, height);
    const double alpha = 1; //

    cv::Mat NewCameraMatrix = getOptimalNewCameraMatrix(K, D, imageSize, alpha, imageSize, 0);
    initUndistortRectifyMap(K, D, cv::Mat(), NewCameraMatrix, imageSize, CV_16SC2, map1, map2);
}
void lidar_callback(const sensor_msgs::PointCloud2ConstPtr &laser_msg)
{
    // 1. convert laser cloud message to pcl
    pcl::PointCloud<PointType>::Ptr laser_cloud_in(new pcl::PointCloud<PointType>());
    pcl::fromROSMsg(*laser_msg, *laser_cloud_in);

    // // 2. downsample new cloud (save memory)
    // pcl::PointCloud<PointType>::Ptr laser_cloud_in_ds(new pcl::PointCloud<PointType>());
    // static pcl::VoxelGrid<PointType> downSizeFilter;
    // downSizeFilter.setLeafSize(0.3, 0.3, 0.3);
    // downSizeFilter.setInputCloud(laser_cloud_in);
    // downSizeFilter.filter(*laser_cloud_in_ds);
    // *laser_cloud_in = *laser_cloud_in_ds;

    // 8. fuse cloud
    std::lock_guard<std::mutex> lock(lidar_mutex);
    depthCloud->clear();
    *depthCloud = *laser_cloud_in;
}
void img_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    static tf::TransformBroadcaster broadcaster;
    // 得到lidar-imu的转换
    cv::Mat cam = cv::Mat(img_msg->height, img_msg->width, CV_8UC1, const_cast<uchar *>(&img_msg->data[0]), img_msg->step);

    // 1.lidar本体坐标系下的点云，转到imu坐标系下表示
    lidar_mutex.lock();
    pcl::PointCloud<PointType>::Ptr laser_cloud_offset(new pcl::PointCloud<PointType>());
    pcl::transformPointCloud(*depthCloud, *laser_cloud_offset, tran_lidar2imu);
    lidar_mutex.unlock();
    // 3.  imu坐标系下的点云，转到vins的world系下表示
    pcl::PointCloud<PointType>::Ptr laser_cloud_global(new pcl::PointCloud<PointType>());
    pcl::transformPointCloud(*laser_cloud_offset, *laser_cloud_global, tran_imu2camera);
    ROS_INFO("depthCloud SIZE:%ld", laser_cloud_global->size());
    // 4.三维点映射为像素，并绘制depth图
    int point = 0;
    cv::Mat depth_img = cv::Mat::zeros(height, width, CV_8UC1);
    for (int i = 0; i < (int)laser_cloud_global->size(); ++i)
    {
        if (laser_cloud_global->points[i].z > 0)
        {
            int u = left_intrinsics[0] * laser_cloud_global->points[i].x / laser_cloud_global->points[i].z + left_intrinsics[2];
            int v = left_intrinsics[1] * laser_cloud_global->points[i].y / laser_cloud_global->points[i].z + left_intrinsics[3];
            // std::cout << "u:" << u << "v:" << v << std::endl;
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
    remap(cam, cam, map1, map2, cv::INTER_LINEAR);
    cv::cvtColor(cam, cam, cv::COLOR_GRAY2BGR);
    for (int u = 0; u < width; u++)
    {
        for (int v = 0; v < height; v++)
        {
            if (depth_img.at<uchar>(v, u) != 0)
            {
                cv::circle(cam, cv::Point(u, v), 2, cv::Scalar(in_color.at<cv::Vec3b>(v, u)));
                // cam.at<cv::Vec3b>(v, u) = in_color.at<cv::Vec3b>(v, u);
            }
        }
    }
    ROS_INFO("3d to 2d:%d", point);
    // cv::imshow("depth", in_color);
    cv::imshow("cam", cam);
    cv::waitKey(1);
}
