#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/ChannelFloat32.h>
#include <tbb/concurrent_queue.h>
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
#include "../camera_models/CataCamera.h"
#include <glog/logging.h>
typedef pcl::PointXYZI PointType;

// sensor_msgs::Image depth_img;
ros::Publisher pub_colorCloud;
std::mutex imu_mutex; // 锁imu坐标系
bool isfirstImu;
nav_msgs::Odometry lidar_now;
geometry_msgs::Point position_first;
std::mutex cam_mutex; // 锁相机
sensor_msgs::PointCloud2 color_msg;
std::deque<nav_msgs::Odometry> imuQueue; // imu位姿队列
std::deque<pcl::PointCloud<PointType>> cloudQueue;
pcl::PointCloud<PointType>::Ptr depthCloud(new pcl::PointCloud<PointType>());
pcl::PointCloud<pcl::PointXYZRGB>::Ptr colorCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
std::deque<double> timeQueue;
int imu_fre;

double left_intri[5], left_dist[4];
double right_intri[5], right_dist[4];
Eigen::Affine3f tran_world2lidar, tran_lidar2cam;
Eigen::Matrix3d R_camera_lidar; // 旋转矩阵
Eigen::Matrix4d imu2camera, world2lidar, lidar2camera, world2camera, lidar2imu;
Eigen::Vector3d T_imu_world, T_camera_lidar, t_imu_cam;
Eigen::Vector4d worldCoord, cameraCoord;
// Eigen::Transform a;
int width, height;
float time_;
cv::Mat cam;
// uchar *cam_left_;
// uchar *cam_le;
double xi, k1, k2, p1, p2, gamma1, gamma2, u0, v0;
camodocal::CataCamera cam_left;
camodocal::CataCamera cam_right;
unsigned char g_GammaLUT[256]; // 全局数组：包含256个元素的gamma校正查找表
tbb::concurrent_bounded_queue<sensor_msgs::Image> photo_left_q, photo_right_q, imu_odomotry;
void BuildTable(float fPrecompensation)
{
    int i;
    float f;
    for (i = 0; i < 256; i++)
    {
        f = (i + 0.5F) / 256; // 归一化
        f = (float)pow(f, fPrecompensation);
        g_GammaLUT[i] = (unsigned char)(f * 256 - 0.5F); // 反归一化
    }
}

void GammaCorrectiom(uchar *src, int iWidth, int iHeight, uchar *Dst)
{
    int iCols, iRows;
    // 对图像的每个像素进行查找表矫正
    for (iRows = 0; iRows < iHeight; iRows++)
    {
        int pixelsNums = iRows * iWidth * 3;
        for (iCols = 0; iCols < iWidth * 3; iCols++)
        {
            // Dst[iRows*iWidth+iCols]=g_GammaLUT[src[iRows*iWidth+iCols]];
            Dst[pixelsNums + iCols] = g_GammaLUT[src[pixelsNums + iCols]];
        }
    }
}

void Init(std::string file_path)
{
    YAML::Node yaml_node = YAML::LoadFile(file_path);
    // cam_left_ = new uchar[640 * 512 * 3];
    // cam_le = new uchar[640 * 512 * 3];
    BuildTable(1 / 2.0);
    width = yaml_node["Width"].as<double>();
    height = yaml_node["Height"].as<double>();
    imu_fre = yaml_node["Imu_fre"].as<int>();
    for (int i = 0; i < 4; i++)
    {
        left_dist[i] = yaml_node["Left_cam"]["distortion_coeffs"][i].as<double>();
        right_dist[i] = yaml_node["Right_cam"]["distortion_coeffs"][i].as<double>();
    }
    for (int i = 0; i < 5; i++)
    {
        left_intri[i] = yaml_node["Left_cam"]["intrinsics"][i].as<double>();
        right_intri[i] = yaml_node["Right_cam"]["intrinsics"][i].as<double>();
    }
    for (int i = 0; i < 16; i++)
    {
        lidar2imu(i / 4, i % 4) = yaml_node["T_lidar_imu"][i / 4][i % 4].as<double>();
        imu2camera(i / 4, i % 4) = yaml_node["Left_cam"]["T_cam_imu"][i / 4][i % 4].as<double>();
    }
    lidar2camera = imu2camera * lidar2imu;
    // camera2lidar=camera2imu*imu2lidar;
    // T_lidar_cam = T_lidar_imu * T_imu_cam
    //
    time_ = yaml_node["Time"].as<float>();
    cam_left = camodocal::CataCamera("cam", width, height,
                                     left_intri[0], left_dist[0], left_dist[1], left_dist[2], left_dist[3],
                                     left_intri[1], left_intri[2], left_intri[3], left_intri[4]);
    cam_right = camodocal::CataCamera("cam", width, height, right_intri[0],
                                      right_dist[0], right_dist[1], right_dist[2], right_dist[3],
                                      right_intri[1], right_intri[2], right_intri[3], right_intri[4]);
}
// 获取lidar的坐标系
void odoLidar(const nav_msgs::OdometryConstPtr &odometry)
{
    if (!isfirstImu)
    {
        position_first = odometry->pose.pose.position;
        isfirstImu = true;
    }
    imu_mutex.lock();
    lidar_now = *odometry;
    lidar_now.pose.pose.position.x = lidar_now.pose.pose.position.x - position_first.x;
    lidar_now.pose.pose.position.y = lidar_now.pose.pose.position.y - position_first.y;
    lidar_now.pose.pose.position.z = lidar_now.pose.pose.position.z - position_first.z;
    imuQueue.push_back(lidar_now);
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

    // 3.匹配坐标系
    imu_mutex.lock();
    for (int i = 0; i < imuQueue.size(); i++)
    {
        if (fabs((laser_msg->header.stamp - imuQueue.front().header.stamp).toSec()) > 1.0 / imu_fre)
        {
            imuQueue.pop_front();
        }
        else
        {
            break;
        }
    }
    imu_mutex.unlock();

    // 4.计算变换矩阵
    Eigen::Quaterniond quaternion4(imuQueue.front().pose.pose.orientation.w,
                                   imuQueue.front().pose.pose.orientation.x,
                                   imuQueue.front().pose.pose.orientation.y,
                                   imuQueue.front().pose.pose.orientation.z);
    Eigen::Matrix3d a = quaternion4.matrix();
    world2lidar << a(0, 0), a(0, 1), a(0, 2), imuQueue.front().pose.pose.position.x,
        a(1, 0), a(1, 1), a(1, 2), imuQueue.front().pose.pose.position.y,
        a(2, 0), a(2, 1), a(2, 2), imuQueue.front().pose.pose.position.z,
        0, 0, 0, 1;
    world2camera = lidar2camera * world2lidar;
    // 5.进行坐标变换并着色
    cam_mutex.lock();
    // memcpy(cam_le, cam_left_, 640 * 512 * 3);
    cv::Mat cam_now = cam.clone();
    cam_mutex.unlock();
    cv::imshow("cam", cam_now);
    cv::waitKey(1);
    for (int i = 0; i < laser_cloud_in->size(); i++)
    {
        worldCoord << laser_cloud_in->points[i].x,
            laser_cloud_in->points[i].y,
            laser_cloud_in->points[i].z,
            1;
        // 转换坐标
        Eigen::Vector4d lidarCoord = world2lidar * worldCoord;
        // std::cout << world2lidar << std::endl
        //           << std::endl;
        cameraCoord = world2camera * worldCoord;
        if (cameraCoord[0] <= 0)
        {
            continue;
        }
        Eigen::Vector2d point2d;
        Eigen::Vector3d point3d;
        point3d << cameraCoord(0), cameraCoord(1), cameraCoord(2);
        cam_left.spaceToPlane(point3d, point2d);
        // 着色
        if (point2d.x() < width && point2d.y() < height && point2d.x() > 0 && point2d.y() > 0)
        {
            pcl::PointXYZRGB colorPoint;
            colorPoint.x = laser_cloud_in->points[i].x;
            colorPoint.y = laser_cloud_in->points[i].y;
            colorPoint.z = laser_cloud_in->points[i].z;
            colorPoint.b = cam_now.at<cv::Vec3b>((int)point2d.y(), (int)point2d.x())[0];
            colorPoint.g = cam_now.at<cv::Vec3b>((int)point2d.y(), (int)point2d.x())[1];
            colorPoint.r = cam_now.at<cv::Vec3b>((int)point2d.y(), (int)point2d.x())[2];
            // colorPoint.b = 0;
            // colorPoint.g = 0;
            // colorPoint.r = 0;
            colorCloud->push_back(colorPoint);
            LOG(INFO) << "point3d:(" << lidarCoord(0) << "," << lidarCoord(1) << "," << lidarCoord(2) << ")";
            LOG(INFO) << "point2d:(" << point2d.x() << "," << point2d.y() << ")";
            LOG(INFO) << "bgr:(" << (int)colorPoint.b << "," << (int)colorPoint.g << "," << (int)colorPoint.r << ")";
            LOG(INFO) << "";
        }
    }
    pcl::toROSMsg(*colorCloud, color_msg); // 将点云转化为消息才能发布
    color_msg.header.frame_id = "cam";
    pub_colorCloud.publish(color_msg);
}

void img_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    cam_mutex.lock();
    // 获取图片并去畸变
    cam = cv::Mat(img_msg->height, img_msg->width, CV_8UC3, const_cast<uchar *>(&img_msg->data[0]), img_msg->step);
    // memcpy(cam_left_, const_cast<uchar *>(&img_msg->data[0]), 640 * 512 * 3);
    cam_mutex.unlock();
}

void initLog(const char *logfile)
{
    google::InitGoogleLogging("cam");
    google::SetLogDestination(google::GLOG_INFO, logfile);
    google::SetStderrLogging(google::GLOG_WARNING);
    google::SetLogFilenameExtension("log_");
    FLAGS_colorlogtostderr = true;          // Set log color
    FLAGS_logbufsecs = 0;                   // Set log output speed(s)
    FLAGS_max_log_size = 1024;              // Set max log file size
    FLAGS_stop_logging_if_full_disk = true; // If disk is full
}
