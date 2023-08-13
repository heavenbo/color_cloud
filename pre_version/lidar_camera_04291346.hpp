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
Eigen::Affine3f tran_lidar2camera;
Eigen::Matrix3d R_camera_imu; // 旋转矩阵
Eigen::Matrix4d imu2camera, imu2camera_inv;
Eigen::Vector3d T_lidar_world;
Eigen::Vector3d T_camera_imu;
Eigen::Vector3d t_imu_cam;
ros::Time imu_time;
// Eigen::Transform a;
int width,
    height;
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
        imu2camera(2, 0), imu2camera(2, 1), imu2camera(2, 2);
    t_imu_cam = -R_camera_imu.transpose() * Eigen::Vector3d(imu2camera(0, 3), imu2camera(1, 3), imu2camera(2, 3));
    // 得到lidar-camera的转换
    T_camera_imu = R_camera_imu.eulerAngles(2, 1, 0);
    tran_lidar2camera = pcl::getTransformation(imu2camera(0, 3), imu2camera(1, 3), imu2camera(2, 3),
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
    // std::cout << "position:" << imu_now.pose.pose.position.x << " "
    //           << imu_now.pose.pose.position.y << " "
    //           << imu_now.pose.pose.position.z << std::endl;
    imu_time = odometry->header.stamp;
    imu_mutex.unlock();
}
void lidar_callback(const sensor_msgs::PointCloud2ConstPtr &laser_msg)
{
    static tf::TransformBroadcaster broadcaster;
    // 得到世界坐标系-imu的转换
    imu_mutex.lock();
    nav_msgs::Odometry imu_now_ = imu_now;
    imu_mutex.unlock();
    Eigen::Quaterniond quaternion4(imu_now_.pose.pose.orientation.x,
                                   imu_now_.pose.pose.orientation.y,
                                   imu_now_.pose.pose.orientation.z,
                                   imu_now_.pose.pose.orientation.w);
    Eigen::Matrix3d a = quaternion4.matrix().transpose();
    T_lidar_world = -a * Eigen::Vector3d(imu_now_.pose.pose.position.x,
                                         imu_now_.pose.pose.position.y,
                                         imu_now_.pose.pose.position.z);
    // std::cout << "R:" << a << std::endl;
    Eigen::Vector3d lidar_imu = a.eulerAngles(2, 1, 0);
    // cout << "yaw(z) pitch(y) roll(x) = " << eulerAngle4.transpose() << endl;
    Eigen::Affine3f tran_world2imu = pcl::getTransformation(T_lidar_world[0], T_lidar_world[1], T_lidar_world[2],
                                                            lidar_imu[2], lidar_imu[1], lidar_imu[0]);

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
            ros::Time::now(), "world", "imu"));
    // 发布camera-imu转换
    // std::cout << "a" << std::endl;
    Eigen::Quaterniond quaternion2(R_camera_imu.transpose());
    broadcaster.sendTransform(
        tf::StampedTransform(
            tf::Transform(tf::Quaternion(quaternion2.x(), quaternion2.y(),
                                         quaternion2.z(), quaternion2.w()),
                          tf::Vector3(t_imu_cam[0], t_imu_cam[1], t_imu_cam[2])),
            ros::Time::now(), "imu", "camera"));
}