#include "colorMap_sync.h"

// 获取imu的坐标系
ros_interface_t::ros_interface_t()
{
}
ros_interface_t::ros_interface_t(double sync_time, std::string file)
{
    this->sync_time = sync_time;
    std::string yaml_file = file + "colorCloud.yaml";
    param = parameter(yaml_file);
    std::string log_file = file + "logs/";
    param.initLog(log_file.c_str());
}
void ros_interface_t::odom_callback(const nav_msgs::OdometryConstPtr &odometry)
{
    if (!isfirstImu)
    {
        position_first = *odometry;
        isfirstImu = true;
    }
    nav_msgs::Odometry imu_now = *odometry;
    imu_now.pose.pose.position.x = imu_now.pose.pose.position.x - position_first.pose.pose.position.x;
    imu_now.pose.pose.position.y = imu_now.pose.pose.position.y - position_first.pose.pose.position.y;
    imu_now.pose.pose.position.z = imu_now.pose.pose.position.z - position_first.pose.pose.position.z;
    odom_q.push(imu_now);
}
// 获取左图
void ros_interface_t::LeftCallback(const sensor_msgs::ImageConstPtr &img_msg)
{
    left_q.push(*img_msg);
}
// 获取右图
void ros_interface_t::RightCallback(const sensor_msgs::ImageConstPtr &img_msg)
{
    right_q.push(*img_msg);
}
// 获取lidar点云并开始处理
void ros_interface_t::LidarCallback(const sensor_msgs::PointCloud2ConstPtr &laser_msg)
{
    lidar_q.push(*laser_msg);
}
// 寻找最大时间戳
int ros_interface_t::maxStamp(std::vector<double> time_q)
{
    for (int i = 0; i < time_q.size(); i++)
    {
        for (int j = 0; j < time_q.size(); j++)
        {
            if (time_q[i] < time_q[j])
            {
                break;
            }
            else if (j == time_q.size() - 1)
            {
                return i;
            }
        }
    }
    return -1;
}
// sync
bool ros_interface_t::sync()
{
    // if (!depth_q.empty() && !odom_q.empty() && !img_q.empty())//不需要判断了
    bool ispush = false;
    LOG(INFO) << "********************";
    left_q.pop(this->left_msg);
    right_q.pop(this->right_msg);
    odom_q.pop(this->odom_msg);
    lidar_q.pop(this->lidar_msg);

    // 排序:img depth odom
    std::vector<double> time_q;
    time_q.push_back(this->left_msg.header.stamp.toSec());
    time_q.push_back(this->right_msg.header.stamp.toSec());
    time_q.push_back(this->odom_msg.header.stamp.toSec());
    time_q.push_back(this->lidar_msg.header.stamp.toSec());

    int i = maxStamp(time_q);
    // ROS_INFO("img_msg:%lf,depth_msg:%lf,odom_msg:%lf", img_msg.header.stamp.toSec(),
    //          depth_msg.header.stamp.toSec(), odom_msg.header.stamp.toSec());
    // 判断是否符合同步要求
    if (fabs(left_msg.header.stamp.toSec() - lidar_msg.header.stamp.toSec()) < sync_time &&
        fabs(left_msg.header.stamp.toSec() - odom_msg.header.stamp.toSec()) < sync_time &&
        fabs(lidar_msg.header.stamp.toSec() - odom_msg.header.stamp.toSec()) < sync_time &&
        fabs(left_msg.header.stamp.toSec() - right_msg.header.stamp.toSec()) < sync_time)
    {
        LOG(INFO) << "success sync";
        return true;
    }
    else
    {
        switch (i)
        {
        case 0:
        {
            if (left_q.empty())
            {
                left_q.push(left_msg);
                ispush = true;
            }
            break;
        }
        case 1:
        {
            if (right_q.empty())
            {
                right_q.push(right_msg);
                ispush = true;
            }
            break;
        }
        case 2:
        {
            if (odom_q.empty())
            {
                odom_q.push(odom_msg);
                ispush = true;
            }
            break;
        }
        case 3:
        {
            if (lidar_q.empty())
            {
                lidar_q.push(lidar_msg);
                ispush = true;
            }

            break;
        }
        default:
            break;
        }
        while (i != 0 && fabs(time_q[i] - left_msg.header.stamp.toSec()) > sync_time)
        {
            if (!left_q.try_pop(left_msg)) // pop雷达队列直到空
            {
                break;
            }
        }
        while (i != 1 && fabs(time_q[i] - right_msg.header.stamp.toSec()) > sync_time)
        {
            if (!right_q.try_pop(right_msg)) // pop雷达队列直到空
            {
                break;
            }
        }
        while (i != 2 && fabs(time_q[i] - odom_msg.header.stamp.toSec()) > sync_time)
        {
            if (!odom_q.try_pop(odom_msg)) // pop位姿队列直到空
            {
                break;
            }
        }
        while (i != 3 && fabs(time_q[i] - lidar_msg.header.stamp.toSec()) > sync_time)
        {
            if (!lidar_q.try_pop(lidar_msg)) // pop相机队列直到空
            {
                break;
            }
        }

        if (fabs(left_msg.header.stamp.toSec() - lidar_msg.header.stamp.toSec()) < sync_time &&
            fabs(left_msg.header.stamp.toSec() - odom_msg.header.stamp.toSec()) < sync_time &&
            fabs(lidar_msg.header.stamp.toSec() - odom_msg.header.stamp.toSec()) < sync_time &&
            fabs(left_msg.header.stamp.toSec() - right_msg.header.stamp.toSec()) < sync_time)
        {
            LOG(INFO) << "success sync";
            // 同步成功，释放信息戳队列最老的消息
            switch (i)
            {
            case 0:
                if (ispush)
                    left_q.pop(left_msg);
                break;
            case 1:
                if (ispush)
                    right_q.pop(right_msg);
                break;
            case 2:
                if (ispush)
                    odom_q.pop(odom_msg);
                break;
            case 3:
                if (ispush)
                    lidar_q.pop(lidar_msg);
                break;
            default:
                break;
            }
            return true;
        }
        else
        {
            LOG(INFO) << "false,size:" << left_q.size() << "\t"
                      << "left_msg:" << std::setprecision(15)
                      << left_msg.header.stamp.toSec();
            LOG(INFO) << "false,size:" << right_q.size() << "\t"
                      << "right_msg:" << std::setprecision(15)
                      << right_msg.header.stamp.toSec();
            LOG(INFO) << "false,size:" << odom_q.size() << "\t"
                      << "odom_msg:" << std::setprecision(15)
                      << odom_msg.header.stamp.toSec();
            LOG(INFO) << "false,size:" << lidar_q.size() << "\t"
                      << "lidar_msg:" << std::setprecision(15)
                      << lidar_msg.header.stamp.toSec();
            return false;
        }
    }
}

//
void ros_interface_t::handle_timer(const ros::TimerEvent &time_e)
{
    if (!left_q.empty() && !odom_q.empty() && !right_q.empty() && !lidar_q.empty())
    {
        if (sync())
        {
            ROS_INFO("sync!");
            pcl::PointCloud<PointType>::Ptr laser_cloud_in(new pcl::PointCloud<PointType>());
            pcl::fromROSMsg(lidar_msg, *laser_cloud_in);

            // 2. downsample new cloud (save memory)
            pcl::PointCloud<PointType>::Ptr laser_cloud_in_ds(new pcl::PointCloud<PointType>());
            static pcl::VoxelGrid<PointType> downSizeFilter;
            downSizeFilter.setLeafSize(0.3, 0.3, 0.3);
            downSizeFilter.setInputCloud(laser_cloud_in);
            downSizeFilter.filter(*laser_cloud_in_ds);
            *laser_cloud_in = *laser_cloud_in_ds;

            // 3.计算变换矩阵ss
            Eigen::Quaterniond quaternion4(odom_msg.pose.pose.orientation.w,
                                           odom_msg.pose.pose.orientation.x,
                                           odom_msg.pose.pose.orientation.y,
                                           odom_msg.pose.pose.orientation.z);
            Eigen::Matrix3d r = quaternion4.matrix().transpose();
            LOG(INFO) << "imu pose:(" << odom_msg.pose.pose.position.x << ","
                      << odom_msg.pose.pose.position.y << ","
                      << odom_msg.pose.pose.position.z << ")";
            Eigen::Vector3d t = -r * Eigen::Vector3d(odom_msg.pose.pose.position.x,
                                                     odom_msg.pose.pose.position.y,
                                                     odom_msg.pose.pose.position.z);
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
            // 5.进行坐标变换并着色
            std::queue<pcl::PointXYZRGB> point_q;
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
                        colorPoint.b = right_msg.data[(int)point2d_right.y() * 640 * 3 + (int)point2d_right.x() * 3];
                        colorPoint.g = right_msg.data[(int)point2d_right.y() * 640 * 3 + (int)point2d_right.x() * 3 + 1];
                        colorPoint.r = right_msg.data[(int)point2d_right.y() * 640 * 3 + (int)point2d_right.x() * 3 + 2];
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
                        colorPoint.b = left_msg.data[(int)point2d_left.y() * 640 * 3 + (int)point2d_left.x() * 3];
                        colorPoint.g = left_msg.data[(int)point2d_left.y() * 640 * 3 + (int)point2d_left.x() * 3 + 1];
                        colorPoint.r = left_msg.data[(int)point2d_left.y() * 640 * 3 + (int)point2d_left.x() * 3 + 2];
                        // int contrast = 1;
                        // colorPoint.b = (int)((float)colorPoint.b + (float)(colorPoint.b - 127) * contrast / 255);
                        // colorPoint.g = (int)((float)colorPoint.g + (float)(colorPoint.b - 127) * contrast / 255);
                        // colorPoint.r = (int)((float)colorPoint.r + (float)(colorPoint.b - 127) * contrast / 255);
                        if (colorPoint.b + colorPoint.g + colorPoint.r > 30)
                            point_q.push(colorPoint);
                    }
                }
            }
            LOG(INFO) << "point_q size:" << point_q.size();
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr colorCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
            while (!point_q.empty())
            {
                colorCloud->push_back(point_q.front());
                point_q.pop();
            }
            sensor_msgs::PointCloud2 color_msg;
            pcl::toROSMsg(*colorCloud, color_msg); // 将点云转化为消息才能发布
            colorCloud->clear();
            color_msg.header.frame_id = "map";
            pub_colorCloud.publish(color_msg);
        }
    }
}