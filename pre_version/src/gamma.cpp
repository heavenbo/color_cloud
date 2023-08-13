#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/ChannelFloat32.h>
#include <opencv2/opencv.hpp>
cv::Mat cam_left, cam_right;
ros::Publisher camera_left, camera_right;
sensor_msgs::Image img_left, img_right;
unsigned char g_GammaLUT[256]; // 全局数组：包含256个元素的gamma校正查找表
void init()
{
    img_left.header.frame_id = "cam0";
    img_left.height = 512;
    img_left.width = 640;
    img_left.encoding = "bgr8";
    img_left.is_bigendian = false;
    img_left.step = 640 * 3;
    img_left.data.resize(img_left.step * 512);
    // 右图
    img_right.header.frame_id = "cam1";
    img_right.height = 512;
    img_right.width = 640;
    img_right.encoding = "bgr8";
    img_right.is_bigendian = false;
    img_right.step = 640 * 3;
    img_right.data.resize(img_right.step * 512);
}
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

void img_left_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    // 获取图片并去畸变
    cam_left = cv::Mat(img_msg->height, img_msg->width, CV_8UC3, const_cast<uchar *>(&img_msg->data[0]), img_msg->step);

    // remap(cam, cam, map1, map2, cv::INTER_LINEAR);
    GammaCorrectiom(cam_left.data, cam_left.cols, cam_left.rows, cam_left.data);
    // std::cout << img_msg->height << " " << img_msg->width << std::endl;
    memcpy(reinterpret_cast<uchar *>(&img_left.data[0]), cam_left.data, 640 * 512 * 3);
    // std::cout << "2" << std::endl;
    img_left.header = img_msg->header;
    camera_left.publish(img_left);
}

void img_right_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    // 获取图片并去畸变
    cam_right = cv::Mat(img_msg->height, img_msg->width, CV_8UC3, const_cast<uchar *>(&img_msg->data[0]), img_msg->step);

    // remap(cam, cam, map1, map2, cv::INTER_LINEAR);
    GammaCorrectiom(cam_right.data, cam_right.cols, cam_right.rows, cam_right.data);
    // std::cout << img_msg->height << " " << img_msg->width << std::endl;
    memcpy(reinterpret_cast<uchar *>(&img_right.data[0]), cam_right.data, 640 * 512 * 3);
    // std::cout << "2" << std::endl;
    img_right.header = img_msg->header;
    camera_right.publish(img_right);
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "gamma");
    ros::NodeHandle n;
    init();
    BuildTable(1 / 2.0);
    ros::Subscriber sub_camera_left = n.subscribe("/cam/left/bgr", 10, img_left_callback);
    ros::Subscriber sub_camera_right = n.subscribe("/cam/right/bgr", 10, img_right_callback);
    camera_left = n.advertise<sensor_msgs::Image>("/cam/left/bgr_gamma", 1000);
    camera_right = n.advertise<sensor_msgs::Image>("/cam/right/bgr_gamma", 1000);
    
    ros::spin();
}