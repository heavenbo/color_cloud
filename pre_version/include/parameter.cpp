#include "parameter.h"
parameter::parameter(std::string file_path)
{
    YAML::Node yaml_node = YAML::LoadFile(file_path);
    gamma = yaml_node["Gamma"].as<double>();
    float f;
    for (int i = 0; i < 256; i++)
    {
        f = (i + 0.5F) / 256; // 归一化
        f = (float)pow(f, 1 / gamma);
        g_GammaLUT[i] = (unsigned char)(f * 256 - 0.5F); // 反归一化
    }
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
        imu2lidar(i / 4, i % 4) = yaml_node["T_lidar_imu"][i / 4][i % 4].as<double>();
        left2lidar(i / 4, i % 4) = yaml_node["Left_cam"]["T_cam_lidar"][i / 4][i % 4].as<double>();
        right2lidar(i / 4, i % 4) = yaml_node["Right_cam"]["T_cam_lidar"][i / 4][i % 4].as<double>();
    }
    imu2left = imu2lidar * left2lidar.inverse();
    imu2right = imu2lidar * right2lidar.inverse();
    time_ = yaml_node["Time"].as<float>();
    cam_left = camodocal::CataCamera("cam", width, height,
                                     left_intri[0], left_dist[0], left_dist[1], left_dist[2], left_dist[3],
                                     left_intri[1], left_intri[2], left_intri[3], left_intri[4]);
    cam_right = camodocal::CataCamera("cam", width, height, right_intri[0],
                                      right_dist[0], right_dist[1], right_dist[2], right_dist[3],
                                      right_intri[1], right_intri[2], right_intri[3], right_intri[4]);
}

void parameter::GammaCorrectiom(uchar *src, int iWidth, int iHeight, uchar *Dst)
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