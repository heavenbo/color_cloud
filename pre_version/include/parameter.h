#include <Eigen/Core>
#include <Eigen/Geometry>
#include <yaml-cpp/yaml.h>
#include <Eigen/Geometry>
#include "../camera_models/CataCamera.h"
class parameter
{
public:
    int imu_fre; // imu频率
    // 相机
    double left_intri[5], left_dist[4];
    double right_intri[5], right_dist[4];
    int width, height;
    double gamma;
    unsigned char g_GammaLUT[256]; // 全局数组：包含256个元素的gamma校正查找表
    camodocal::CataCamera cam_left, cam_right;
    // 变换外参
    Eigen::Affine3f tran_left2lidar, tran_right2lidar;
    Eigen::Matrix4d left2lidar, right2lidar, imu2lidar, imu2left, imu2right;
    float time_;

    parameter(std::string file_path);
    void GammaCorrectiom(uchar *src, int iWidth, int iHeight, uchar *Dst);
};