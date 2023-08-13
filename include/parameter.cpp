#include "parameter.h"
parameter::parameter(std::string file_path)
{
    YAML::Node yaml_node = YAML::LoadFile(file_path);
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
        lidar2left(i / 4, i % 4) = yaml_node["Left_cam"]["T_cam_lidar"][i / 4][i % 4].as<double>();
        lidar2right(i / 4, i % 4) = yaml_node["Right_cam"]["T_cam_lidar"][i / 4][i % 4].as<double>();
    }
    imu2left = lidar2left * lidar2imu.inverse();
    imu2right = lidar2right * lidar2imu.inverse();
    time_ = yaml_node["Time"].as<float>();
    cam_left = camodocal::CataCamera("cam", width, height,
                                     left_intri[0], left_dist[0], left_dist[1], left_dist[2], left_dist[3],
                                     left_intri[1], left_intri[2], left_intri[3], left_intri[4]);
    cam_right = camodocal::CataCamera("cam", width, height, right_intri[0],
                                      right_dist[0], right_dist[1], right_dist[2], right_dist[3],
                                      right_intri[1], right_intri[2], right_intri[3], right_intri[4]);
}
parameter::parameter()
{
}

void parameter::initLog(const char *logfile)
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