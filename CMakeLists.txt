cmake_minimum_required(VERSION 3.0.2)
project(color_cloud)
set(CMAKE_CXX_STANDARD 14)

find_package(PCL 1.10 REQUIRED)
# find_package(Ceres REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  tf
  roscpp
  sensor_msgs
  message_filters
  nav_msgs
  pcl_ros
)

include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
)
include_directories(
  /usr/include/pcl-1.10
)
add_library(camera_model STATIC
  camera_models/Camera.h
  camera_models/CataCamera.cc
  camera_models/CataCamera.h
  camera_models/gpl.h
  camera_models/Camera.cc
)

add_library(libsync STATIC
  include/parameter.h
  include/parameter.cpp
  include/colorMap_sync.h
  include/colorMap_sync.cpp
)

generate_messages(
 DEPENDENCIES
 sensor_msgs
)
add_executable(colorMap src/colorMap.cpp)
target_link_libraries(colorMap libsync ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} yaml-cpp camera_model glog tbb)

add_executable(sync_color src/sync_main.cpp)
target_link_libraries(sync_color libsync ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} 
                       yaml-cpp camera_model glog tbb)
# add_executable(test_tbb src/test_tbb.cpp)
# target_link_libraries(test_tbb  ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES}  yaml-cpp camera_model glog tbb)
