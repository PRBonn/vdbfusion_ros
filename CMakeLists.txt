cmake_minimum_required(VERSION 3.0.2)
project(vdbfusion_ros)

set(CMAKE_CXX_STANDARD 17)

# System dependencies are found with CMake's conventions
include(GNUInstallDirs)
list(APPEND CMAKE_MODULE_PATH "/usr/local/lib/cmake/OpenVDB")
find_package(OpenVDB REQUIRED)
find_package(VDBFusion REQUIRED)

find_package(
  catkin REQUIRED
  COMPONENTS rospy
             roscpp
             tf2_ros
             tf2_sensor_msgs
             std_msgs
             geometry_msgs
             sensor_msgs
             message_generation)

add_service_files(FILES save_vdb_volume.srv)

generate_messages()

catkin_package(
  INCLUDE_DIRS
  ${CMAKE_CURRENT_SOURCE_DIR}/include/vdbfusion_ros/
  LIBRARIES
  CATKIN_DEPENDS
  roscpp
  rospy
  tf2_ros
  tf2_sensor_msgs
  std_msgs
  geometry_msgs
  sensor_msgs
  message_runtime)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/vdbfusion_ros)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/src)

include(3rdparty/find_dependencies.cmake)
