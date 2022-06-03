cmake_minimum_required(VERSION 3.0.2)
project(vdbfusion_ros)

# System dependencies are found with CMake's conventions
include(GNUInstallDirs)
list(APPEND CMAKE_MODULE_PATH "/usr/local/lib/cmake/OpenVDB")
find_package(OpenVDB REQUIRED)
find_package(VDBFusion REQUIRED)
find_package(yaml-cpp REQUIRED) # TODO: Move to 3rdparty dependencies
find_package(Eigen3 REQUIRED) # TODO: Move to 3rdparty dependencies

find_package(
  catkin REQUIRED
  COMPONENTS roscpp
             rospy
             tf2_sensor_msgs
             tf2_ros
             std_msgs
             geometry_msgs
             sensor_msgs
             message_generation)

add_service_files(FILES save_volume.srv)

generate_messages()

catkin_package(
  INCLUDE_DIRS
  ${PROJECT_SOURCE_DIR}/include
  LIBRARIES
  type_conversions
  CATKIN_DEPENDS
  roscpp
  rospy
  tf2_ros
  tf2_sensor_msgs
  std_msgs
  geometry_msgs
  sensor_msgs
  message_runtime
  DEPENDS
  VDBFusion::vdbfusion
  Eigen3::Eigen)

include_directories(${PROJECT_SOURCE_DIR}/utils ${PROJECT_SOURCE_DIR}/include ${catkin_INCLUDE_DIRS})
include(3rdparty/find_dependencies.cmake)
