cmake_minimum_required(VERSION 3.0.2)
project(vdbfusion_ros)

# Compile as C++11, supported in ROS Kinetic and newer add_compile_options(-std=c++11)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

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

# System dependencies are found with CMake's conventions
list(APPEND CMAKE_MODULE_PATH "/usr/local/lib/cmake/OpenVDB")
find_package(OpenVDB REQUIRED)
find_package(VDBFusion REQUIRED)
if(VDBFusion_FOUND)
  set(VDBFusion_INCLUDE_DIRS "/usr/local/include/vdbfusion")
endif()
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)

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

include_directories(${PROJECT_SOURCE_DIR}/utils ${PROJECT_SOURCE_DIR}/include ${catkin_INCLUDE_DIRS}
                    ${VDBFusion_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

include(3rdparty/find_dependencies.cmake)
