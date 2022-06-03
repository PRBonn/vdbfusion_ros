cmake_minimum_required(VERSION 3.0.2)
project(vdbfusion_ros)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(catkin REQUIRED COMPONENTS
roscpp
rospy
tf2_sensor_msgs
tf2_ros
std_msgs
geometry_msgs
sensor_msgs
message_generation
)

## System dependencies are found with CMake's conventions
list(APPEND CMAKE_MODULE_PATH "/usr/local/lib/cmake/OpenVDB")
find_package(OpenVDB REQUIRED)
find_package(VDBFusion REQUIRED)
if(VDBFusion_FOUND)
  set(VDBFusion_INCLUDE_DIRS "/usr/local/include/vdbfusion")
endif()
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################
add_service_files(
  FILES
  save_volume.srv
)

generate_messages()

catkin_package(
  INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include
  LIBRARIES type_conversions
  CATKIN_DEPENDS roscpp rospy tf2_ros tf2_sensor_msgs std_msgs geometry_msgs sensor_msgs message_runtime
  DEPENDS VDBFusion::vdbfusion Eigen3::Eigen
)

###########
## Build ##
###########
include_directories(
  ${PROJECT_SOURCE_DIR}/utils
  ${PROJECT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
  ${VDBFusion_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

include(3rdparty/find_dependencies.cmake)


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_vdbfusion_ros.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
