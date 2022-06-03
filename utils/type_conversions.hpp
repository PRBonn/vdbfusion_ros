#pragma once

#include <geometry_msgs/Point.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_msgs/TFMessage.h>

#include <eigen3/Eigen/Core>
#include <vector>

namespace typeconvert {
void pcl2SensorMsgToEigen(const sensor_msgs::PointCloud2 &pcl2,
                          std::vector<Eigen::Vector3d> &points);

void tf2ToEigen(const tf2_msgs::TFMessage &tf, Eigen::Vector3d &e);

}  // namespace typeconvert