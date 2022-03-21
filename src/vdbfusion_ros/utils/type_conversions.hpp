#pragma once

#include <eigen3/Eigen/Core>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/PointCloud.h>

#include <vector>

namespace tf {
void pclEigenToSensorMsg(const std::vector<Eigen::Vector3d> &points,
                         sensor_msgs::PointCloud &pcl);
void pclSensorMsgToEigen(const sensor_msgs::PointCloud &pcl,
                         std::vector<Eigen::Vector3d> &points);

void originEigenToPointMsg(const Eigen::Vector3d &e, geometry_msgs::Point &m);
void originPointMsgToEigen(const geometry_msgs::Point &m, Eigen::Vector3d &e);

} // namespace tf