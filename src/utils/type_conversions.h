#pragma once

#include <geometry_msgs/Point.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <tf2_msgs/TFMessage.h>

#include <Eigen/Core>
#include <vector>

namespace typeconvert {
void pcl2SensorMsgToEigen(const sensor_msgs::PointCloud2 &pcl2,
                          std::vector<Eigen::Vector3d> &points) {
    sensor_msgs::PointCloud pcl;
    sensor_msgs::convertPointCloud2ToPointCloud(pcl2, pcl);
    std::for_each(pcl.points.begin(), pcl.points.end(), [&](const auto &point) {
        points.emplace_back(Eigen::Vector3d(point.x, point.y, point.z));
    });
}

}  // namespace typeconvert
