#include "type_conversions.hpp"

#include <geometry_msgs/Point32.h>

namespace tf {
void pclEigenToSensorMsg(const std::vector<Eigen::Vector3d> &points,
                         sensor_msgs::PointCloud &pcl) {
  geometry_msgs::Point32 p[points.size()];
  for (int i = 0; i < points.size(); i++) {
    p[i].x = points[i](0);
    p[i].y = points[i](1);
    p[i].z = points[i](2);
  }
}

void pclSensorMsgToEigen(const sensor_msgs::PointCloud &pcl,
                         std::vector<Eigen::Vector3d> &points) {
  for (int i = 0; i < pcl.points.size(); i++) {
    points.emplace_back(
        Eigen::Vector3d(pcl.points[i].x, pcl.points[i].y, pcl.points[i].z));
  }
}

void originEigenToPointMsg(const Eigen::Vector3d &e, geometry_msgs::Point &m) {
  m.x = e(0);
  m.y = e(1);
  m.z = e(2);
}
} // namespace tf