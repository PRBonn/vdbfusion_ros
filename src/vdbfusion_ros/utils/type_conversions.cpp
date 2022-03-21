#include "type_conversions.hpp"
#include <geometry_msgs/Point32.h>

namespace tf {
void pclEigenToSensorMsg(const std::vector<Eigen::Vector3d> &points,
                         sensor_msgs::PointCloud &pcl) {
  std::vector<geometry_msgs::Point32> p;
  for (int i = 0; i < points.size(); i++) {
    geometry_msgs::Point32 point;
    point.x = points[i](0);
    point.y = points[i](1);
    point.z = points[i](2);
    p.emplace_back(point);
  }
  pcl.points = p;
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

void originPointMsgToEigen(const geometry_msgs::Point &m, Eigen::Vector3d &e) {
  e = Eigen::Vector3d(m.x, m.y, m.z);
}
} // namespace tf