#include "type_conversions.hpp"

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>

namespace typeconvert {
void pcl2SensorMsgToEigen(const sensor_msgs::PointCloud2 &pcl2,
                          std::vector<Eigen::Vector3d> &points) {
    sensor_msgs::PointCloud pcl;
    sensor_msgs::convertPointCloud2ToPointCloud(pcl2, pcl);
    std::for_each(pcl.points.begin(), pcl.points.end(), [&](const auto &point) {
        points.emplace_back(Eigen::Vector3d(point.x, point.y, point.z));
    });
}

void tf2ToEigen(const tf2_msgs::TFMessage &tf, Eigen::Vector3d &e) {
    auto origin = tf.transforms[0].transform.translation;
    e = Eigen::Vector3d(origin.x, origin.y, origin.z);
}
}  // namespace typeconvert