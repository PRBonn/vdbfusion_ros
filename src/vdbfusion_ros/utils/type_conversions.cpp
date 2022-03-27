#include "type_conversions.hpp"

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>

namespace typeconvert {
void pcl2SensorMsgToEigen(const sensor_msgs::PointCloud2 &pcl2,
                          std::vector<Eigen::Vector3d> &points) {
    sensor_msgs::PointCloud pcl;
    sensor_msgs::convertPointCloud2ToPointCloud(pcl2, pcl);
    for (int i = 0; i < pcl.points.size(); i++) {
        points.emplace_back(Eigen::Vector3d(pcl.points[i].x, pcl.points[i].y, pcl.points[i].z));
    }
}

void tf2ToEigen(const tf2_msgs::TFMessage &tf, Eigen::Vector3d &e) {
    auto origin = tf.transforms[0].transform.translation;
    e = Eigen::Vector3d(origin.x, origin.y, origin.z);
}
}  // namespace typeconvert