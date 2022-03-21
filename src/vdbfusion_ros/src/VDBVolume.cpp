#include "vdbfusion/VDBVolume.h"
#include "yaml-cpp/yaml.h"
#include <fstream>
#include <iostream>
#include <openvdb/openvdb.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "type_conversions.hpp"

namespace vdbfusion {
class VDBVolumeROS : public VDBVolume {
public:
  VDBVolumeROS(float voxel_size, float sdf_trunc, bool space_carving = false)
      : VDBVolume(voxel_size, sdf_trunc, space_carving) {}

  void integrateCallback(const sensor_msgs::PointCloud &pcl) {
    std::vector<Eigen::Vector3d> points;
    tf::pclSensorMsgToEigen(pcl, points);
    this->Integrate(points, Eigen::Vector3d(0, 0, 0),
                    [](float /*unused*/) { return 1.0; });
  }
};
} // namespace vdbfusion

int main(int argc, char **argv) {
  float voxel_size = 0.1;
  float sdf_trunc = 0.3;
  bool space_carving = false;

  openvdb::initialize();

  vdbfusion::VDBVolumeROS tsdf_volume(voxel_size, sdf_trunc, space_carving);

  ros::init(argc, argv, "vdbfusion_ros");
  ros::NodeHandle n;

  ros::Subscriber sub =
      n.subscribe("pointcloud", 1000,
                  &vdbfusion::VDBVolumeROS::integrateCallback, &tsdf_volume);

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).
   * ros::spin() will exit when Ctrl-C is pressed, or the node is shutdown by
   * the master.
   */
  ros::spin();

  return 0;
}