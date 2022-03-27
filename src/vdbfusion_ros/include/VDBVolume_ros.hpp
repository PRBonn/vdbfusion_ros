#pragma once

#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Empty.h>
#include <tf2_ros/transform_listener.h>

#include "vdbfusion/VDBVolume.h"
#include "vdbfusion_ros/save_volume.h"

namespace vdbfusion {
class VDBVolumeNode {
public:
    VDBVolumeNode(float voxel_size,
                  float sdf_trunc,
                  bool space_carving = false,
                  bool fill_holes = true,
                  float min_weight = 5.0);
    void Integrate(const sensor_msgs::PointCloud2& pcl2);
    bool saveVolume(vdbfusion_ros::save_volume::Request& save_path,
                    vdbfusion_ros::save_volume::Response& response);

private:
    tf2_ros::Buffer buffer_;
    tf2_ros::TransformListener tf_;
    VDBVolume vdb_volume_;
    const bool fill_holes_;
    const float min_weight_;
};
}  // namespace vdbfusion