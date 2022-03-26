#pragma once

#include "std_srvs/Empty.h"
#include "vdbfusion/VDBVolume.h"
#include "vdbfusion_ros/pointcloud_with_origin.h"
#include "vdbfusion_ros/save_volume.h"

namespace vdbfusion {
class VDBVolumeROS : public VDBVolume {
public:
    VDBVolumeROS(float voxel_size,
                 float sdf_trunc,
                 bool space_carving = false,
                 bool fill_holes = true,
                 float min_weight = 5.0);
    void integrate(const vdbfusion_ros::pointcloud_with_origin& pcl);
    bool saveVolume(vdbfusion_ros::save_volume::Request& request, vdbfusion_ros::save_volume::Response& response);

public:
    bool fill_holes_;
    float min_weight_;
};
}  // namespace vdbfusion