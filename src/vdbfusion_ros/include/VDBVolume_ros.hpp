#pragma once

#include "vdbfusion/VDBVolume.h"
#include "vdbfusion_ros/pointcloud_with_origin.h"
#include "std_srvs/Empty.h"

namespace vdbfusion {
class VDBVolumeROS : public VDBVolume {
public:
    VDBVolumeROS(float voxel_size, float sdf_trunc, 
        bool space_carving = false);
    void integrate(const vdbfusion_ros::pointcloud_with_origin &pcl);
    bool saveVolume(std_srvs::Empty::Request& request, 
        std_srvs::Empty::Response& response);
};
}