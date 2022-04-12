#pragma once

#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Empty.h>
#include <tf2_ros/transform_listener.h>

#include <string>

#include "vdbfusion/VDBVolume.h"
#include "vdbfusion_ros/save_volume.h"

namespace vdbfusion {
class VDBVolumeNode {
public:
    VDBVolumeNode();

private:
    void Integrate(const sensor_msgs::PointCloud2& pcl2);
    VDBVolume InitVDBVolume();
    bool saveVolume(vdbfusion_ros::save_volume::Request& save_path,
                    vdbfusion_ros::save_volume::Response& response);

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::ServiceServer srv_;
    tf2_ros::Buffer buffer_;
    tf2_ros::TransformListener tf_;

private:
    VDBVolume vdb_volume_;
    bool fill_holes_;
    float min_weight_;
    std::string parent_frame_;
    std::string child_frame_;
};
}  // namespace vdbfusion