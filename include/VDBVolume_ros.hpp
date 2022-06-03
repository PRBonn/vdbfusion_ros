#pragma once

#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Empty.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Core>
#include <string>

#include "Transform.hpp"
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
    Transform tf_;

private:
    VDBVolume vdb_volume_;

    bool preprocess_;
    bool apply_pose_;
    float min_range_;
    float max_range_;

    bool fill_holes_;
    float min_weight_;
};
}  // namespace vdbfusion