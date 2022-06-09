#pragma once

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <Eigen/Core>
#include <vector>

#include "Transform.hpp"
#include "vdbfusion/VDBVolume.h"
#include "vdbfusion_ros/save_vdb_volume.h"

namespace {
void pcl2SensorMsgToEigen(const sensor_msgs::PointCloud2& pcl2,
                          std::vector<Eigen::Vector3d>& points);
void PreProcessCloud(std::vector<Eigen::Vector3d>& points, float min_range, float max_range);
}  // namespace

namespace vdbfusion {
class VDBVolumeNode {
public:
    VDBVolumeNode();

private:
    VDBVolume InitVDBVolume();
    void Integrate(const sensor_msgs::PointCloud2& pcd);
    bool saveVDBVolume(vdbfusion_ros::save_vdb_volume::Request& path,
                       vdbfusion_ros::save_vdb_volume::Response& response);

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::ServiceServer srv_;
    Transform tf_;
    ros::Duration timestamp_tolerance_;

private:
    VDBVolume vdb_volume_;

    // PointCloud Processing
    bool preprocess_;
    bool apply_pose_;
    float min_range_;
    float max_range_;

    // Triangle Mesh Extraction
    bool fill_holes_;
    float min_weight_;
};
}  // namespace vdbfusion