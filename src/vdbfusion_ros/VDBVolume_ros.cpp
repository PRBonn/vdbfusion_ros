#include "VDBVolume_ros.hpp"

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TransformStamped.h>
#include <happly.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <tf/transform_listener.h>
#include <tf2_msgs/TFMessage.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

#include <Eigen/Core>
#include <array>
#include <iostream>
#include <tuple>
#include <vector>

#include "openvdb/openvdb.h"

namespace {

typedef std::tuple<std::vector<openvdb::Vec3i>, std::vector<Eigen::Vector3d>> ColorsAndPoints;

ColorsAndPoints PointcloudToColorsAndPoints(const pcl::PointCloud<pcl::PointXYZRGB>& pcl) {
    std::vector<openvdb::Vec3i> colors;
    std::vector<Eigen::Vector3d> points;
    colors.reserve(pcl.size());
    points.reserve(pcl.size());
    std::for_each(pcl.points.begin(), pcl.points.end(), [&](const auto& point) {
        if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) {
            colors.emplace_back(openvdb::Vec3i(point.r, point.g, point.b));
            points.emplace_back(Eigen::Vector3d(point.x, point.y, point.z));
        }
    });
    return std::forward_as_tuple(std::move(colors), std::move(points));
}

void PreProcessCloud(pcl::PointCloud<pcl::PointXYZRGB>& points, float min_range, float max_range) {
    points.erase(std::remove_if(points.begin(), points.end(),
                                [&](auto p) { return p.getVector3fMap().norm() > max_range; }),
                 points.end());
    points.erase(std::remove_if(points.begin(), points.end(),
                                [&](auto p) { return p.getVector3fMap().norm() < min_range; }),
                 points.end());
}
}  // namespace

vdbfusion::VDBVolume vdbfusion::VDBVolumeNode::InitVDBVolume() {
    float voxel_size;
    float sdf_trunc;
    bool space_carving;

    nh_.getParam("/voxel_size", voxel_size);
    nh_.getParam("/sdf_trunc", sdf_trunc);
    nh_.getParam("/space_carving", space_carving);

    VDBVolume vdb_volume(voxel_size, sdf_trunc, space_carving);

    return vdb_volume;
}

vdbfusion::VDBVolumeNode::VDBVolumeNode() : vdb_volume_(InitVDBVolume()), tf_(nh_) {
    openvdb::initialize();

    std::string pcl_topic;
    nh_.getParam("/pcl_topic", pcl_topic);
    nh_.getParam("/preprocess", preprocess_);
    nh_.getParam("/apply_pose", apply_pose_);
    nh_.getParam("/min_range", min_range_);
    nh_.getParam("/max_range", max_range_);

    nh_.getParam("/fill_holes", fill_holes_);
    nh_.getParam("/min_weight", min_weight_);

    int32_t tol;
    nh_.getParam("/timestamp_tolerance_ns", tol);
    timestamp_tolerance_ = ros::Duration(0, tol);

    const int queue_size = 500;

    sub_ = nh_.subscribe(pcl_topic, queue_size, &vdbfusion::VDBVolumeNode::Integrate, this);
    srv_ = nh_.advertiseService("/save_vdb_volume", &vdbfusion::VDBVolumeNode::saveVDBVolume, this);

    ROS_INFO("Use '/save_vdb_volume' service to save the integrated volume");
}

void vdbfusion::VDBVolumeNode::Integrate(const sensor_msgs::PointCloud2& pcd) {
    geometry_msgs::TransformStamped transform;
    sensor_msgs::PointCloud2 pcd_ = pcd;

    if (tf_.lookUpTransform(pcd.header.stamp, timestamp_tolerance_, transform)) {
        ROS_INFO("Transform available");
        if (apply_pose_) {
            tf2::doTransform(pcd, pcd_, transform);
        }

        pcl::PointCloud<pcl::PointXYZRGB> point_cloud;
        pcl::moveFromROSMsg(pcd_, point_cloud);
        if (preprocess_) {
            PreProcessCloud(point_cloud, min_range_, max_range_);
        }

        auto [colors, points] = PointcloudToColorsAndPoints(point_cloud);

        const auto& x = transform.transform.translation.x;
        const auto& y = transform.transform.translation.y;
        const auto& z = transform.transform.translation.z;
        auto origin = Eigen::Vector3d(x, y, z);

        vdb_volume_.Integrate(points, colors, origin, [](float /*unused*/) { return 1.0; });
    }
}

bool vdbfusion::VDBVolumeNode::saveVDBVolume(vdbfusion_ros::save_vdb_volume::Request& path,
                                             vdbfusion_ros::save_vdb_volume::Response& response) {
    ROS_INFO("Saving the mesh and VDB grid files ...");
    std::string volume_name = path.path;
    openvdb::io::File(volume_name + "_grid.vdb").write({vdb_volume_.tsdf_});

    // Run marching cubes and save a .ply file
    auto [vertices, triangles, colors] =
        this->vdb_volume_.ExtractTriangleMesh(this->fill_holes_, this->min_weight_);

    // Suppose these hold your data
    std::vector<std::array<double, 3>> meshVertexPositions;
    for (size_t i = 0; i < vertices.size(); i++) {
        std::array<double, 3> vertex = {vertices[i][0], vertices[i][1], vertices[i][2]};
        meshVertexPositions.emplace_back(vertex);
    }

    std::vector<std::vector<size_t>> meshFaceIndices;
    for (size_t i = 0; i < triangles.size(); i++) {
        std::vector<size_t> triangle = {static_cast<size_t>(triangles[i][0]),
                                        static_cast<size_t>(triangles[i][1]),
                                        static_cast<size_t>(triangles[i][2])};
        meshFaceIndices.emplace_back(triangle);
    }

    std::vector<std::array<double, 3>> meshVertexColors;
    for (size_t i = 0; i < colors.size(); i++) {
        std::array<double, 3> triangle = {colors[i][0], colors[i][1], colors[i][2]};
        meshVertexColors.emplace_back(triangle);
    }

    // Create an empty object
    happly::PLYData plyOut;

    // Add mesh data (elements are created automatically)
    plyOut.addVertexPositions(meshVertexPositions);
    plyOut.addVertexColors(meshVertexColors);
    plyOut.addFaceIndices(meshFaceIndices);

    // Write the object to file
    std::string filename = volume_name + "_mesh.ply";
    plyOut.write(filename, happly::DataFormat::Binary);

    ROS_INFO("Done saving the mesh and VDB grid files in file %s", filename.c_str());
    return true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "vdbfusion_rosnode");
    vdbfusion::VDBVolumeNode vdb_volume_node;
    ros::spin();
}
