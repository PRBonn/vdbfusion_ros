#include "VDBVolume_ros.hpp"

#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

#include <eigen3/Eigen/Core>
#include <fstream>
#include <iostream>
#include <vector>

#include "igl/write_triangle_mesh.h"
#include "openvdb/openvdb.h"
#include "type_conversions.hpp"

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

vdbfusion::VDBVolumeNode::VDBVolumeNode() : tf_(buffer_), vdb_volume_(InitVDBVolume()) {
    openvdb::initialize();

    std::string pcl_topic;
    nh_.getParam("/pcl_topic", pcl_topic);
    nh_.getParam("/fill_holes", fill_holes_);
    nh_.getParam("/min_weight", min_weight_);
    nh_.getParam("/parent_frame", parent_frame_);
    nh_.getParam("/child_frame", child_frame_);

    const int queue_size = 100;

    sub_ = nh_.subscribe(pcl_topic, queue_size, &vdbfusion::VDBVolumeNode::Integrate, this);
    srv_ = nh_.advertiseService("/save_volume", &vdbfusion::VDBVolumeNode::saveVolume, this);

    ROS_INFO("Initialized VDBVolumeNode");
    ROS_INFO("Use '/save_volume' ros service to save the integrated volume");
}

void vdbfusion::VDBVolumeNode::Integrate(const sensor_msgs::PointCloud2& pcl2) {
    std::vector<Eigen::Vector3d> scan;
    geometry_msgs::TransformStamped transform;

    auto block_time = ros::Duration(0, 1e8);
    if (buffer_.canTransform(parent_frame_, child_frame_, pcl2.header.stamp, block_time)) {
        ROS_INFO("Transform available");
        transform =
            buffer_.lookupTransform(parent_frame_, child_frame_, pcl2.header.stamp, block_time);
        sensor_msgs::PointCloud2 pcl_transformed;
        tf2::doTransform(pcl2, pcl_transformed, transform);
        Eigen::Vector3d origin =
            Eigen::Vector3d(transform.transform.translation.x, transform.transform.translation.y,
                            transform.transform.translation.z);

        typeconvert::pcl2SensorMsgToEigen(pcl_transformed, scan);
        vdb_volume_.Integrate(scan, origin, [](float /*unused*/) { return 1.0; });
    }
}

bool vdbfusion::VDBVolumeNode::saveVolume(vdbfusion_ros::save_volume::Request& save_path,
                                          vdbfusion_ros::save_volume::Response& response) {
    ROS_INFO("Saving the mesh and VDB grid files ...");

    std::string volume_name = save_path.path;

    openvdb::io::File(volume_name + "_grid.vdb").write({vdb_volume_.tsdf_});

    // Run marching cubes and save a .ply file
    {
        auto [vertices, triangles] =
            this->vdb_volume_.ExtractTriangleMesh(this->fill_holes_, this->min_weight_);

        Eigen::MatrixXd V(vertices.size(), 3);
        for (size_t i = 0; i < vertices.size(); i++) {
            V.row(i) = Eigen::VectorXd::Map(&vertices[i][0], vertices[i].size());
        }

        // TODO: Also this
        Eigen::MatrixXi F(triangles.size(), 3);
        for (size_t i = 0; i < triangles.size(); i++) {
            F.row(i) = Eigen::VectorXi::Map(&triangles[i][0], triangles[i].size());
        }
        igl::write_triangle_mesh(volume_name + "_mesh.ply", V, F, igl::FileEncoding::Binary);
    }
    ROS_INFO("Done saving the mesh and VDB grid files");
    return true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "vdbfusion_rosnode");

    vdbfusion::VDBVolumeNode vdb_volume_node;

    ros::spin();
}