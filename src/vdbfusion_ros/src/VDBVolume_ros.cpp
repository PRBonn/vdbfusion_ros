#include "VDBVolume_ros.hpp"

#include <geometry_msgs/TransformStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <ros/ros.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

#include <eigen3/Eigen/Core>
#include <fstream>
#include <iostream>
#include <vector>

#include "igl/write_triangle_mesh.h"
#include "openvdb/openvdb.h"
#include "type_conversions.hpp"

vdbfusion::VDBVolumeNode::VDBVolumeNode(float voxel_size,
                                        float sdf_trunc,
                                        bool space_carving /* = false*/,
                                        bool fill_holes /* = true*/,
                                        float min_weight /* = 5.0*/)
    : tf_(buffer_),
      vdb_volume_(voxel_size, sdf_trunc, space_carving),
      fill_holes_(fill_holes),
      min_weight_(min_weight) {}

void vdbfusion::VDBVolumeNode::Integrate(const sensor_msgs::PointCloud2& pcl2) {
    std::vector<Eigen::Vector3d> scan;
    geometry_msgs::TransformStamped transform;

    // transform_
    transform =
        this->buffer_.lookupTransform("world", "base_link", ros::Time(0), ros::Duration(1, 0));
    sensor_msgs::PointCloud2 pcl_transformed;
    tf2::doTransform(pcl2, pcl_transformed, transform);
    Eigen::Vector3d origin =
        Eigen::Vector3d(transform.transform.translation.x, transform.transform.translation.y,
                        transform.transform.translation.z);
    // typeconvert::tf2ToEigen(tf, origin);
    typeconvert::pcl2SensorMsgToEigen(pcl_transformed, scan);
    this->vdb_volume_.Integrate(scan, origin, [](float /*unused*/) { return 1.0; });
}

bool vdbfusion::VDBVolumeNode::saveVolume(vdbfusion_ros::save_volume::Request& save_path,
                                          vdbfusion_ros::save_volume::Response& response) {
    ROS_INFO("Saving the mesh and VDB grid files ...");

    std::string volume_name = save_path.path;
    {
        auto tsdf_grid = this->vdb_volume_.tsdf_;
        openvdb::io::File(volume_name + "_grid.vdb").write({tsdf_grid});
    }

    // Run marching cubes and save a .ply file
    {
        auto [vertices, triangles] =
            this->vdb_volume_.ExtractTriangleMesh(this->fill_holes_, this->min_weight_);

        // TODO: Fix this!
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
    openvdb::initialize();

    ros::init(argc, argv, "vdbfusion_rosnode");
    ros::NodeHandle nh;

    float voxel_size;
    float sdf_trunc;
    bool space_carving;
    bool fill_holes;
    float min_weight;

    nh.getParam("/voxel_size", voxel_size);
    nh.getParam("/sdf_trunc", sdf_trunc);
    nh.getParam("/space_carving", space_carving);
    nh.getParam("/fill_holes", fill_holes);
    nh.getParam("/min_weight", min_weight);

    vdbfusion::VDBVolumeNode vdb_volume(voxel_size, sdf_trunc, space_carving, fill_holes,
                                        min_weight);

    const int queue_size = 100;
    const std::string& pcl_topic = "/kitti/velo/pointcloud";
    const std::string& tf_topic = "/tf/transforms";

    // message_filters::Subscriber<sensor_msgs::PointCloud2> scan_sub(nh, pcl_topic, queue_size);
    // message_filters::Subscriber<tf2_msgs::TFMessage> tf_sub(nh, tf_topic, queue_size);
    // message_filters::TimeSynchronizer<sensor_msgs::PointCloud2, tf2_msgs::TFMessage> sync(
    //     scan_sub, tf_sub, queue_size);
    // sync.registerCallback(&vdbfusion::VDBVolumeNode::Integrate, &vdb_volume);
    // ros::Subscriber sub =
    //     nh.subscribe("/pointcloud", 1000, &vdbfusion::VDBVolumeNode::Integrate, &vdb_volume);

    ros::Subscriber sub =
        nh.subscribe(pcl_topic, queue_size, &vdbfusion::VDBVolumeNode::Integrate, &vdb_volume);
    ros::ServiceServer srv =
        nh.advertiseService("/save_volume", &vdbfusion::VDBVolumeNode::saveVolume, &vdb_volume);

    ros::spin();
    return 0;
}