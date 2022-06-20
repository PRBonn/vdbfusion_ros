#include "VDBVolume_ros.hpp"

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <tf/transform_listener.h>
#include <tf2_msgs/TFMessage.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

#include <Eigen/Core>
#include <iostream>
#include <vector>

#include "igl/write_triangle_mesh.h"
#include "openvdb/openvdb.h"

namespace {

template <typename PCLPoint>
inline bool isPointFinite(const PCLPoint& point) {
  return std::isfinite(point.x) && std::isfinite(point.y) &&
         std::isfinite(point.z);
}

std::vector<Eigen::Vector3d> pcl2SensorMsgToEigen(const sensor_msgs::PointCloud2& pcl2) {
    std::vector<Eigen::Vector3d> points;
    points.reserve(pcl2.width);
    ROS_INFO("** Total of %ld message points.", pcl2.width * pcl2.height);
    sensor_msgs::PointCloud pcl;
    sensor_msgs::convertPointCloud2ToPointCloud(pcl2, pcl);
    std::for_each(pcl.points.begin(), pcl.points.end(), [&](const auto& point) {
        if(isPointFinite(point)) {
            
        }
    });
    return points;
}

void pclRrbToVed3i(pcl::PointCloud<pcl::PointXYZRGB>& pcl, std::vector<openvdb::Vec3i>& colors, 
    std::vector<Eigen::Vector3d>& points) {
    colors.reserve(pcl.size());
    points.reserve(pcl.size());
    ROS_INFO("** Total of %ld RGBD points.", pcl.size());
    std::for_each(pcl.points.begin(), pcl.points.end(), [&](const auto& point) {
        if(isPointFinite(point)) {
            colors.emplace_back(openvdb::Vec3i(point.r, point.g, point.b));
            points.emplace_back(Eigen::Vector3d(point.x, point.y, point.z));
        }
    });
}


/*
template <>
inline Color convertColor(const pcl::PointXYZRGB& point) {
  return Color(point.r, point.g, point.b, point.a);
}


template <typename PCLPoint>
std::vector<openvdb::Vec3i> convertPointcloud(
    const typename pcl::PointCloud<PCLPoint>& pointcloud_pcl) {
  std::vector<openvdb::Vec3i> colors;
  colors.reserve(pcl.size());
  for (size_t i = 0; i < pointcloud_pcl.points.size(); ++i) {
    //if (!isPointFinite(pointcloud_pcl.points[i])) {
    //  continue;
    //}
    colors->emplace_back(
        convertColor<PCLPoint>(pointcloud_pcl.points[i]));
  }
}
*/

void PreProcessCloud(pcl::PointCloud<pcl::PointXYZRGB>& points, 
    float min_range, float max_range) {
    points.erase(
        std::remove_if(points.begin(), points.end(), [&](auto p) { return p.getVector3fMap().norm() > max_range; }),
        points.end());
    points.erase(
        std::remove_if(points.begin(), points.end(), [&](auto p) { return p.getVector3fMap().norm() < min_range; }),
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
        //auto scan = pcl2SensorMsgToEigen(pcd_);
        
        pcl::PointCloud<pcl::PointXYZRGB> point_cloud;
        //sensor_msgs::PointCloud2 no_cte_point_cloud_msg = pcd_;
        pcl::moveFromROSMsg(pcd_, point_cloud);        
        if (preprocess_) {
            PreProcessCloud(point_cloud, min_range_, max_range_);            
        }

        std::vector<openvdb::Vec3i> colors;
        std::vector<Eigen::Vector3d> scan;
        pclRrbToVed3i(point_cloud, colors, scan);

        const auto& x = transform.transform.translation.x;
        const auto& y = transform.transform.translation.y;
        const auto& z = transform.transform.translation.z;
        auto origin = Eigen::Vector3d(x, y, z);

        //ROS_INFO("Total of %ld , %ld points.", scan.size(), colors.size());
        vdb_volume_.Integrate(scan, colors, origin, [](float /*unused*/) { return 1.0; });        
    }
}

bool vdbfusion::VDBVolumeNode::saveVDBVolume(vdbfusion_ros::save_vdb_volume::Request& path,
                                             vdbfusion_ros::save_vdb_volume::Response& response) {
    ROS_INFO("Saving the mesh and VDB grid files ...");
    std::string volume_name = path.path;
    openvdb::io::File(volume_name + "_grid.vdb").write({vdb_volume_.tsdf_});

    // Run marching cubes and save a .ply file
    auto [vertices, triangles] =
        this->vdb_volume_.ExtractTriangleMesh(this->fill_holes_, this->min_weight_);

    Eigen::MatrixXd V(vertices.size(), 3);
    for (size_t i = 0; i < vertices.size(); i++) {
        V.row(i) = Eigen::VectorXd::Map(&vertices[i][0], vertices[i].size());
    }

    Eigen::MatrixXi F(triangles.size(), 3);
    for (size_t i = 0; i < triangles.size(); i++) {
        F.row(i) = Eigen::VectorXi::Map(&triangles[i][0], triangles[i].size());
    }
    igl::write_triangle_mesh(volume_name + "_mesh.ply", V, F, igl::FileEncoding::Binary);
    ROS_INFO("Done saving the mesh and VDB grid files");
    return true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "vdbfusion_rosnode");
    vdbfusion::VDBVolumeNode vdb_volume_node;
    ros::spin();
}
