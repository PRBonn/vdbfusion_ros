#include <fstream>
#include <iostream>
#include <vector>
#include <eigen3/Eigen/Core>

#include "igl/write_triangle_mesh.h"
#include "ros/ros.h"
#include "openvdb/openvdb.h"
#include "VDBVolume_ros.hpp"
#include "type_conversions.hpp"

vdbfusion::VDBVolumeROS::VDBVolumeROS(float voxel_size, float sdf_trunc, 
  bool space_carving /* = false*/)
    : VDBVolume(voxel_size, sdf_trunc, space_carving) {}

void vdbfusion::VDBVolumeROS::integrate(
    const vdbfusion_ros::pointcloud_with_origin &pcl
  ) {
  std::vector<Eigen::Vector3d> points;
  Eigen::Vector3d origin = Eigen::Vector3d(0, 0, 0);

  tf::originPointMsgToEigen(pcl.origin, origin);
  tf::pclSensorMsgToEigen(pcl.pcl, points);
  this->Integrate(points, origin, [](float /*unused*/) { return 1.0; });
}

bool vdbfusion::VDBVolumeROS::saveVolume(std_srvs::Empty::Request& request,
    std_srvs::Empty::Response& response){
  // Store the grid results to disks
  std::string map_name = "kitti_seq_00";
  {
    ROS_INFO_ONCE("Writing VDB grid to disk");
    auto tsdf_grid = this->tsdf_;
    std::string filename = map_name + ".vdb";
    openvdb::io::File(filename).write({tsdf_grid});
  }

  // Run marching cubes and save a .ply file
  {   
    bool fill_holes = true;
    float min_weight = 5.0;

    auto [vertices, triangles] = 
      this->ExtractTriangleMesh(fill_holes, min_weight);

    // TODO: Fix this!
    Eigen::MatrixXd V(vertices.size(), 3);
    for (size_t i = 0; i < vertices.size(); i++) {
        V.row(i) = Eigen::VectorXd::Map(&vertices[i][0], vertices[i].size());
    }

    // TODO: Also this
    Eigen::MatrixXi F(triangles.size(), 3);
    for (size_t i = 0; i < triangles.size(); i++) {
        F.row(i) = 
          Eigen::VectorXi::Map(&triangles[i][0], triangles[i].size());
    }
    std::string filename = map_name + ".ply";
    igl::write_triangle_mesh(filename, V, F, igl::FileEncoding::Binary);
  }
  return true;
}

int main(int argc, char **argv) {
  float voxel_size = 0.1;
  float sdf_trunc = 0.3;
  bool space_carving = false;

  openvdb::initialize();

  vdbfusion::VDBVolumeROS tsdf_volume(voxel_size, sdf_trunc, space_carving);

  ros::init(argc, argv, "vdbfusion_ros");
  ros::NodeHandle n;

  ros::Subscriber sub =
      n.subscribe("pointcloud", 1000, &vdbfusion::VDBVolumeROS::integrate,
        &tsdf_volume);

  ros::ServiceServer srv = n.advertiseService("/save_volume",
    &vdbfusion::VDBVolumeROS::saveVolume, &tsdf_volume);

  ros::spin();

  if (!ros::ok()){
    std::cout << "Exiting..";
  }
  return 0;
}