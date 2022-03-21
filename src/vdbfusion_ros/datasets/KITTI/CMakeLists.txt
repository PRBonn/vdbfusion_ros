add_executable(kitti_dataset_node KITTIOdometry.cpp)
target_link_libraries(kitti_dataset_node PUBLIC
    ${catkin_LIBRARIES}
    type_conversions
    VDBFusion::vdbfusion
    Eigen3::Eigen
    stdc++fs
)
target_include_directories(kitti_dataset_node PUBLIC
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${PROJECT_SOURCE_DIR}/utils
)
