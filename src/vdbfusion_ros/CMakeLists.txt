add_library(transforms STATIC src/Transform.cpp)
add_executable(${PROJECT_NAME}_node src/VDBVolume_ros.cpp)

target_link_libraries(${PROJECT_NAME}_node PUBLIC
  ${catkin_LIBRARIES}
  type_conversions
  VDBFusion::vdbfusion
  Eigen3::Eigen
  igl::core
  transforms
)

target_include_directories(${PROJECT_NAME}_node PUBLIC
  ${PROJECT_SOURCE_DIR}/utils
  ${PROJECT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)
