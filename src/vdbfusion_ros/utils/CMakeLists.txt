add_library(type_conversions STATIC
  type_conversions.cpp
)
target_link_libraries(type_conversions
  ${catkin_LIBRARIES}
  Eigen3::Eigen
)