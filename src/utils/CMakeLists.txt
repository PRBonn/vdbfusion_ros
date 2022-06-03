add_library(type_conversions STATIC type_conversions.cpp)
target_link_libraries(type_conversions PUBLIC ${catkin_LIBRARIES} Eigen3::Eigen)
target_include_directories(type_conversions PUBLIC ${EIGEN3_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})