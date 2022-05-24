#include <geometry_msgs/Transform.h>
#include <tf/transform_listener.h>

#include <eigen3/Eigen/Core>

void InvertTransform(geometry_msgs::Transform& tf) {
    tf::Transform transform;
    tf::transformMsgToTF(tf, transform);
    tf::transformTFToMsg(transform.inverse(), tf);
}

void PreProcessCloud(std::vector<Eigen::Vector3d>& points, float min_range, float max_range) {
    points.erase(
        std::remove_if(points.begin(), points.end(), [&](auto p) { return p.norm() > max_range; }),
        points.end());
    points.erase(
        std::remove_if(points.begin(), points.end(), [&](auto p) { return p.norm() < min_range; }),
        points.end());
}