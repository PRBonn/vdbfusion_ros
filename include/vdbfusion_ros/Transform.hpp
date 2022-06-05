#pragma once

#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Core>
#include <deque>

using geometry_msgs::TransformStamped;

namespace vdbfusion {
class Transform {
public:
    explicit Transform(ros::NodeHandle& nh);

    bool lookUpTransform(const ros::Time& timestamp,
                         const ros::Duration& tolerance,
                         TransformStamped& transform);

private:
    bool lookUpTransformTF2(const std::string& parent_frame,
                            const std::string& child_frame,
                            const ros::Time& timestamp,
                            const ros::Duration& tolerance,
                            TransformStamped& transform);

    bool lookUpTransformQ(const ros::Time& timestamp,
                          const ros::Duration& tolerance,
                          TransformStamped& transform);

    void tfCallback(const TransformStamped& transform_msg);

private:
    bool use_tf2_;

    tf2_ros::Buffer buffer_;
    tf2_ros::TransformListener tf_;
    std::string parent_frame_;
    std::string child_frame_;

    ros::Subscriber tf_sub_;
    std::deque<TransformStamped, Eigen::aligned_allocator<TransformStamped>> tf_queue_;
};
}  // namespace vdbfusion
