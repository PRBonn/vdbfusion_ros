#include "Transform.hpp"

#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>

vdbfusion::Transform::Transform(ros::NodeHandle& nh) : buffer_(ros::Duration(50, 0)), tf_(buffer_) {
    ROS_INFO("Transform init");
    nh.getParam("/use_tf_transforms", use_tf2_);
    if (use_tf2_) {
        nh.getParam("/parent_frame", parent_frame_);
        nh.getParam("/child_frame", child_frame_);
    } else {
        std::string tf_topic;
        nh.getParam("/tf_topic", tf_topic);
        const int queue_size = 500;
        tf_sub_ = nh.subscribe(tf_topic, queue_size, &vdbfusion::Transform::tfCallback, this);
    }
}

void vdbfusion::Transform::tfCallback(const geometry_msgs::TransformStamped& transform_msg) {
    tf_queue_.push_back(transform_msg);
}

bool vdbfusion::Transform::lookUpTransform(const ros::Time& timestamp,
                                           const ros::Duration& duration,
                                           geometry_msgs::TransformStamped& transform) {
    if (use_tf2_) {
        return lookUpTransformTF2(parent_frame_, child_frame_, timestamp, duration, transform);
    } else {
        return lookUpTransformQ(timestamp, transform);
    }
}

bool vdbfusion::Transform::lookUpTransformTF2(const std::string& parent_frame,
                                              const std::string& child_frame,
                                              const ros::Time& timestamp,
                                              const ros::Duration& duration,
                                              geometry_msgs::TransformStamped& transform) {
    bool can_transform = buffer_.canTransform(parent_frame_, child_frame_, timestamp, duration);
    if (can_transform) {
        ROS_INFO("Transform available");
        transform = buffer_.lookupTransform(parent_frame_, child_frame_, timestamp, duration);
    }
    return can_transform;
}

bool vdbfusion::Transform::lookUpTransformQ(const ros::Time& timestamp,
                                            geometry_msgs::TransformStamped& transform) {
    if (tf_queue_.empty()) {
        ROS_WARN_STREAM_THROTTLE(30, "No match found for transform timestamp: "
                                         << timestamp << " as transform queue is empty.");
        return false;
    }
    const auto timestamp_tolerance_ns_ = 1e3;
    for (const auto& elem : tf_queue_) {
        if (elem.header.stamp > timestamp) {
            if ((elem.header.stamp - timestamp).toNSec() < timestamp_tolerance_ns_) {
                transform = elem;
                return true;
            }
            break;
        }

        if ((timestamp - elem.header.stamp).toNSec() < timestamp_tolerance_ns_) {
            transform = elem;
            return true;
        }
    }
    return false;
}
