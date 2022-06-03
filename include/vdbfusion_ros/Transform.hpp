#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

#include <deque>
#include <eigen3/Eigen/Core>

namespace vdbfusion {
class Transform {
private:
    template <typename Type>
    using AlignedDeque = std::deque<Type, Eigen::aligned_allocator<Type>>;

public:
    explicit Transform(ros::NodeHandle& nh);

    bool lookUpTransform(const ros::Time& timestamp,
                         const ros::Duration& duration,
                         geometry_msgs::TransformStamped& transform);

    bool lookUpTransformTF2(const std::string& parent_frame,
                            const std::string& child_frame,
                            const ros::Time& timestamp,
                            const ros::Duration& duration,
                            geometry_msgs::TransformStamped& transform);

    bool lookUpTransformQ(const ros::Time& timestamp, geometry_msgs::TransformStamped& transform);

    void tfCallback(const geometry_msgs::TransformStamped& transform_msg);

private:
    tf2_ros::Buffer buffer_;
    tf2_ros::TransformListener tf_;
    bool use_tf2_;
    std::string parent_frame_;
    std::string child_frame_;
    ros::Subscriber tf_sub_;
    AlignedDeque<geometry_msgs::TransformStamped> tf_queue_;
};
}  // namespace vdbfusion