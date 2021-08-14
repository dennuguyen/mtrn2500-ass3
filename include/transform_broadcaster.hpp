// Copyright 2019 Zhihao Zhang License MIT

// MTRN2500 Assignment 2 - Monday 3pm
// Completed By Dan Nguyen (z5206032)

#ifndef TRANSFORM_BROADCASTER_HPP_
#define TRANSFORM_BROADCASTER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/transform_broadcaster.h"

#include <string>

namespace assignment2
{
class TransformBroadcaster : public rclcpp::Node
{
public:
    explicit TransformBroadcaster(std::string const & zid);

private:
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
        pose_source_;
    tf2_ros::TransformBroadcaster transform_output_;
    std::string zid_;

    auto incoming_callback(
        geometry_msgs::msg::PoseStamped::UniquePtr pose_message) -> void;
};
} // namespace assignment2
#endif // TRANSFORM_BROADCASTER_HPP_
