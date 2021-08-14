// Copyright 2019 Zhihao Zhang License MIT

// MTRN2500 Assignment 2 - Monday 3pm
// Completed By Dan Nguyen (z5206032)

#ifndef POSE_KINEMATIC_HPP_
#define POSE_KINEMATIC_HPP_

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

#include <chrono>
#include <string>

namespace assignment2
{
class PoseKinematic final : public rclcpp::Node
{
public:
    explicit PoseKinematic(
        std::string const & zid, std::chrono::milliseconds refresh_period);
    
    bool send_posestamped_;    // Boolean to send TwistStamped message

private:
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr
        velocity_input_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_output_;
    geometry_msgs::msg::TwistStamped::UniquePtr velocity_;
    geometry_msgs::msg::PoseStamped::UniquePtr pose_;

    rclcpp::TimerBase::SharedPtr timer_;
    std::string const zid_;

    auto velocity_callback(
        geometry_msgs::msg::TwistStamped::UniquePtr input_message) -> void;
    auto pose_callback(void) -> void;
};

} // namespace assignment2
#endif // POSE_KINEMATIC_HPP_
