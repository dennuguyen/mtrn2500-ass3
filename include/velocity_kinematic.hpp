// Copyright 2019 Zhihao Zhang License MIT

// MTRN2500 Assignment 2 - Monday 3pm
// Completed By Dan Nguyen (z5206032)

#ifndef VELOCITY_KINEMATIC_HPP_
#define VELOCITY_KINEMATIC_HPP_

#include "config_parser.hpp"
#include "geometry_msgs/msg/accel_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

#include <chrono>
#include <string>

namespace assignment2
{
class VelocityKinematic final : public rclcpp::Node
{
public:
    explicit VelocityKinematic(std::string const & zid,
        std::chrono::milliseconds refresh_period, KinematicLimits config);
    
    bool send_twiststamped_;    // Boolean to send TwistStamped message

private:
    rclcpp::Subscription<geometry_msgs::msg::AccelStamped>::SharedPtr
        acceleration_input_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr
        velocity_output_;
    geometry_msgs::msg::AccelStamped::UniquePtr acceleration_;
    geometry_msgs::msg::TwistStamped::UniquePtr velocity_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::string const zid_;
    KinematicLimits config_;

    auto acceleration_callback(
        geometry_msgs::msg::AccelStamped::UniquePtr input_message) -> void;
    auto velocity_callback(void) -> void;
};
} // namespace assignment2

#endif // VELOCITY_KINEMATIC_HPP_
