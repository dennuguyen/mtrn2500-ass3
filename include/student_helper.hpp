// Copyright 2019 Zhihao Zhang License MIT

#ifndef STUDENT_HELPER_HPP_
#define STUDENT_HELPER_HPP_

#include "rclcpp/rclcpp.hpp"

#include <string>

// constant expressions to represent the types of acceleration and velocity
#define LINEAR 1
#define ANGULAR !(LINEAR)
#define PRESSED 1
#define DROPPED 1
#define CLEARED 1

namespace helper
{
auto constexpr pi = 3.14159265358979323846;

// Magic
auto constexpr is_linear = 1;

// send_twiststamped is a constexpr to replace true/false for clarity
auto constexpr send_twiststamped = 1;

auto constexpr local_frame_name = [](std::string const & zid) {
    return "/" + zid + "/local_frame";
};

auto constexpr world_frame_name = [](std::string const & zid) {
    return "/" + zid + "/world_frame";
};

auto constexpr joy_node_name = [](std::string const & zid) {
    return zid + "_joy_node";
};

auto constexpr input_node_name = [](std::string const & zid) {
    return zid + "_input_node";
};

auto constexpr velocity_node_name = [](std::string const & zid) {
    return zid + "_velocity_node";
};

auto constexpr pose_node_name = [](std::string const & zid) {
    return zid + "_pose_node";
};

auto constexpr marker_node_name = [](std::string const & zid) {
    return zid + "_marker_node";
};

auto constexpr transform_node_name = [](std::string const & zid) {
    return zid + "_transform_node";
};

// Declaration of topic naming functions
auto constexpr joy_topic_name = [](std::string const & zid)
{
    return "/" + zid + "/joy";
};

auto constexpr acceleration_topic_name = [](std::string const & zid)
{
    return "/" + zid + "/acceleration";
};

auto constexpr velocity_topic_name = [](std::string const & zid)
{
    return "/" + zid + "/velocity";
};

auto constexpr pose_topic_name = [](std::string const & zid)
{
    return "/" + zid + "/pose";
};

/// get_time is a helper function that gets the change in time of the input
/// parameters
auto get_time(rclcpp::Time current_time, rclcpp::Time previous_time) -> double;

} // namespace helper
#endif // STUDENT_HELPER_HPP_
