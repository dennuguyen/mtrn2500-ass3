// Copyright 2019 Zhihao Zhang License MIT

// MTRN2500 Assignment 2 - Monday 3pm
// Completed By Dan Nguyen (z5206032)

#include "student_helper.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "transform_broadcaster.hpp"

#include <memory>
#include <string>
#include <utility>

namespace assignment2
{
TransformBroadcaster::TransformBroadcaster(std::string const & zid)
    : rclcpp::Node{helper::transform_node_name(zid)}
    , pose_source_{create_subscription<geometry_msgs::msg::PoseStamped>(
          "/" + zid + "/pose", 10,
          [this](geometry_msgs::msg::PoseStamped::UniquePtr input_message)
              -> void { incoming_callback(std::move(input_message)); })}
    , transform_output_{*this}
    , zid_{zid}
{
}

auto TransformBroadcaster::incoming_callback(
    geometry_msgs::msg::PoseStamped::UniquePtr pose_message) -> void
{
    if (pose_message)
    {
        auto transform_message =
            std::make_unique<geometry_msgs::msg::TransformStamped>();
        transform_message->header.frame_id = helper::world_frame_name(zid_);
        transform_message->header.stamp = pose_message->header.stamp;
        transform_message->child_frame_id = helper::local_frame_name(zid_);

        transform_message->transform.translation.x =
            pose_message->pose.position.x;
        transform_message->transform.translation.y =
            pose_message->pose.position.y;
        transform_message->transform.translation.z =
            pose_message->pose.position.z;

        auto const new_heading = pose_message->pose.orientation.z;
        auto const new_heading_quaternion =
            tf2::Quaternion{tf2::Vector3{0, 0, 1}, new_heading};
        transform_message->transform.rotation =
            tf2::toMsg(new_heading_quaternion);
        transform_output_.sendTransform(*transform_message);
    }
}
} // namespace assignment2
