// Copyright 2019 Zhihao Zhang License MIT

// MTRN2500 Assignment 2 - Monday 3pm
// Completed By Dan Nguyen (z5206032)

#include "pose_kinematic.hpp"
#include "student_helper.hpp"

#include <cassert>
#include <cmath>
#include <memory>
#include <string>
#include <utility>

extern int x_cargo, y_cargo, z_cargo, o_cargo;

/// Helper Function Prototype Declarations
static auto resolve_velocity(
    geometry_msgs::msg::TwistStamped::UniquePtr & velocity_,
    double orientation) -> std::pair<double, double>;
static auto get_pose(double v, double dt, double s_0) -> double;
static auto handle_error(double & x, double & y, double & z) -> void;

namespace assignment2
{
PoseKinematic::PoseKinematic(
    std::string const & zid, std::chrono::milliseconds const refresh_period)
    : rclcpp::Node{helper::pose_node_name(zid)}
    , zid_{zid}
{
    // Initialise members
    this->velocity_ = std::make_unique<geometry_msgs::msg::TwistStamped>();
    this->pose_ = std::make_unique<geometry_msgs::msg::PoseStamped>();

    // Assume starting origin to be zero with zero orientation
    pose_->pose.position.x = 0.0;
    pose_->pose.position.y = 0.0;
    pose_->pose.position.z = 0.0;
    pose_->pose.orientation.z = 0.0;
    pose_->header.stamp = rclcpp::Node::now();
    pose_->header.frame_id = zid_;

    // Binding partial function to callback to pass "this" argument
    auto veloc_callback = std::bind(&PoseKinematic::velocity_callback, this,
        std::placeholders::_1);
    auto pose_callback = std::bind(&PoseKinematic::pose_callback, this);
    
    // Assign subscriber object to /velocity
    this->velocity_input_ = create_subscription<
        geometry_msgs::msg::TwistStamped>(helper::velocity_topic_name(zid_),
            10, veloc_callback);
    
    // Assign publisher object to /pose
    this->pose_output_ = create_publisher<geometry_msgs::msg::PoseStamped>(
        helper::pose_topic_name(zid_), 10);

    // Create timer to periodically pose_callback as constructor is created
    this->timer_ = create_wall_timer(refresh_period, pose_callback);
}

auto PoseKinematic::velocity_callback(
    geometry_msgs::msg::TwistStamped::UniquePtr input_message) -> void
{
    // Get current_time
    rclcpp::Time current_time = rclcpp::Node::now();

    // Get the time derivative = current_time - previous_time
    double dt = helper::get_time(current_time, pose_->header.stamp);

    // Change ownership of input_message to velocity_
    velocity_ = std::move(input_message);

    // Calculate the orientation first to get theta for vector resolution
    double z_orient = get_pose(velocity_->twist.angular.z, dt,
        pose_->pose.orientation.z);

    // Resolve the velocity vector into x and y components
    double veloc_x = resolve_velocity(velocity_, z_orient).first;
    double veloc_y = resolve_velocity(velocity_, z_orient).second;

    // Calculate the position from velocity message
    double x_pose = get_pose(velocity_->twist.linear.x, dt, pose_->pose.position.x);
    double y_pose = get_pose(velocity_->twist.linear.y, dt, pose_->pose.position.y);
    double z_pose = get_pose(velocity_->twist.linear.z, dt, pose_->pose.position.z);

    // Handle error in xyz position
    handle_error(x_pose, y_pose, z_pose);

    // Reassign calculated values to pose_
    pose_->pose.position.x = x_pose;
    pose_->pose.position.y = y_pose;
    pose_->pose.position.z = z_pose;
    pose_->pose.orientation.z = z_orient;
    pose_->header.stamp = current_time;
    pose_->header.frame_id = zid_;
}

auto PoseKinematic::pose_callback(void) -> void
{
    // Get the xyz position and orientation for cargo marker
    x_cargo = pose_->pose.position.x;
    y_cargo = pose_->pose.position.y;
    z_cargo = pose_->pose.position.z;
    o_cargo = pose_->pose.orientation.z;

    // Create the pose_message
    geometry_msgs::msg::PoseStamped pose_message;
    pose_message.pose.position.x = pose_->pose.position.x;
    pose_message.pose.position.y = pose_->pose.position.y;
    pose_message.pose.position.z = pose_->pose.position.z;
    pose_message.pose.orientation.z = pose_->pose.orientation.z;
    pose_message.header.stamp = pose_->header.stamp;
    pose_message.header.frame_id = pose_->header.frame_id;

    // Publish the pose_message
    pose_output_->publish(pose_message);
}
} // namespace assignment2


/// Helper function to resolve the velocity vector into x and y components
static auto resolve_velocity(
    geometry_msgs::msg::TwistStamped::UniquePtr & velocity_,
    double orientation) -> std::pair<double, double>
{
    // Vector trigonometry
    //      x = R*cos(theta)
    //      y = R*sin(theta)

    return std::make_pair(
        velocity_->twist.linear.x * cos(orientation),
        velocity_->twist.linear.x * sin(orientation));
}

/// Helper function to calculate the displacement
static auto get_pose(double v, double dt, double s_0) -> double
{
    // Use of kinematic equation: s = v*t + s_0
    //      s = final displacement
    //      v = velocity
    //      t = time
    //      s_0 = initial displacement
    // 
    // The same equation can be analogously used for orientation

    return v * dt + s_0;
}

// Handles the error of the UAV reaching out of bounds
static auto handle_error(double & x, double & y, double & z) -> void
{
    // Check x boundary
    if (x < -15.0)
    {
        std::cout << "Reached map boundary!\n";
        x = -15.0;
    }
    else if (x > 15.0)
    {
        std::cout << "Reached map boundary!\n";
        x = 15.0;
    }

    // Check x boundary
    if (y < -15.0)
    {
        std::cout << "Reached map boundary!\n";
        y = -15.0;
    }
    else if (y > 15.0)
    {
        std::cout << "Reached map boundary!\n";
        y = 15.0;
    }

    // Check z boundary
    if (z < 0.0) // Account for cargo
    {
        std::cout << "Reached map boundary!\n";
        z = 0.0;
    }
    else if (z > 15.0)
    {
        std::cout << "Reached map boundary!\n";
        z = 15.0;
    }
}
