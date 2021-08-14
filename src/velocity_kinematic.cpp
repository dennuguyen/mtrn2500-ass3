// Copyright 2019 Zhihao Zhang License MIT

// MTRN2500 Assignment 2 - Monday 3pm
// Completed By Dan Nguyen (z5206032)

#include "config_parser.hpp"
#include "student_helper.hpp"
#include "velocity_kinematic.hpp"

#include <cassert>
#include <memory>
#include <string>
#include <utility>

/// Helper Function Prototype Declarations
static auto get_veloc(double u, double a, double t) -> double;
static auto friction(double input) -> double;
static auto print_accel_veloc(
    geometry_msgs::msg::AccelStamped::UniquePtr & acceleration_,
    geometry_msgs::msg::TwistStamped::UniquePtr & velocity_,
    double dt) -> void;
static auto handle_error(
    geometry_msgs::msg::AccelStamped::UniquePtr & acceleration_,
    geometry_msgs::msg::TwistStamped::UniquePtr & velocity_,
    assignment2::KinematicLimits config_) -> bool;
static auto check_age(
    geometry_msgs::msg::AccelStamped::UniquePtr & acceleration_,
    geometry_msgs::msg::TwistStamped::UniquePtr & velocity_,
    rclcpp::Time current_time) -> bool;
static auto check_max(double & kinematic, double & max) -> void;

namespace assignment2
{
VelocityKinematic::VelocityKinematic(std::string const & zid,
    std::chrono::milliseconds const refresh_period, KinematicLimits config)
    : rclcpp::Node{helper::velocity_node_name(zid)}
    , zid_{zid}
    , config_{config}
{
    // Initialise members
    this->acceleration_ = std::make_unique<geometry_msgs::msg::AccelStamped>();
    this->velocity_ = std::make_unique<geometry_msgs::msg::TwistStamped>();

    // Set the initial conditions of velocity_
    velocity_->twist.linear.x = 0.0;
    velocity_->twist.linear.y = 0.0;
    velocity_->twist.linear.z = 0.0;
    velocity_->twist.angular.z = 0.0;
    velocity_->header.stamp = rclcpp::Node::now();
    velocity_->header.frame_id = zid_;

    // Binding partial function to callback to pass "this" argument
    auto accel_callback = std::bind(&VelocityKinematic::acceleration_callback,
        this, std::placeholders::_1);
    auto veloc_callback = std::bind(&VelocityKinematic::velocity_callback,
        this);

    // Assign subscriber object to /acceleration
    this->acceleration_input_ = create_subscription<
        geometry_msgs::msg::AccelStamped>(helper::acceleration_topic_name(zid_),
        10, accel_callback);

    // Assign publisher object to /velocity
    this->velocity_output_ = create_publisher<geometry_msgs::msg::TwistStamped>(
        helper::velocity_topic_name(zid_), 10);

    // Create timer to periodically veloc_callback as constructor is created
    this->timer_ = create_wall_timer(refresh_period, veloc_callback);
}

// acceleration_callback does calculations every subscriber message
auto VelocityKinematic::acceleration_callback(
    geometry_msgs::msg::AccelStamped::UniquePtr input_message) -> void
{
    // Get current_time
    rclcpp::Time current_time = rclcpp::Node::now();

    // Get the change in time = current_time - previous_time
    double dt = helper::get_time(current_time, velocity_->header.stamp);

    // Change ownership of input_message to acceleration_
    acceleration_ = std::move(input_message);

    // Calculate the linear and angular velocity from acceleration message
    double x_veloc = get_veloc(velocity_->twist.linear.x,
        acceleration_->accel.linear.x, dt);
    double y_veloc = get_veloc(velocity_->twist.linear.y,
        acceleration_->accel.linear.y, dt);
    double z_veloc = get_veloc(velocity_->twist.linear.z,
        acceleration_->accel.linear.z, dt);
    double angular_veloc = get_veloc(velocity_->twist.angular.z,
        acceleration_->accel.angular.z, dt);

    // Frictitious forces slow the vehicle down over time
    x_veloc = friction(x_veloc);
    y_veloc = friction(y_veloc);
    z_veloc = friction(z_veloc);
    angular_veloc = friction(angular_veloc);
    
    // Reassign calculated values to velocity_
    velocity_->twist.linear.x = x_veloc;
    velocity_->twist.linear.y = y_veloc;
    velocity_->twist.linear.z = z_veloc;
    velocity_->twist.angular.z = angular_veloc;
    velocity_->header.stamp = current_time;
    velocity_->header.frame_id = zid_;

    // Place kinematic constraints and deal with lost communication
    send_twiststamped_ = handle_error(acceleration_, velocity_,
        VelocityKinematic::config_);

    // Print the status of the acceleration and velocities
    print_accel_veloc(acceleration_, velocity_, dt);
}

// velocity_callback publishes velocity_ and prints status every refresh_period
auto VelocityKinematic::velocity_callback(void) -> void
{
    rclcpp::Time current_time = rclcpp::Node::now();
    
    // Check age of velocity message and set acceleration_ and velocity_ values
    // to zero if messages older than 10s
    send_twiststamped_ = check_age(acceleration_, velocity_, current_time);

    // Only publish if true
    if (send_twiststamped_)
    {
        // Create the veloc_message
        // velocity_ is not published to ensure it stays within scope
        geometry_msgs::msg::TwistStamped veloc_message;
        veloc_message.twist.linear.x = velocity_->twist.linear.x;
        veloc_message.twist.linear.y = velocity_->twist.linear.y;
        veloc_message.twist.linear.z = velocity_->twist.linear.z;
        veloc_message.twist.angular.z = velocity_->twist.angular.z;
        veloc_message.header.stamp = velocity_->header.stamp;
        veloc_message.header.frame_id = velocity_->header.frame_id;
        
        // Publish to /velocity
        velocity_output_->publish(veloc_message);
    }
}
} // namespace assignment2

/// Helper function to get the linear and angular velocity
static auto get_veloc(double u, double a, double t) -> double
{
    // Use kinematic equation: v = u + a*t
    //      v = final velocity
    //      u = initial velocity
    //      a = acceleration
    //      t = time
    // 
    // The same equation can be analogously used for angular velocity

    return u + a * t;
}

/// Helper function that returns a decreased value of input unless 0
static auto friction(double input) -> double
{
    if (input > 0.02)
    {
        return input -= 0.02;
    }
    else if (input < -0.02)
    {
        return input += 0.02;
    }
    else
    {
        return 0.0;
    }
}

/// Helper function to print the status of acceleration and velocity values
static auto print_accel_veloc(
    geometry_msgs::msg::AccelStamped::UniquePtr & acceleration_,
    geometry_msgs::msg::TwistStamped::UniquePtr & velocity_,
    double dt) -> void
{
    std::cout << "\n";
    std::cout << "DT: " << dt;
    std::cout << "\n\n";
    std::cout << "Acceleration: \tX: " << acceleration_->accel.linear.x;
    std::cout << ", Y: " << acceleration_->accel.linear.y;
    std::cout << ", Z: " << acceleration_->accel.linear.z;
    std::cout << "\n";
    std::cout << "    Velocity: \tX: " << velocity_->twist.linear.x;
    std::cout << ", Y: " << velocity_->twist.linear.y;
    std::cout << ", Z: " << velocity_->twist.linear.z;
    std::cout << "\n\n";
    std::cout << "Angular Acceleration: " << acceleration_->accel.angular.z;
    std::cout << ", Angular Velocity " << velocity_->twist.angular.z;
    std::cout << "\n";
    std::cout << "\n==========================================================\n\n";
}

/// Helper function to handle errors in acceleration and velocity values
/// Calls another helper function: check_max
static auto handle_error(
    geometry_msgs::msg::AccelStamped::UniquePtr & acceleration_,
    geometry_msgs::msg::TwistStamped::UniquePtr & velocity_,
    assignment2::KinematicLimits config_) -> bool
{   
    check_max(velocity_->twist.linear.x, config_.max_linear_speed);
    check_max(velocity_->twist.linear.y, config_.max_linear_speed);
    check_max(velocity_->twist.linear.z, config_.max_linear_speed);
    check_max(velocity_->twist.angular.z, config_.max_angular_speed);
    check_max(acceleration_->accel.linear.x, config_.max_linear_acceleration);
    check_max(acceleration_->accel.linear.y, config_.max_linear_acceleration);
    check_max(acceleration_->accel.linear.z, config_.max_linear_acceleration);
    check_max(acceleration_->accel.angular.z, config_.max_angular_acceleration);
    
    return helper::send_twiststamped;
}

/// Helper function that checks the age of the most recent velocity_message
/// 
/// Velocity messages older than 10 seconds will trigger a commandline print,
/// reassignment of acceleration and velocity values to 0 and flag to no longer
/// publish velocity messages.
static auto check_age(
    geometry_msgs::msg::AccelStamped::UniquePtr & acceleration_,
    geometry_msgs::msg::TwistStamped::UniquePtr & velocity_,
    rclcpp::Time current_time) -> bool
{
    // Check if the age is greater than 10 seconds
    if (helper::get_time(current_time, velocity_->header.stamp) > 10)
    {
        std::cout << "Communication lost.\n";
        
        // Assign all values to 0
        velocity_->twist.linear.x = 0;
        velocity_->twist.linear.y = 0;
        velocity_->twist.linear.z = 0;
        velocity_->twist.angular.z = 0;
        acceleration_->accel.linear.x = 0;
        acceleration_->accel.linear.y = 0;
        acceleration_->accel.linear.z = 0;
        acceleration_->accel.angular.z = 0;

        // Do not publish velocity_message
        return !(helper::send_twiststamped);
    }
    else
    {
        // Publish velocity_message
        return helper::send_twiststamped;
    }
}

/// Helper function to check if the acceleration and velocity values have
/// reached their max limit. If so, then reassign these values to the limit.
static auto check_max(double & kinematic, double & max) -> void
{
    if (abs(kinematic) > abs(max))
    {
        if (kinematic >= 0) // Positive case
        {
            kinematic = max;
        }
        else // Negative case
        {
            kinematic = -max;
        }
    }
}
