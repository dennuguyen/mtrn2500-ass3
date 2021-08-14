// Copyright 2019 Zhihao Zhang License MIT

// MTRN2500 Assignment 2 - Monday 3pm
// Completed By Dan Nguyen (z5206032)

#include "config_parser.hpp"
#include "joystick_listener.hpp"
#include "student_helper.hpp"

#include "cargo.hpp"

// #include "controls.hpp"

#include <memory>
#include <string>
#include <utility>

extern int is_dropped;
extern int is_cleared;

/// Helper Function Prototype Declarations
static auto print_axes(sensor_msgs::msg::Joy::UniquePtr & joy_message) -> void;
static auto print_buttons(sensor_msgs::msg::Joy::UniquePtr & joy_message)
    -> void;
static auto correct_deadzone(double axis_input, double deadzone) -> double;
static auto get_accel(double axis_input, bool is_linear_) -> double;
static auto convert_input(double axis_input, double old_min, double old_max, 
    double new_min, double new_max) -> double;

namespace assignment2
{
    // Implement the JoystickListener constructor
    // Initialise the input_node and subscriber and publisher objects
    JoystickListener::JoystickListener(std::string const & zid,
        JoystickConfig config)

        : rclcpp::Node{helper::joy_node_name(zid)}
        , zid_{zid}
        , config_{config}
    {   
        // Binding partial function to callback to pass "this" argument
        auto joy_callback = std::bind(&JoystickListener::joy_message_callback,
            this, std::placeholders::_1);

        // Assign subscriber object to /joy
        this->joystick_input_ = create_subscription<sensor_msgs::msg::Joy>(
            helper::joy_topic_name(zid_), 10, joy_callback);
        
        // Assign publisher object to /acceleration
        this->acceleration_output_ = create_publisher<
            geometry_msgs::msg::AccelStamped>(
            helper::acceleration_topic_name(zid_), 10);
    }

    // Implement the JoystickListener joy_message_callback method
    //
    // Axes configurations
    // 
    // joy_message->axes[0] = XAxis
    // joy_message->axes[1] = YAxis
    // joy_message->axes[2] = ZAxis Down
    // joy_message->axes[5] = ZAxis Up
    // joy_message->axes[3] = Yaw

    // Button configurations
    //
    // joy_message->buttons[2] = Drop shape
    // joy_message->buttons[5] = Clear dropped shapes
    //
    auto JoystickListener::joy_message_callback(sensor_msgs::msg::Joy::UniquePtr
        joy_message) -> void
    {
        // Correct the axis input for deadzone
        joy_message->axes[0] = correct_deadzone(joy_message->axes[0],
            config_.speed_deadzone);
        joy_message->axes[1] = correct_deadzone(joy_message->axes[1],
            config_.speed_deadzone);
        joy_message->axes[2] = correct_deadzone(joy_message->axes[2], 0.49);
        joy_message->axes[5] = correct_deadzone(joy_message->axes[5], 0.49);
        joy_message->axes[3] = correct_deadzone(joy_message->axes[3],
            config_.steering_deadzone);

        // Print the axis and buttons status to std::cout
        print_axes(joy_message);
        print_buttons(joy_message);

        // If buttons have been pressed
        if (joy_message->buttons[2] == PRESSED)
        {
            std::cout << "\t\t\t\t\t\t\t\t\t\tDropping cargo!\n";
            is_dropped = DROPPED;
        }
        else if (joy_message->buttons[5] == PRESSED)
        {
            std::cout << "\t\t\t\t\t\t\t\t\t\tClearing dropped cargo!\n";
            is_cleared = CLEARED;
        }
        
        // Calculate the linear and angular acceleration from axes as axis_input
        double x_accel = get_accel(joy_message->axes[0], ANGULAR);
        double y_accel = get_accel(joy_message->axes[1], ANGULAR);
        double z_plus_accel = get_accel(joy_message->axes[5], LINEAR);
        double z_minus_accel = get_accel(joy_message->axes[2], LINEAR);
        double angular_accel = get_accel(joy_message->axes[3], ANGULAR);

        // Net acceleration is the sum of positive and negative acceleration
        double z_accel = z_plus_accel - z_minus_accel;
        
        // Create the accel_message
        geometry_msgs::msg::AccelStamped accel_message;
        accel_message.accel.linear.x = x_accel;
        accel_message.accel.linear.y = y_accel;
        accel_message.accel.linear.z = z_accel;
        accel_message.accel.angular.z = angular_accel;
        accel_message.header.stamp = joy_message->header.stamp;
        accel_message.header.frame_id = zid_;

        // Publish the accel_package to /acceleration
        acceleration_output_->publish(accel_message);
    }
} // namespace assignment2


/// Helper function to print the status of the joystick axes
static auto print_axes(sensor_msgs::msg::Joy::UniquePtr & joy_message) -> void
{
    std::for_each(joy_message->axes.begin(), joy_message->axes.end(),
        [](double i) -> void {std::cout << i << "\t";});
}

/// Helper function to pring the status of the number of buttons pressed
static auto print_buttons(sensor_msgs::msg::Joy::UniquePtr &
    joy_message) -> void
{
    std::cout << "\nTotal number of buttons pressed is "
        << std::count(joy_message->buttons.begin(),
            joy_message->buttons.end(), 1)
        << ".\n";
    std::cout << "\n==========================================================\n";
}

/// Helper function to correct for the deadzone of axis_input
static auto correct_deadzone(double axis_input, double deadzone) -> double
{
    // If axis_input is within deadzone region then axis_input is zero
    if (-deadzone <= axis_input && axis_input <= deadzone)
    {
        return 0.0;
    }
    else if (axis_input > deadzone) // if axis_input is positive
    {
        // Account for deadzone of axis_input
        // [deadzone, 1] -> [0, 1]
        return convert_input(axis_input, deadzone, 1.0, 0.0, 1.0);
    }
    else // if axis_input is negative
    {
        // [-deadzone, -1] -> [0, -1]
        return convert_input(axis_input, -deadzone, -1.0, 0.0, -1.0);
    }
}

/// Helper function to calculate linear and angular acceleration from config_
static auto get_accel(double axis_input, bool is_linear_) -> double
{
    if (is_linear_ && axis_input >= 0.0)
    {
        // Get positive accel from axis_input
        // [-1, 1] -> [0, 1]
        return convert_input(axis_input, -1.0, 1.0, 0.0, 1.0);
    }
    else if (is_linear_ && axis_input < 0.0)
    {
        // Get negative accel from axis_input
        // [-1, 1] -> [0, -1]
        return -convert_input(axis_input, -1.0, 1.0, 0.0, -1.0);
    }
    else
    {
        // Angular acceleration maps one-to-one
        // [-1, 1] -> [-1, 1]
        return axis_input;
    }
}

/// Helper function to do a linear map between two ranges
/// NewValue = ((OldValue - OldMin) * (NewMax - NewMin))
///     / (OldMax - OldMin) + NewMin
static auto convert_input(double axis_input, double old_min, double old_max, 
    double new_min, double new_max) -> double
{
    return (axis_input-old_min)*(new_max-new_min) / (old_max-old_min) + new_min;
}
