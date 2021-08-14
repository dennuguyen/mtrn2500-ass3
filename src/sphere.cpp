// Copyright 2019 Zhihao Zhang License MIT
#include "sphere.hpp"

#include "rclcpp/rclcpp.hpp" // http://docs.ros2.org/dashing/api/rclcpp/
#include "student_helper.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/impl/convert.h"
#include "tf2_ros/transform_broadcaster.h"

#include <chrono>
#include <memory>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace shapes
{
Sphere::Sphere(int id)
    : size_{1.0}
    , parent_frame_name_{"local_frame"}
    , shapes_list_ptr_{
        std::make_shared<std::vector<visualization_msgs::msg::Marker>>()}
{
    // Create references for use
    auto & shapes_list = *shapes_list_ptr_;
    shapes_list.emplace_back();
    auto & shape = shapes_list[0];

    // Create a SPHERE marker
    shape.type = visualization_msgs::msg::Marker::SPHERE;
    shape.action = visualization_msgs::msg::Marker::ADD;
    shape.header.frame_id = helper::world_frame_name("z0000000");
    shape.ns = "";
    shape.id = id;

    // Set position
    shape.pose.position.x = 0.0;
    shape.pose.position.y = 0.0;
    shape.pose.position.z = 0.0;

    // Set orientation in quaternion
    shape.pose.orientation.x = 0;
    shape.pose.orientation.y = 0;
    shape.pose.orientation.z = 1;
    shape.pose.orientation.w = 1;

    // Set scale
    shape.scale.x = 1.0;
    shape.scale.y = 1.0;
    shape.scale.z = 1.0;

    // Set RGB & A (transparency)
    shape.color.r = 1.0;
    shape.color.g = 0.0;
    shape.color.b = 0.0;
    shape.color.a = 1.0;

    using namespace std::chrono_literals;
    shape.lifetime = rclcpp::Duration{1s};
}

// Destructor
Sphere::~Sphere(void)
{
    // delete shapes_list_ptr_;
}

// Set transparency because I ran out of time
auto Sphere::set_transparency(double new_transparency) -> void
{
    shapes_list_ptr_->at(0).color.a = new_transparency;
}

// Implementation function to resize marker
auto Sphere::resize_imple(AllAxis const new_size) -> void
{
    size_ = new_size;

    shapes_list_ptr_->at(0).scale.x = size_.get_value();
    shapes_list_ptr_->at(0).scale.y = size_.get_value();
    shapes_list_ptr_->at(0).scale.z = size_.get_value();
}

// Implementation function to rescale marker
auto Sphere::rescale_imple(AnyAxis const factor) -> void
{
    size_ = AllAxis{size_.get_value() * factor.get_value()};

    shapes_list_ptr_->at(0).scale.x = size_.get_value();
    shapes_list_ptr_->at(0).scale.y = size_.get_value();
    shapes_list_ptr_->at(0).scale.z = size_.get_value();
}

// Implementation function to set the new colour of marker
auto Sphere::set_colour_imple(Colour const new_colour) -> void
{
    switch (new_colour)
    {
        case Colour::red:
            shapes_list_ptr_->at(0).color.r = 1.0;
            shapes_list_ptr_->at(0).color.g = 0.0;
            shapes_list_ptr_->at(0).color.b = 0.0;
            shapes_list_ptr_->at(0).color.a = 1.0;
            break;

        case Colour::yellow:
            shapes_list_ptr_->at(0).color.r = 1.0;
            shapes_list_ptr_->at(0).color.g = 1.0;
            shapes_list_ptr_->at(0).color.b = 0.0;
            shapes_list_ptr_->at(0).color.a = 1.0;
            break;

        case Colour::green:
            shapes_list_ptr_->at(0).color.r = 0.0;
            shapes_list_ptr_->at(0).color.g = 1.0;
            shapes_list_ptr_->at(0).color.b = 0.0;
            shapes_list_ptr_->at(0).color.a = 1.0;
            break;

        case Colour::blue:
            shapes_list_ptr_->at(0).color.r = 0.0;
            shapes_list_ptr_->at(0).color.g = 0.0;
            shapes_list_ptr_->at(0).color.b = 1.0;
            shapes_list_ptr_->at(0).color.a = 1.0;
            break;

        case Colour::black:
            shapes_list_ptr_->at(0).color.r = 0.0;
            shapes_list_ptr_->at(0).color.g = 0.0;
            shapes_list_ptr_->at(0).color.b = 0.0;
            shapes_list_ptr_->at(0).color.a = 1.0;
            break;

        case Colour::white:
            shapes_list_ptr_->at(0).color.r = 1.0;
            shapes_list_ptr_->at(0).color.g = 1.0;
            shapes_list_ptr_->at(0).color.b = 1.0;
            shapes_list_ptr_->at(0).color.a = 1.0;
            break;
    }
}

// Implementation function to get the current colour of marker
auto Sphere::get_colour_imple(void) const -> Colour
{
    double R = shapes_list_ptr_->at(0).color.r;
    double G = shapes_list_ptr_->at(0).color.g;
    double B = shapes_list_ptr_->at(0).color.b;

    if (R == 1.0 && G == 0.0 && B == 0.0)
    {
        return Colour::red;
    }
    else if (R == 1.0 && G == 1.0 && B == 0.0)
    {
        return Colour::yellow;
    }
    else if (R == 0.0 && G == 1.0 && B == 0.0)
    {
        return Colour::green;
    }
    else if (R == 0.0 && G == 0.0 && B == 1.0)
    {
        return Colour::blue;
    }
    else if (R == 1.0 && G == 1.0 && B == 1.0)
    {
        return Colour::black;
    }
    else // (R == 0.0 && G == 0.0 && B == 0.0)
    {
        return Colour::white;
    }
}

// Implementation function to set parent frame name of marker
auto Sphere::set_parent_frame_name_imple(std::string frame_name) -> void
{
    parent_frame_name_ = std::move(frame_name);
    shapes_list_ptr_->at(0).header.frame_id = parent_frame_name_;
}

// Implementation function to get location of  marker
auto Sphere::get_location_imple() const -> std::tuple<XAxis, YAxis, ZAxis>
{
    return std::tuple{
        XAxis{shapes_list_ptr_->at(0).pose.position.x},
        YAxis{shapes_list_ptr_->at(0).pose.position.y},
        ZAxis{shapes_list_ptr_->at(0).pose.position.z}};
}

// Implementation function to absolutely move the marker along XAxis
auto Sphere::move_to_imple(XAxis const x) -> void
{
    shapes_list_ptr_->at(0).pose.position.x = x.get_value();
}

// Implementation function to absolutely move the sphere marker along YAxis
auto Sphere::move_to_imple(YAxis const y) -> void
{
    shapes_list_ptr_->at(0).pose.position.y = y.get_value();
}

// Implementation function to absolutely move the sphere marker along ZAxis
auto Sphere::move_to_imple(ZAxis const z) -> void
{
    shapes_list_ptr_->at(0).pose.position.z = z.get_value();
}

// Implementation function to absolutely move the sphere marker along XYZAxis
auto Sphere::move_to_imple(XAxis const x, YAxis const y, ZAxis const z) -> void
{
    shapes_list_ptr_->at(0).pose.position.x = x.get_value();
    shapes_list_ptr_->at(0).pose.position.y = y.get_value();
    shapes_list_ptr_->at(0).pose.position.z = z.get_value();
}

// Implementation function to relatively move the sphere marker along XAxis
auto Sphere::move_by_imple(XAxis const x) -> void
{
    shapes_list_ptr_->at(0).pose.position.x += x.get_value();
}

// Implementation function to relatively move the sphere marker along YAxis
auto Sphere::move_by_imple(YAxis const y) -> void
{
    shapes_list_ptr_->at(0).pose.position.y += y.get_value();
}

// Implementation function to relatively move the sphere marker along ZAxis
auto Sphere::move_by_imple(ZAxis const z) -> void
{
    shapes_list_ptr_->at(0).pose.position.z += z.get_value();
}

// Implementation function to relatively move the sphere marker along XYZAxis
auto Sphere::move_by_imple(XAxis const x, YAxis const y, ZAxis const z) -> void
{
    shapes_list_ptr_->at(0).pose.position.x += x.get_value();
    shapes_list_ptr_->at(0).pose.position.y += y.get_value();
    shapes_list_ptr_->at(0).pose.position.z += z.get_value();
}

// Implementation function to returns the marker
auto Sphere::get_display_markers_imple()
    -> std::shared_ptr<std::vector<visualization_msgs::msg::Marker>>
{
    return shapes_list_ptr_;
}

// Implementation function to rotate about the ZAxis
auto Sphere::rotate_about_axis_to_imple(ZAxis radians) -> void
{
    // Get the rotation as a quaternion
    auto const rotation = tf2::Quaternion(tf2::Vector3(0, 0, 1), radians.get_value());

    // Get the current heading as a quaternion
    tf2::Quaternion heading;
    tf2::fromMsg(shapes_list_ptr_->at(0).pose.orientation, heading);

    // Get the new heading
    shapes_list_ptr_->at(0).pose.orientation = tf2::toMsg(rotation * heading);
}

// Implementation function to get the orientation about the ZAxis
auto Sphere::get_orientation_imple() const -> ZAxis
{
    return ZAxis{shapes_list_ptr_->at(0).pose.orientation.z};
}
} // namespace shapes
