// Copyright 2019 Zhihao Zhang License MIT
#include "triangle_list.hpp"

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
TriangleList::TriangleList(int id, PointList point_list)
    : point_list_{point_list}
    , size_{1.0}
    , parent_frame_name_{"local_frame"}
    , shapes_list_ptr_{
        std::make_shared<std::vector<visualization_msgs::msg::Marker>>()}
{
    // Create references for use
    auto & shapes_list = *shapes_list_ptr_;
    shapes_list.emplace_back();
    auto & shape = shapes_list[0];

    // Create a TRIANGLE_LIST marker
    shape.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
    shape.action = visualization_msgs::msg::Marker::ADD;
    shape.header.frame_id = helper::world_frame_name("z0000000");
    shape.ns = "";
    shape.id = id;

    // Push vertices into shape.points
    std::shared_ptr<std::vector<geometry_msgs::msg::Point>> vertex_list =
        point_list_.get_points();

    assert(!vertex_list->empty());
    std::for_each(vertex_list->begin(), vertex_list->end(),
        [&shape](geometry_msgs::msg::Point pt) -> void
            {
                shape.points.push_back(pt);
            });

    // Set position
    shape.pose.position.x = 0;
    shape.pose.position.y = 0;
    shape.pose.position.z = 0;

    // Set orientation in quaternion
    shape.pose.orientation.x = 0;
    shape.pose.orientation.y = 0;
    shape.pose.orientation.z = 0;
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

// Implementation function to resize marker
auto TriangleList::resize_imple(AllAxis const new_size) -> void
{
    size_ = new_size;

    shapes_list_ptr_->at(0).scale.x = size_.get_value();
    shapes_list_ptr_->at(0).scale.y = size_.get_value();
    shapes_list_ptr_->at(0).scale.z = size_.get_value();
}

// Implementation function to rescale marker
auto TriangleList::rescale_imple(AnyAxis const factor) -> void
{
    size_ = AllAxis{size_.get_value() * factor.get_value()};

    shapes_list_ptr_->at(0).scale.x = size_.get_value();
    shapes_list_ptr_->at(0).scale.y = size_.get_value();
    shapes_list_ptr_->at(0).scale.z = size_.get_value();
}

// Implementation function to set the new colour of marker
auto TriangleList::set_colour_imple(Colour const new_colour) -> void
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
auto TriangleList::get_colour_imple(void) const -> Colour
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
auto TriangleList::set_parent_frame_name_imple(std::string frame_name) -> void
{
    parent_frame_name_ = std::move(frame_name);
    shapes_list_ptr_->at(0).header.frame_id = parent_frame_name_;
}

// Implementation function to get location of  marker
auto TriangleList::get_location_imple(void) const -> std::tuple<XAxis, YAxis, ZAxis>
{
    return std::tuple{
        XAxis{shapes_list_ptr_->at(0).pose.position.x},
        YAxis{shapes_list_ptr_->at(0).pose.position.y},
        ZAxis{shapes_list_ptr_->at(0).pose.position.z}};
}

// Implementation function to absolutely move the marker along XAxis
auto TriangleList::move_to_imple(XAxis const x) -> void
{
    shapes_list_ptr_->at(0).pose.position.x = x.get_value();
}

// Implementation function to absolutely move the sphere marker along YAxis
auto TriangleList::move_to_imple(YAxis const y) -> void
{
    shapes_list_ptr_->at(0).pose.position.y = y.get_value();
}

// Implementation function to absolutely move the sphere marker along ZAxis
auto TriangleList::move_to_imple(ZAxis const z) -> void
{
    shapes_list_ptr_->at(0).pose.position.z = z.get_value();
}

// Implementation function to absolutely move the sphere marker along XYZAxis
auto TriangleList::move_to_imple(XAxis const x, YAxis const y, ZAxis const z) -> void
{
    shapes_list_ptr_->at(0).pose.position.x = x.get_value();
    shapes_list_ptr_->at(0).pose.position.y = y.get_value();
    shapes_list_ptr_->at(0).pose.position.z = z.get_value();
}

// Implementation function to relatively move the sphere marker along XAxis
auto TriangleList::move_by_imple(XAxis const x) -> void
{
    shapes_list_ptr_->at(0).pose.position.x += x.get_value();
}

// Implementation function to relatively move the sphere marker along YAxis
auto TriangleList::move_by_imple(YAxis const y) -> void
{
    shapes_list_ptr_->at(0).pose.position.y += y.get_value();
}

// Implementation function to relatively move the sphere marker along ZAxis
auto TriangleList::move_by_imple(ZAxis const z) -> void
{
    shapes_list_ptr_->at(0).pose.position.z += z.get_value();
}

// Implementation function to relatively move the sphere marker along XYZAxis
auto TriangleList::move_by_imple(XAxis const x, YAxis const y, ZAxis const z) -> void
{
    shapes_list_ptr_->at(0).pose.position.x += x.get_value();
    shapes_list_ptr_->at(0).pose.position.y += y.get_value();
    shapes_list_ptr_->at(0).pose.position.z += z.get_value();
}

// Implementation function to returns the marker
auto TriangleList::get_display_markers_imple(void)
    -> std::shared_ptr<std::vector<visualization_msgs::msg::Marker>>
{
    return shapes_list_ptr_;
}

// Implementation function to rotate about the ZAxis
auto TriangleList::rotate_about_axis_to_imple(ZAxis radians) -> void
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
auto TriangleList::get_orientation_imple(void) const -> ZAxis
{
    return ZAxis{shapes_list_ptr_->at(0).pose.orientation.z};
}
} // namespace shapes
