// Dan Nguyen (z5206032) &&
// Mon-1500

// The UAV is an uav of parts
// uav.cpp encapsulates the instantiation of shape classes and its display

#include "uav.hpp"

#include "rclcpp/rclcpp.hpp" // http://docs.ros2.org/dashing/api/rclcpp/
#include "student_helper.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/impl/convert.h"
#include "tf2_ros/transform_broadcaster.h"
#include "shape_factory.hpp"

#include <chrono>
#include <memory>
#include <string>
#include <tuple>
#include <utility>
#include <vector>
/*
// Helper Function
static auto account_rotation(shapes::ZAxis radians, shapes::XAxis cx, 
    shapes::YAxis cy, shapes::XAxis x, shapes::YAxis y) -> pair<double, double>;
*/

namespace shapes
{
using namespace std::chrono_literals;

UAV::UAV(void)
    : size_{1.0}
    , parent_frame_name_{"local_frame"}
    , display_list_{Assembly::get_display_list()}
    , shapes_list_ptr_{Assembly::get_shapes_list_ptr()}
{
    assert(shapes_list_ptr_);

    // Set the body for reference
    body_ = Assembly::get_flat_plane();

    // Spawn the UAV at origin
    move_to(XAxis{0.0}, YAxis{0.0}, ZAxis{0.0});

    // Set the appropriate dimensions
    Assembly::get_flat_plane()->rescale(AnyAxis{0.1});

    // Set the frame name for the UAV
    set_parent_frame_name(helper::local_frame_name("z0000000"));

    // Make it white
    set_colour(ColourInterface::Colour::white);

    Assembly::get_cube()->move_to(XAxis{0.0}, YAxis{0.0}, ZAxis{3.6});
    Assembly::get_cube()->rescale(AnyAxis{0.5});
    Assembly::get_cube()->set_colour(ColourInterface::Colour::green);

    Assembly::get_octagonal_prism()->move_to(XAxis{0.0}, YAxis{0.0}, ZAxis{2.1});
    Assembly::get_octagonal_prism()->set_colour(ColourInterface::Colour::blue);
    //Assembly::get_octagonal_prism()->rescale(AnyAxis{1.5});

    Assembly::get_cone()->move_to(XAxis{0.0}, YAxis{0.0}, ZAxis{3.7});
    Assembly::get_cone()->set_colour(ColourInterface::Colour::green);
    Assembly::get_cone()->rescale(AnyAxis{0.5});

    Assembly::get_parallelepiped()->move_to(XAxis{0.0}, YAxis{0.0}, ZAxis{3});
    Assembly::get_parallelepiped()->set_colour(ColourInterface::Colour::green);
    Assembly::get_parallelepiped()->rescale(AnyAxis{0.5});
    
    Assembly::get_rectangular_prism()->move_to(XAxis{0.0}, YAxis{0.0}, ZAxis{2.9});
    Assembly::get_rectangular_prism()->set_colour(ColourInterface::Colour::black);
    Assembly::get_rectangular_prism()->rescale(AnyAxis{0.5});


    Assembly::get_square_pyramid()->move_to(XAxis{0.0}, YAxis{-0.7}, ZAxis{2.9});
    Assembly::get_square_pyramid()->set_colour(ColourInterface::Colour::yellow);
    Assembly::get_square_pyramid()->rescale(AnyAxis{0.25});

    Assembly::get_rectangular_pyramid()->move_to(XAxis{0.0}, YAxis{0.7}, ZAxis{2.9});
    Assembly::get_rectangular_pyramid()->set_colour(ColourInterface::Colour::yellow);
    Assembly::get_rectangular_pyramid()->rescale(AnyAxis{0.25});

    Assembly::get_triangular_prism()->move_to(XAxis{0.0}, YAxis{0.0}, ZAxis{1.0});
    Assembly::get_triangular_prism()->set_colour(ColourInterface::Colour::white);

    Assembly::get_sphere()->move_to(XAxis{0.0}, YAxis{0.0}, ZAxis{4.8});
    Assembly::get_sphere()->set_colour(ColourInterface::Colour::white);
    Assembly::get_sphere()->rescale(AnyAxis{0.25});

    Assembly::get_cylinder()->move_to(XAxis{0.0}, YAxis{0.0}, ZAxis{5.0});
    Assembly::get_cylinder()->set_colour(ColourInterface::Colour::black);
    Assembly::get_cylinder()->rescale(AnyAxis{0.25});
}

// Destructor
UAV::~UAV(void)
{
}

// Get display list from the Assembly
const std::vector<std::shared_ptr<display::SingleShapeDisplay>>
    & UAV::get_display_list(void)
{
    return display_list_;
}

// Get the shapes list pointer from the Assembly
auto UAV::get_shapes_list_ptr(void) ->
    std::shared_ptr<std::vector<visualization_msgs::msg::Marker>>
{
    return shapes_list_ptr_;
}

// Update uav state
auto UAV::tick(double dt) -> void
{
    double dtheta = dt * helper::pi / 2 * 
        (std::get<2>(body_->get_location())).get_value();
    rotor_->rotate_about_axis_to(ZAxis{dtheta});
}

// Implementation function to resize marker
auto UAV::resize_imple(AllAxis const new_size) -> void
{
    size_ = new_size;
/*
    for (const auto & part : shapes_list_ptr_)
    {
        part->resize(new_size);
    }
*/

    // Account for distance between parts
    // DO THIS===========================================
}

// Implementation function to rescale marker
auto UAV::rescale_imple(AnyAxis const factor) -> void
{
    body_->rescale(factor);
    rotor_->rescale(factor);

    // Account for distance between parts
    // DO THIS===========================================
}

// Implementation function to set the new colour of marker
auto UAV::set_colour_imple(Colour const new_colour) -> void
{
    Assembly::get_cone()->set_colour(new_colour);
    Assembly::get_cube()->set_colour(new_colour);
    Assembly::get_cylinder()->set_colour(new_colour);
    Assembly::get_sphere()->set_colour(new_colour);
    Assembly::get_triangular_pyramid()->set_colour(new_colour);
    Assembly::get_square_pyramid()->set_colour(new_colour);
    Assembly::get_rectangular_pyramid()->set_colour(new_colour);
    Assembly::get_octagonal_pyramid()->set_colour(new_colour);
    Assembly::get_triangular_prism()->set_colour(new_colour);
    Assembly::get_rectangular_prism()->set_colour(new_colour);
    Assembly::get_octagonal_prism()->set_colour(new_colour);
    Assembly::get_flat_plane()->set_colour(new_colour);
    Assembly::get_parallelepiped()->set_colour(new_colour);
}

// Implementation function to get the current colour of marker
auto UAV::get_colour_imple(void) const -> Colour
{
    return body_->get_colour();
}

// Implementation function to set parent frame name of marker
auto UAV::set_parent_frame_name_imple(std::string frame_name) -> void
{
    parent_frame_name_ = std::move(frame_name);
    Assembly::get_cone()->set_parent_frame_name(parent_frame_name_);
    Assembly::get_cube()->set_parent_frame_name(parent_frame_name_);
    Assembly::get_cylinder()->set_parent_frame_name(parent_frame_name_);
    Assembly::get_sphere()->set_parent_frame_name(parent_frame_name_);
    Assembly::get_triangular_pyramid()->set_parent_frame_name(parent_frame_name_);
    Assembly::get_square_pyramid()->set_parent_frame_name(parent_frame_name_);
    Assembly::get_rectangular_pyramid()->set_parent_frame_name(parent_frame_name_);
    Assembly::get_octagonal_pyramid()->set_parent_frame_name(parent_frame_name_);
    Assembly::get_triangular_prism()->set_parent_frame_name(parent_frame_name_);
    Assembly::get_rectangular_prism()->set_parent_frame_name(parent_frame_name_);
    Assembly::get_octagonal_prism()->set_parent_frame_name(parent_frame_name_);
    Assembly::get_flat_plane()->set_parent_frame_name(parent_frame_name_);
    Assembly::get_parallelepiped()->set_parent_frame_name(parent_frame_name_);
}

// Implementation function to get location of  marker
auto UAV::get_location_imple(void) const -> std::tuple<XAxis, YAxis, ZAxis>
{
    return body_->get_location();
}

// Implementation function to absolutely move the marker along XAxis
auto UAV::move_to_imple(XAxis const x) -> void
{
    Assembly::get_cone()->move_to(x);
    Assembly::get_cube()->move_to(x);
    Assembly::get_cylinder()->move_to(x);
    Assembly::get_sphere()->move_to(x);
    Assembly::get_triangular_pyramid()->move_to(x);
    Assembly::get_square_pyramid()->move_to(x);
    Assembly::get_rectangular_pyramid()->move_to(x);
    Assembly::get_octagonal_pyramid()->move_to(x);
    Assembly::get_triangular_prism()->move_to(x);
    Assembly::get_rectangular_prism()->move_to(x);
    Assembly::get_octagonal_prism()->move_to(x);
    Assembly::get_flat_plane()->move_to(x);
    Assembly::get_parallelepiped()->move_to(x);
}

// Implementation function to absolutely move the sphere marker along YAxis
auto UAV::move_to_imple(YAxis const y) -> void
{
    Assembly::get_cone()->move_to(y);
    Assembly::get_cube()->move_to(y);
    Assembly::get_cylinder()->move_to(y);
    Assembly::get_sphere()->move_to(y);
    Assembly::get_triangular_pyramid()->move_to(y);
    Assembly::get_square_pyramid()->move_to(y);
    Assembly::get_rectangular_pyramid()->move_to(y);
    Assembly::get_octagonal_pyramid()->move_to(y);
    Assembly::get_triangular_prism()->move_to(y);
    Assembly::get_rectangular_prism()->move_to(y);
    Assembly::get_octagonal_prism()->move_to(y);
    Assembly::get_flat_plane()->move_to(y);
    Assembly::get_parallelepiped()->move_to(y);
}

// Implementation function to absolutely move the sphere marker along ZAxis
auto UAV::move_to_imple(ZAxis const z) -> void
{
    Assembly::get_cone()->move_to(z);
    Assembly::get_cube()->move_to(z);
    Assembly::get_cylinder()->move_to(z);
    Assembly::get_sphere()->move_to(z);
    Assembly::get_triangular_pyramid()->move_to(z);
    Assembly::get_square_pyramid()->move_to(z);
    Assembly::get_rectangular_pyramid()->move_to(z);
    Assembly::get_octagonal_pyramid()->move_to(z);
    Assembly::get_triangular_prism()->move_to(z);
    Assembly::get_rectangular_prism()->move_to(z);
    Assembly::get_octagonal_prism()->move_to(z);
    Assembly::get_flat_plane()->move_to(z);
    Assembly::get_parallelepiped()->move_to(z);
}

// Implementation function to absolutely move the sphere marker along XYZAxis
auto UAV::move_to_imple(XAxis const x, YAxis const y, ZAxis const z) -> void
{
    Assembly::get_cone()->move_to(x, y, z);
    Assembly::get_cube()->move_to(x, y, z);
    Assembly::get_cylinder()->move_to(x, y, z);
    Assembly::get_sphere()->move_to(x, y, z);
    Assembly::get_triangular_pyramid()->move_to(x, y, z);
    Assembly::get_square_pyramid()->move_to(x, y, z);
    Assembly::get_rectangular_pyramid()->move_to(x, y, z);
    Assembly::get_octagonal_pyramid()->move_to(x, y, z);
    Assembly::get_triangular_prism()->move_to(x, y, z);
    Assembly::get_rectangular_prism()->move_to(x, y, z);
    Assembly::get_octagonal_prism()->move_to(x, y, z);
    Assembly::get_flat_plane()->move_to(x, y, z);
    Assembly::get_parallelepiped()->move_to(x, y, z);
}

// Implementation function to relatively move the sphere marker along XAxis
auto UAV::move_by_imple(XAxis const x) -> void
{
    Assembly::get_cone()->move_by(x);
    Assembly::get_cube()->move_by(x);
    Assembly::get_cylinder()->move_by(x);
    Assembly::get_sphere()->move_by(x);
    Assembly::get_triangular_pyramid()->move_by(x);
    Assembly::get_square_pyramid()->move_by(x);
    Assembly::get_rectangular_pyramid()->move_by(x);
    Assembly::get_octagonal_pyramid()->move_by(x);
    Assembly::get_triangular_prism()->move_by(x);
    Assembly::get_rectangular_prism()->move_by(x);
    Assembly::get_octagonal_prism()->move_by(x);
    Assembly::get_flat_plane()->move_by(x);
    Assembly::get_parallelepiped()->move_by(x);
}

// Implementation function to relatively move the sphere marker along YAxis
auto UAV::move_by_imple(YAxis const y) -> void
{
    Assembly::get_cone()->move_by(y);
    Assembly::get_cube()->move_by(y);
    Assembly::get_cylinder()->move_by(y);
    Assembly::get_sphere()->move_by(y);
    Assembly::get_triangular_pyramid()->move_by(y);
    Assembly::get_square_pyramid()->move_by(y);
    Assembly::get_rectangular_pyramid()->move_by(y);
    Assembly::get_octagonal_pyramid()->move_by(y);
    Assembly::get_triangular_prism()->move_by(y);
    Assembly::get_rectangular_prism()->move_by(y);
    Assembly::get_octagonal_prism()->move_by(y);
    Assembly::get_flat_plane()->move_by(y);
    Assembly::get_parallelepiped()->move_by(y);
}

// Implementation function to relatively move the sphere marker along ZAxis
auto UAV::move_by_imple(ZAxis const z) -> void
{
    Assembly::get_cone()->move_by(z);
    Assembly::get_cube()->move_by(z);
    Assembly::get_cylinder()->move_by(z);
    Assembly::get_sphere()->move_by(z);
    Assembly::get_triangular_pyramid()->move_by(z);
    Assembly::get_square_pyramid()->move_by(z);
    Assembly::get_rectangular_pyramid()->move_by(z);
    Assembly::get_octagonal_pyramid()->move_by(z);
    Assembly::get_triangular_prism()->move_by(z);
    Assembly::get_rectangular_prism()->move_by(z);
    Assembly::get_octagonal_prism()->move_by(z);
    Assembly::get_flat_plane()->move_by(z);
    Assembly::get_parallelepiped()->move_by(z);
}

// Implementation function to relatively move the sphere marker along XYZAxis
auto UAV::move_by_imple(XAxis const x, YAxis const y, ZAxis const z) -> void
{
    Assembly::get_cone()->move_by(x, y, z);
    Assembly::get_cube()->move_by(x, y, z);
    Assembly::get_cylinder()->move_by(x, y, z);
    Assembly::get_sphere()->move_by(x, y, z);
    Assembly::get_triangular_pyramid()->move_by(x, y, z);
    Assembly::get_square_pyramid()->move_by(x, y, z);
    Assembly::get_rectangular_pyramid()->move_by(x, y, z);
    Assembly::get_octagonal_pyramid()->move_by(x, y, z);
    Assembly::get_triangular_prism()->move_by(x, y, z);
    Assembly::get_rectangular_prism()->move_by(x, y, z);
    Assembly::get_octagonal_prism()->move_by(x, y, z);
    Assembly::get_flat_plane()->move_by(x, y, z);
    Assembly::get_parallelepiped()->move_by(x, y, z);
}

// Implementation function to returns the marker
auto UAV::get_display_markers_imple(void)
    -> std::shared_ptr<std::vector<visualization_msgs::msg::Marker>>
{
    return body_->get_display_markers();
}

// Implementation function to rotate about the ZAxis
auto UAV::rotate_about_axis_to_imple(ZAxis radians) -> void
{
    Assembly::get_cone()->rotate_about_axis_to(radians);
    Assembly::get_cube()->rotate_about_axis_to(radians);
    Assembly::get_cylinder()->rotate_about_axis_to(radians);
    Assembly::get_sphere()->rotate_about_axis_to(radians);
    Assembly::get_triangular_pyramid()->rotate_about_axis_to(radians);
    Assembly::get_square_pyramid()->rotate_about_axis_to(radians);
    Assembly::get_rectangular_pyramid()->rotate_about_axis_to(radians);
    Assembly::get_octagonal_pyramid()->rotate_about_axis_to(radians);
    Assembly::get_triangular_prism()->rotate_about_axis_to(radians);
    Assembly::get_rectangular_prism()->rotate_about_axis_to(radians);
    Assembly::get_octagonal_prism()->rotate_about_axis_to(radians);
    Assembly::get_flat_plane()->rotate_about_axis_to(radians);
    Assembly::get_parallelepiped()->rotate_about_axis_to(radians);
}

// Implementation function to get the orientation about the ZAxis
auto UAV::get_orientation_imple(void) const -> ZAxis
{
    return ZAxis{body_->get_orientation()};
}
} // namespace shapes

/*
// cx, cy - center of square coordinates
// x, y - coordinates of a corner point of the square
// theta is the angle of rotation
static auto account_rotation(shapes::ZAxis radians, shapes::XAxis cx, 
    shapes::YAxis cy, shapes::XAxis x, shapes::YAxis y) -> pair<double, double>
{
    // translate point to origin
    double tempX = x.get_value() - cx.get_value();
    double tempY = y.get_value() - cy.get_value();

    // now apply rotation
    double rotatedX = tempX*cos(radians.get_value()) - tempY*sin(radians.get_value());
    double rotatedY = tempX*sin(radians).get_value()) + tempY*cos(radians.get_value());

    // translate back
    x = rotatedX + cx;
    y = rotatedY + cy;

    return { x, y };
}
*/
