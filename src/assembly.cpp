// Dan Nguyen (z5206032) &&
// Mon-1500

// The assembly consists of one of each shape
// assembly.cpp encapsulates the instantiation of shape classes and its display

#include "assembly.hpp"

#include "rclcpp/rclcpp.hpp" // http://docs.ros2.org/dashing/api/rclcpp/
#include "student_helper.hpp"
#include "shape_factory.hpp"

#include <chrono>
#include <memory>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace shapes
{
using namespace std::chrono_literals;

Assembly::Assembly(void)
    : shapes_list_ptr_{
        std::make_shared<std::vector<visualization_msgs::msg::Marker>>()}
{
    // Create the shape objects and display objects
    //
    // Primitives
    auto [cone, cone_display] = display::make_cone();
    auto [cube, cube_display] = display::make_cube();
    auto [cylinder, cylinder_display] = display::make_cylinder();
    auto [sphere, sphere_display] = display::make_sphere();
    // Pyramids
    auto [triangular_pyramid, triangular_pyramid_display] = display::make_triangular_pyramid();
    auto [square_pyramid, square_pyramid_display] = display::make_square_pyramid();
    auto [rectangular_pyramid, rectangular_pyramid_display] = display::make_rectangular_pyramid();
    auto [octagonal_pyramid, octagonal_pyramid_display] = display::make_octagonal_pyramid();
    // Prisms
    auto [triangular_prism, triangular_prism_display] = display::make_triangular_prism();
    auto [rectangular_prism, rectangular_prism_display] = display::make_rectangular_prism();
    auto [octagonal_prism, octagonal_prism_display] = display::make_octagonal_prism();
    // Others
    auto [flat_plane, flat_plane_display] = display::make_flat_plane();
    auto [parallelepiped, parallelepiped_display] = display::make_parallelepiped();

    // Assign shape members with the created object
    //
    // Primitives
    cone_ = cone;
    cube_ = cube;
    cylinder_ = cylinder;
    sphere_ = sphere;
    // Pyramids
    triangular_pyramid_ = triangular_pyramid;
    square_pyramid_ = square_pyramid;
    rectangular_pyramid_ = rectangular_pyramid;
    octagonal_pyramid_ = octagonal_pyramid;
    // Prisms
    triangular_prism_ = triangular_prism;
    rectangular_prism_ = rectangular_prism;
    octagonal_prism_ = octagonal_prism;
    // Others
    flat_plane_ = flat_plane;
    parallelepiped_ = parallelepiped;

    // Push display objects into a list
    //
    // This is done to allow the display object to be added as a node in main
    // to avoid an rclcpp linkage error
    //
    // This feature allows the display of individual shapes
    //
    // Primitives
    display_list_.push_back(cone_display);
    display_list_.push_back(cube_display);
    display_list_.push_back(cylinder_display);
    display_list_.push_back(sphere_display);
    // Pyramids
    display_list_.push_back(triangular_pyramid_display);
    display_list_.push_back(square_pyramid_display);
    display_list_.push_back(rectangular_pyramid_display);
    display_list_.push_back(octagonal_pyramid_display);
    // Prisms
    display_list_.push_back(triangular_prism_display);
    display_list_.push_back(rectangular_prism_display);
    display_list_.push_back(octagonal_prism_display);
    // Others
    display_list_.push_back(flat_plane_display);
    display_list_.push_back(parallelepiped_display);

    // Pushes shape objects into the shapes_list_ptr_
    //
    // This feature allows the display of all shapes with a single display object
    //
    // Primitives
    shapes_list_ptr_->push_back(cone_->get_display_markers()->at(0));
    shapes_list_ptr_->push_back(cube_->get_display_markers()->at(0));
    shapes_list_ptr_->push_back(cylinder_->get_display_markers()->at(0));
    shapes_list_ptr_->push_back(sphere_->get_display_markers()->at(0));
    // Pyramids
    shapes_list_ptr_->push_back(triangular_pyramid_->get_display_markers()->at(0));
    shapes_list_ptr_->push_back(square_pyramid_->get_display_markers()->at(0));
    shapes_list_ptr_->push_back(rectangular_pyramid_->get_display_markers()->at(0));
    shapes_list_ptr_->push_back(octagonal_pyramid_->get_display_markers()->at(0));
    // Prisms
    shapes_list_ptr_->push_back(triangular_prism_->get_display_markers()->at(0));
    shapes_list_ptr_->push_back(rectangular_prism_->get_display_markers()->at(0));
    shapes_list_ptr_->push_back(octagonal_prism_->get_display_markers()->at(0));
    // Others
    shapes_list_ptr_->push_back(flat_plane_->get_display_markers()->at(0));
    shapes_list_ptr_->push_back(parallelepiped_->get_display_markers()->at(0));
}

// Destructor
Assembly::~Assembly(void)
{
}

// Get display list
const std::vector<std::shared_ptr<display::SingleShapeDisplay>>
    & Assembly::get_display_list(void)
{
    return display_list_;
}

// Get shapes list pointer
auto Assembly::get_shapes_list_ptr(void) ->
    std::shared_ptr<std::vector<visualization_msgs::msg::Marker>>
{
    assert(shapes_list_ptr_);
    return shapes_list_ptr_;
}

// Getters
auto Assembly::get_cone(void) -> std::shared_ptr<shapes::Cone>
{
    return cone_;
}

auto Assembly::get_cube(void) -> std::shared_ptr<shapes::Cube>
{
    return cube_;
}

auto Assembly::get_cylinder(void) -> std::shared_ptr<shapes::Cylinder>
{
    return cylinder_;
}

auto Assembly::get_sphere(void) -> std::shared_ptr<shapes::Sphere>
{
    return sphere_;
}

auto Assembly::get_triangular_pyramid(void) -> std::shared_ptr<shapes::TriangleList>
{
    return triangular_pyramid_;
}

auto Assembly::get_square_pyramid(void) -> std::shared_ptr<shapes::TriangleList>
{
    return square_pyramid_;
}

auto Assembly::get_rectangular_pyramid(void) -> std::shared_ptr<shapes::TriangleList>
{
    return rectangular_pyramid_;
}

auto Assembly::get_octagonal_pyramid(void) -> std::shared_ptr<shapes::TriangleList>
{
    return octagonal_pyramid_;
}

auto Assembly::get_triangular_prism(void) -> std::shared_ptr<shapes::TriangleList>
{
    return triangular_prism_;
}

auto Assembly::get_rectangular_prism(void) -> std::shared_ptr<shapes::TriangleList>
{
    return rectangular_prism_;
}

auto Assembly::get_octagonal_prism(void) -> std::shared_ptr<shapes::TriangleList>
{
    return octagonal_prism_;
}

auto Assembly::get_flat_plane(void) -> std::shared_ptr<shapes::TriangleList>
{
    return flat_plane_;
}

auto Assembly::get_parallelepiped(void) -> std::shared_ptr<shapes::TriangleList>
{
    return parallelepiped_;
}
} // namespace shapes
