// shape_factory.cpp deals with the instantiation of shape objects and its
// display objects and returns them as a pair

#include "shape_factory.hpp"

#include <string>

using namespace std::chrono_literals;

// Global variable to keep track of instantiated shapes
extern int id_global;

namespace display
{
auto make_cone(void) -> pair<shared_ptr<shapes::Cone>,
    shared_ptr<display::SingleShapeDisplay>>
{
    // Create and display a cone
    auto cone = std::make_shared<shapes::Cone>(id_global);
    auto cone_display =
        std::make_shared<display::SingleShapeDisplay>("cone" + std::to_string(id_global++), 10ms);
    cone_display->display_object(cone);

    return { cone, cone_display };
}

auto make_cube(void) -> pair<shared_ptr<shapes::Cube>,
    shared_ptr<display::SingleShapeDisplay>>
{
    // Create and display a cube
    auto cube = std::make_shared<shapes::Cube>(id_global);
    auto cube_display =
        std::make_shared<display::SingleShapeDisplay>("cube" + std::to_string(id_global++), 10ms);
    cube_display->display_object(cube);

    return { cube, cube_display };
}

auto make_cylinder(void) -> pair<shared_ptr<shapes::Cylinder>,
    shared_ptr<display::SingleShapeDisplay>>
{
    // Create and display a cylinder
    auto cylinder = std::make_shared<shapes::Cylinder>(id_global);
    auto cylinder_display =
        std::make_shared<display::SingleShapeDisplay>("cylinder" + std::to_string(id_global++), 10ms);
    cylinder_display->display_object(cylinder);

    return { cylinder, cylinder_display };
}

auto make_sphere(void) -> pair<shared_ptr<shapes::Sphere>,
    shared_ptr<display::SingleShapeDisplay>>
{
    // Create and display a sphere
    auto sphere = std::make_shared<shapes::Sphere>(id_global);
    auto sphere_display =
        std::make_shared<display::SingleShapeDisplay>("sphere" + std::to_string(id_global++), 10ms);
    sphere_display->display_object(sphere);

    return { sphere, sphere_display };
}

auto make_triangular_pyramid(void) -> pair<shared_ptr<shapes::TriangleList>,
    shared_ptr<display::SingleShapeDisplay>>
{
    // Create point list object for triangular pyramid
    auto triangular_pyramid_points = shapes::PointList(
        shapes::PointList::FaceName::triangular,
        shapes::PointList::BodyName::pyramid);
    // Create a triangular pyramid
    auto triangular_pyramid = std::make_shared<shapes::TriangleList>(id_global,
        triangular_pyramid_points);
    // Display triangular pyramid object
    auto triangular_pyramid_display = std::make_shared<display::SingleShapeDisplay>(
        "triangular_pyramid" + std::to_string(id_global++), 10ms);
    triangular_pyramid_display->display_object(triangular_pyramid);

    return { triangular_pyramid, triangular_pyramid_display };
}

auto make_square_pyramid(void) -> pair<shared_ptr<shapes::TriangleList>,
    shared_ptr<display::SingleShapeDisplay>> 
{
    // Create point list object for square pyramid
    auto square_pyramid_points = shapes::PointList(
        shapes::PointList::FaceName::square,
        shapes::PointList::BodyName::pyramid);
    // Create a square pyramid
    auto square_pyramid = std::make_shared<shapes::TriangleList>(id_global,
        square_pyramid_points);
    // Display square pyramid object
    auto square_pyramid_display = std::make_shared<display::SingleShapeDisplay>(
        "square_pyramid" + std::to_string(id_global++), 10ms);
    square_pyramid_display->display_object(square_pyramid);

    return { square_pyramid, square_pyramid_display };
}

auto make_rectangular_pyramid(void) -> pair<shared_ptr<shapes::TriangleList>,
    shared_ptr<display::SingleShapeDisplay>>
{
    // Create point list object for rectangular pyramid
    auto rectangular_pyramid_points = shapes::PointList(
        shapes::PointList::FaceName::rectangular,
        shapes::PointList::BodyName::pyramid);
    // Create a rectangular pyramid
    auto rectangular_pyramid = std::make_shared<shapes::TriangleList>(id_global,
        rectangular_pyramid_points);
    // Display rectangular pyramid object
    auto rectangular_pyramid_display = std::make_shared<display::SingleShapeDisplay>(
        "rectangular_pyramid" + std::to_string(id_global++), 10ms);
    rectangular_pyramid_display->display_object(rectangular_pyramid);

    return { rectangular_pyramid, rectangular_pyramid_display };
}

auto make_octagonal_pyramid(void) -> pair<shared_ptr<shapes::TriangleList>,
    shared_ptr<display::SingleShapeDisplay>>
{
    // Create point list object for octagonal pyramid
    auto octagonal_pyramid_points = shapes::PointList(
        shapes::PointList::FaceName::octagonal,
        shapes::PointList::BodyName::pyramid);
    // Create an octagonal pyramid
    auto octagonal_pyramid = std::make_shared<shapes::TriangleList>(id_global,
        octagonal_pyramid_points);
    // Display octagonal pyramid object
    auto octagonal_pyramid_display = std::make_shared<display::SingleShapeDisplay>(
        "octagonal_pyramid" + std::to_string(id_global++), 10ms);
    octagonal_pyramid_display->display_object(octagonal_pyramid);

    return { octagonal_pyramid, octagonal_pyramid_display };
}

auto make_triangular_prism(void) -> pair<shared_ptr<shapes::TriangleList>,
    shared_ptr<display::SingleShapeDisplay>>
{
    // Create point list object for triangular prism
    auto triangular_prism_points = shapes::PointList(
        shapes::PointList::FaceName::triangular,
        shapes::PointList::BodyName::prism);
    // Create a triangular prism
    auto triangular_prism = std::make_shared<shapes::TriangleList>(id_global,
        triangular_prism_points);
    // Display triangular prism object
    auto triangular_prism_display = std::make_shared<display::SingleShapeDisplay>(
        "triangular_prism" + std::to_string(id_global++), 10ms);
    triangular_prism_display->display_object(triangular_prism);
    
    return { triangular_prism, triangular_prism_display };
}

auto make_rectangular_prism(void) -> pair<shared_ptr<shapes::TriangleList>,
    shared_ptr<display::SingleShapeDisplay>>
{
    // Create point list object for rectangular prism
    auto rectangular_prism_points = shapes::PointList(
        shapes::PointList::FaceName::rectangular,
        shapes::PointList::BodyName::prism);
    // Create a rectangular prism
    auto const rectangular_prism = std::make_shared<shapes::TriangleList>(id_global,
        rectangular_prism_points);
    // Display rectangular prism object
    auto rectangular_prism_display = std::make_shared<display::SingleShapeDisplay>
        ("rectangular_prism" + std::to_string(id_global++), 10ms);
    rectangular_prism_display->display_object(rectangular_prism);

    return { rectangular_prism, rectangular_prism_display };
}

auto make_octagonal_prism(void) -> pair<shared_ptr<shapes::TriangleList>,
    shared_ptr<display::SingleShapeDisplay>>
{
    // Create point list object for octagonal prism
    auto octagonal_prism_points = shapes::PointList(
        shapes::PointList::FaceName::octagonal,
        shapes::PointList::BodyName::prism);
    // Create an octagonal prism
    auto octagonal_prism = std::make_shared<shapes::TriangleList>(id_global,
        octagonal_prism_points);
    // Display octagonal prism object
    auto octagonal_prism_display = std::make_shared<display::SingleShapeDisplay>(
        "octagonal_prism" + std::to_string(id_global++), 10ms);
    octagonal_prism_display->display_object(octagonal_prism);
    
    return { octagonal_prism, octagonal_prism_display };
}

auto make_flat_plane(void) -> pair<shared_ptr<shapes::TriangleList>,
    shared_ptr<display::SingleShapeDisplay>> 
{
    // Create point list object for flatplane
    auto flat_plane_points = shapes::PointList(
        shapes::PointList::FaceName::square, 
        shapes::PointList::BodyName::flat_plane);
    // Create flat plane object
    auto flat_plane = std::make_shared<shapes::TriangleList>(id_global,
        flat_plane_points);
    // Display flat plane object
    auto flat_plane_display = std::make_shared<display::SingleShapeDisplay>(
        "flat_plane" + std::to_string(id_global++), 10ms);
    flat_plane_display->display_object(flat_plane);

    return { flat_plane, flat_plane_display };
}

auto make_parallelepiped(void) -> pair<shared_ptr<shapes::TriangleList>,
    shared_ptr<display::SingleShapeDisplay>>
{
    // Create point list object for parallelepiped
    auto parallelepiped_points = shapes::PointList(
        shapes::PointList::FaceName::square,
        shapes::PointList::BodyName::parallelepiped);
    // Create a parallelepiped
    auto parallelepiped = std::make_shared<shapes::TriangleList>(id_global,
        parallelepiped_points);
    // Display parallelepiped object
    auto parallelepiped_display = std::make_shared<display::SingleShapeDisplay>(
        "parallelepiped" + std::to_string(id_global++), 10ms);
    parallelepiped_display->display_object(parallelepiped);

    return { parallelepiped, parallelepiped_display };
}
} // namespace shapes
