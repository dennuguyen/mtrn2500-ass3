#ifndef SHAPE_FACTORY_HPP_
#define SHAPE_FACTORY_HPP_

#include "interfaces.hpp"
#include "rclcpp/rclcpp.hpp" 
#include "single_shape_display.hpp"

#include "cone.hpp"
#include "cube.hpp"
#include "cylinder.hpp"
#include "sphere.hpp"
#include "triangle_list.hpp"

using std::pair;
using std::shared_ptr;

namespace display
{
// Make primitives
auto make_cone(void) -> pair<shared_ptr<shapes::Cone>,
    shared_ptr<display::SingleShapeDisplay>>;
auto make_cube(void) -> pair<shared_ptr<shapes::Cube>,
    shared_ptr<display::SingleShapeDisplay>>;
auto make_cylinder(void) -> pair<shared_ptr<shapes::Cylinder>,
    shared_ptr<display::SingleShapeDisplay>>;
auto make_sphere(void) -> pair<shared_ptr<shapes::Sphere>,
    shared_ptr<display::SingleShapeDisplay>>;

// Make pyramids
auto make_triangular_pyramid(void) -> pair<shared_ptr<shapes::TriangleList>,
    shared_ptr<display::SingleShapeDisplay>>;
auto make_square_pyramid(void) -> pair<shared_ptr<shapes::TriangleList>,
    shared_ptr<display::SingleShapeDisplay>>;
auto make_rectangular_pyramid(void) -> pair<shared_ptr<shapes::TriangleList>,
    shared_ptr<display::SingleShapeDisplay>>;
auto make_octagonal_pyramid(void) -> pair<shared_ptr<shapes::TriangleList>,
    shared_ptr<display::SingleShapeDisplay>>;

// Make prisms
auto make_triangular_prism(void) -> pair<shared_ptr<shapes::TriangleList>,
    shared_ptr<display::SingleShapeDisplay>>;
auto make_rectangular_prism(void) -> pair<shared_ptr<shapes::TriangleList>,
    shared_ptr<display::SingleShapeDisplay>>;
auto make_octagonal_prism(void) -> pair<shared_ptr<shapes::TriangleList>,
    shared_ptr<display::SingleShapeDisplay>>;

// Make other shapes
auto make_flat_plane(void) -> pair<shared_ptr<shapes::TriangleList>,
    shared_ptr<display::SingleShapeDisplay>>;
auto make_parallelepiped() -> pair<shared_ptr<shapes::TriangleList>,
    shared_ptr<display::SingleShapeDisplay>>;
} // namespace shapes
#endif // SHAPE_FACTORY_HPP_