// Dan Nguyen (z5206032) &&
// Mon-1500

// The environment consists of one of each shape
// environment.cpp encapsulates the instantiation of shape classes and its display

#include "environment.hpp"

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
// Constructor
Environment::Environment(void)
{
    // Ground
    Assembly::get_flat_plane()->set_colour(ColourInterface::Colour::green);
    Assembly::get_flat_plane()->rotate_about_axis_to(ZAxis{helper::pi / 4});

    // Sun
    Assembly::get_sphere()->move_to(XAxis{5.0}, YAxis{5.0}, ZAxis{7.0});
    Assembly::get_sphere()->set_colour(ColourInterface::Colour::yellow);

    // House
    Assembly::get_cube()->move_to(XAxis{4.0}, YAxis{4.0}, ZAxis{0.5});
    Assembly::get_cube()->set_colour(ColourInterface::Colour::red);
    Assembly::get_cube()->rotate_about_axis_to(ZAxis{helper::pi / 4});
    Assembly::get_square_pyramid()->move_to(XAxis{4.0}, YAxis{4.0}, ZAxis{1.0});
    Assembly::get_square_pyramid()->set_colour(ColourInterface::Colour::red);

    // Tree
    Assembly::get_cone()->move_to(XAxis{-1.0}, YAxis{-3.0}, ZAxis{1.0});
    Assembly::get_cone()->set_colour(ColourInterface::Colour::green);
    Assembly::get_cone()->rescale(AnyAxis{2});
    Assembly::get_cylinder()->move_to(XAxis{-1.0}, YAxis{-3.0}, ZAxis{0.5});
    Assembly::get_cylinder()->set_colour(ColourInterface::Colour::black);

    // Water Tank
    Assembly::get_octagonal_prism()->move_to(XAxis{-5.0}, YAxis{6.0}, ZAxis{1.0});
    Assembly::get_octagonal_prism()->set_colour(ColourInterface::Colour::white);
    Assembly::get_octagonal_prism()->rescale(AnyAxis{1.5});
    Assembly::get_triangular_prism()->move_to(XAxis{-5.0}, YAxis{5.0}, ZAxis{0.0});
    Assembly::get_triangular_prism()->set_colour(ColourInterface::Colour::white);
    Assembly::get_octagonal_pyramid()->move_to(XAxis{-5.0}, YAxis{7.0}, ZAxis{0.0});
    Assembly::get_octagonal_pyramid()->set_colour(ColourInterface::Colour::white);
    
    // Car
    Assembly::get_rectangular_prism()->move_to(XAxis{6.0}, YAxis{5.0}, ZAxis{0.0});
    Assembly::get_rectangular_prism()->set_colour(ColourInterface::Colour::blue);
    Assembly::get_rectangular_prism()->rotate_about_axis_to(ZAxis{helper::pi / 4});
    Assembly::get_rectangular_prism()->rescale(AnyAxis{0.5});
    Assembly::get_parallelepiped()->move_to(XAxis{6.0}, YAxis{5.0}, ZAxis{0.5});
    Assembly::get_parallelepiped()->set_colour(ColourInterface::Colour::blue);
    Assembly::get_parallelepiped()->rescale(AnyAxis{0.5});

    // Some pyramids
    Assembly::get_rectangular_pyramid()->move_to(XAxis{-9.0}, YAxis{-3}, ZAxis{0.0});
    Assembly::get_rectangular_pyramid()->set_colour(ColourInterface::Colour::yellow);
    Assembly::get_triangular_pyramid()->move_to(XAxis{-11.0}, YAxis{6}, ZAxis{0.0});
    Assembly::get_triangular_pyramid()->set_colour(ColourInterface::Colour::yellow);
}

// Destructor
Environment::~Environment(void)
{
}
} // namespace shapes
