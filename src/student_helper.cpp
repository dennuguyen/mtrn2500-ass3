// Copyright 2019 Zhihao Zhang License MIT

#include "student_helper.hpp"

namespace helper
{
/// Helper function to calculate the time derivative
auto get_time(rclcpp::Time current_time, rclcpp::Time previous_time) -> double
{
    // Change in time
    rclcpp::Duration dt = current_time - previous_time;

    // Return double
    return dt.seconds();
}
} // namespace helper


/*
// Spawn shapes with an iteration through the choice of shapes
auto spawn(void) -> void
{
    switch (shape_count_)
    {
        case shape_count_ == 0:
            auto [cone, cone_display] = display::make_cone();
            cone_ = cone;
            cone_display_ = cone_display;
            shape_count_++;
            break;

        case shape_count_ == 1:
            auto [cube, cube_display] = display::make_cube();
            cube_ = cube;
            cube_display_ = cube_display;
            shape_count_++;
            break;

        case shape_count_ == 2:
            auto [cylinder, cylinder_display] = display::make_cylinder();
            cylinder_ = cylinder;
            cylinder_display_ = cylinder_display;
            shape_count_++;
            break;

        case shape_count_ == 3:
            auto [sphere, sphere_display] = display::make_sphere();
            sphere_ = sphere;
            sphere_display_ = sphere_display;
            shape_count_++;
            break;

        case shape_count_ == 4:
            auto [triangular_pyramid, triangular_pyramid_display] = display::make_triangular_pyramid();
            triangular_pyramid_ = triangular_pyramid;
            triangular_pyramid_display_ = triangular_pyramid_display;
            shape_count_++;
            break;

        case shape_count_ == 5:
            auto [square_pyramid, square_pyramid_display] = display::make_square_pyramid();
            square_pyramid_ = square_pyramid;
            square_pyramid_display_ = square_pyramid_display;
            shape_count_++;
            break;

        case shape_count_ == 6:
            auto [rectangular_pyramid, rectangular_pyramid_display] = display::make_rectangular_pyramid();
            rectangular_pyramid_ = rectangular_pyramid;
            rectangular_pyramid_display_ = rectangular_pyramid_display;
            shape_count_++;
            break;

        case shape_count_ == 7:
            auto [octagonal_pyramid, octagonal_pyramid_display] = display::make_octagonal_pyramid();
            octagonal_pyramid_ = octagonal_pyramid;
            octagonal_pyramid_display_ = octagonal_pyramid_display;
            shape_count_++;
            break;

        case shape_count_ == 8:
            auto [triangular_prism, triangular_prism_display] = display::make_triangular_prism();
            triangular_prism_ = triangular_prism;
            triangular_prism_display_ = triangular_prism_display;
            shape_count_++;
            break;

        case shape_count_ == 9:
            auto [rectangular_prism, rectangular_prism_display] = display::make_rectangular_prism();
            rectangular_prism_ = rectangular_prism;
            rectangular_prism_display_ = rectangular_prism_display;
            shape_count_++;
            break;

        case shape_count_ == 10:
            auto [octagonal_prism, octagonal_prism_display] = display::make_octagonal_prism();
            octagonal_prism_ = octagonal_prism;
            octagonal_prism_display_ = octagonal_prism_display;
            shape_count_++;
            break;

        case shape_count_ == 11:
            auto [flat_plane, flat_plane_display] = display::make_flat_plane();
            flat_plane_ = flat_plane;
            flat_plane_display_ = flat_plane_display;
            shape_count_++;
            break;

        case shape_count_ == 12:
            auto [parallelepiped, parallelepiped_display] = display::make_parallelepiped();
            parallelepiped_ = parallelepiped;
            parallelepiped_display_ = parallelepiped_display;
            shape_count_ == 0; // reset the shape count
            break;
    }
}
*/
/*
// Getters for shape objects
auto Cargo::get_cone(void) -> std::shared_ptr<shapes::Cone> { return cone_; }
auto Cargo::get_cube(void) -> std::shared_ptr<shapes::Cube> { return cube_; }
auto Cargo::get_cylinder(void) -> std::shared_ptr<shapes::Cylinder> { return cylinder_; }
auto Cargo::get_sphere(void) -> std::shared_ptr<shapes::Sphere> { return sphere_; }
auto Cargo::get_triangular_pyramid(void) -> std::shared_ptr<shapes::TriangleList> { return triangular_pyramid_; }
auto Cargo::get_square_pyramid(void) -> std::shared_ptr<shapes::TriangleList> { return square_pyramid_; }
auto Cargo::get_rectangular_pyramid(void) -> std::shared_ptr<shapes::TriangleList> { return rectangular_pyramid_; }
auto Cargo::get_octagonal_pyramid(void) -> std::shared_ptr<shapes::TriangleList> { return octagonal_pyramid_; }
auto Cargo::get_triangular_prism(void) -> std::shared_ptr<shapes::TriangleList> { return triangular_prism_; }
auto Cargo::get_rectangular_prism(void) -> std::shared_ptr<shapes::TriangleList> { return rectangular_prism_; }
auto Cargo::get_octagonal_prism(void) -> std::shared_ptr<shapes::TriangleList> { return octagonal_prism_; }
auto Cargo::get_flat_plane(void) -> std::shared_ptr<shapes::TriangleList> { return flat_plane_; }
auto Cargo::get_parallelepiped(void) -> std::shared_ptr<shapes::TriangleList> { return parallelepiped_; }

// Getters for display objects
auto Cargo::get_cone(void) -> std::shared_ptr<display::SingleShapeDisplay> { return cone_display_; }
auto Cargo::get_cube(void) -> std::shared_ptr<display::SingleShapeDisplay> { return cube_display_; }
auto Cargo::get_cylinder(void) -> std::shared_ptr<display::SingleShapeDisplay> { return cylinder_display_; }
auto Cargo::get_sphere(void) -> std::shared_ptr<display::SingleShapeDisplay> { return sphere_display_; }
auto Cargo::get_triangular_pyramid(void) -> std::shared_ptr<display::SingleShapeDisplay> { return triangular_pyramid_display_; }
auto Cargo::get_square_pyramid(void) -> std::shared_ptr<display::SingleShapeDisplay> { return square_pyramid_display_; }
auto Cargo::get_rectangular_pyramid(void) -> std::shared_ptr<display::SingleShapeDisplay> { return rectangular_pyramid_display_; }
auto Cargo::get_octagonal_pyramid(void) -> std::shared_ptr<display::SingleShapeDisplay> { return octagonal_pyramid_display_; }
auto Cargo::get_triangular_prism(void) -> std::shared_ptr<display::SingleShapeDisplay> { return triangular_prism_display_; }
auto Cargo::get_rectangular_prism(void) -> std::shared_ptr<display::SingleShapeDisplay> { return rectangular_prism_display_; }
auto Cargo::get_octagonal_prism(void) -> std::shared_ptr<display::SingleShapeDisplay> { return octagonal_prism_display_; }
auto Cargo::get_flat_plane(void) -> std::shared_ptr<display::SingleShapeDisplay> { return flat_plane_display_; }
auto Cargo::get_parallelepiped(void) -> std::shared_ptr<display::SingleShapeDisplay> { return parallelepiped_display_; }
*/

/*
    // Getter methods shape objects
    auto get_cone(void) -> std::shared_ptr<shapes::Cone>;
    auto get_cube(void) -> std::shared_ptr<shapes::Cube>;
    auto get_cylinder(void) -> std::shared_ptr<shapes::Cylinder>;
    auto get_sphere(void) -> std::shared_ptr<shapes::Sphere>;
    auto get_triangular_pyramid(void) -> std::shared_ptr<shapes::TriangleList>;
    auto get_square_pyramid(void) -> std::shared_ptr<shapes::TriangleList>;
    auto get_rectangular_pyramid(void) -> std::shared_ptr<shapes::TriangleList>;
    auto get_octagonal_pyramid(void) -> std::shared_ptr<shapes::TriangleList>;
    auto get_triangular_prism(void) -> std::shared_ptr<shapes::TriangleList>;
    auto get_rectangular_prism(void) -> std::shared_ptr<shapes::TriangleList>;
    auto get_octagonal_prism(void) -> std::shared_ptr<shapes::TriangleList>;
    auto get_flat_plane(void) -> std::shared_ptr<shapes::TriangleList>;
    auto get_parallelepiped(void) -> std::shared_ptr<shapes::TriangleList>;

    // Getter methods for display objects
    auto get_cone_display(void) -> std::shared_ptr<display::SingleShapeDisplay>;
    auto get_cube_display(void) -> std::shared_ptr<display::SingleShapeDisplay>;
    auto get_cylinder_display(void) -> std::shared_ptr<display::SingleShapeDisplay>;
    auto get_sphere_display(void) -> std::shared_ptr<display::SingleShapeDisplay>;
    auto get_triangular_pyramid_display(void) -> std::shared_ptr<display::SingleShapeDisplay>;
    auto get_square_pyramid_display(void) -> std::shared_ptr<display::SingleShapeDisplay>;
    auto get_rectangular_pyramid_display(void) -> std::shared_ptr<display::SingleShapeDisplay>;
    auto get_octagonal_pyramid_display(void) -> std::shared_ptr<display::SingleShapeDisplay>;
    auto get_triangular_prism_display(void) -> std::shared_ptr<display::SingleShapeDisplay>;
    auto get_rectangular_prism_display(void) -> std::shared_ptr<display::SingleShapeDisplay>;
    auto get_octagonal_prism_display(void) -> std::shared_ptr<display::SingleShapeDisplay>;
    auto get_flat_plane_display(void) -> std::shared_ptr<display::SingleShapeDisplay>;
    auto get_parallelepiped_display(void) -> std::shared_ptr<display::SingleShapeDisplay>;
*/
/*
    std::shared_ptr<shapes::Cone> cone_;
    std::shared_ptr<shapes::Cube> cube_;
    std::shared_ptr<shapes::Cylinder> cylinder_;
    std::shared_ptr<shapes::Sphere> sphere_;
    std::shared_ptr<shapes::TriangleList> triangular_pyramid_;
    std::shared_ptr<shapes::TriangleList> square_pyramid_;
    std::shared_ptr<shapes::TriangleList> rectangular_pyramid_;
    std::shared_ptr<shapes::TriangleList> octagonal_pyramid_;
    std::shared_ptr<shapes::TriangleList> triangular_prism_;
    std::shared_ptr<shapes::TriangleList> rectangular_prism_;
    std::shared_ptr<shapes::TriangleList> octagonal_prism_;
    std::shared_ptr<shapes::TriangleList> flat_plane_;
    std::shared_ptr<shapes::TriangleList> parallelepiped_;

    std::shared_ptr<display::SingleShapeDisplay> cone_display_;
    std::shared_ptr<display::SingleShapeDisplay> cube_display_;
    std::shared_ptr<display::SingleShapeDisplay> cylinder_display_;
    std::shared_ptr<display::SingleShapeDisplay> sphere_display_;
    std::shared_ptr<display::SingleShapeDisplay> triangular_pyramid_display_;
    std::shared_ptr<display::SingleShapeDisplay> square_pyramid_display_;
    std::shared_ptr<display::SingleShapeDisplay> rectangular_pyramid_display_;
    std::shared_ptr<display::SingleShapeDisplay> octagonal_pyramid_display_;
    std::shared_ptr<display::SingleShapeDisplay> triangular_prism_display_;
    std::shared_ptr<display::SingleShapeDisplay> rectangular_prism_display_;
    std::shared_ptr<display::SingleShapeDisplay> octagonal_prism_display_;
    std::shared_ptr<display::SingleShapeDisplay> flat_plane_display_;
    std::shared_ptr<display::SingleShapeDisplay> parallelepiped_display_;
*/