// Dan Nguyen (z5206032) &&
// Mon-1500

#ifndef ASSEMBLY_HPP_
#define ASSEMBLY_HPP_

#include "interfaces.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "single_shape_display.hpp"

#include "cone.hpp"
#include "cube.hpp"
#include "cylinder.hpp"
#include "point_list.hpp"
#include "sphere.hpp"
#include "triangle_list.hpp"

#include <memory>
#include <string>
#include <tuple>
#include <vector>

namespace shapes
{
class Assembly
{
public:
    explicit Assembly(void);
    ~Assembly(void);
    
    const std::vector<std::shared_ptr<display::SingleShapeDisplay>>
        & get_display_list(void);

    auto get_shapes_list_ptr(void) ->
        std::shared_ptr<std::vector<visualization_msgs::msg::Marker>>;

    // Getter methods
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

protected:
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

    std::vector<std::shared_ptr<display::SingleShapeDisplay>> display_list_;
    
    std::shared_ptr<std::vector<visualization_msgs::msg::Marker>>
        shapes_list_ptr_;
};
} // namespace shapes
#endif // ASSEMBLY_HPP_
