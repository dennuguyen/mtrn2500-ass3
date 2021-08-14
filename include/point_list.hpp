#ifndef POINT_LIST_HPP_
#define POINT_LIST_HPP_

#include "interfaces.hpp"
#include "student_helper.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include <memory>
#include <string>
#include <tuple>
#include <vector>

namespace shapes
{
class PointList
{
public:
    enum class FaceName
    {
        triangular = 3,
        square = 4,
        rectangular = 5,
        octagonal = 8
    };

    enum class BodyName
    {
        pyramid,
        prism,
        parallelepiped, // special case
        flat_plane // special case
    };

    PointList(FaceName face_name, BodyName body_name);

    // Set and get the points of the vertices of any regular 3D polygon shape
    auto set_points(void) -> void;
    auto set_pyramid_points(void) -> void;
    auto set_prism_points(void) -> void;

    // Calculates the x and y coordinates of the vertices of regular polygons
    auto get_x(int i) -> double;
    auto get_y(int i) -> double;

    auto get_points(void)
        -> std::shared_ptr<std::vector<geometry_msgs::msg::Point>>;
    auto get_face_name(FaceName name) -> std::string;
    auto get_body_name(BodyName name) -> std::string;
    
private:
    std::shared_ptr<std::vector<geometry_msgs::msg::Point>> vertex_list_ptr_;
    FaceName face_name_;
    BodyName body_name_;
    int side_number_;
};
} // namespace shapes
#endif // POINT_LIST_HPP_
