// Dan Nguyen (z5206032) &&
// Mon-1500

#ifndef CUBE_HPP_
#define CUBE_HPP_

#include "interfaces.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include <memory>
#include <string>
#include <tuple>
#include <vector>

namespace shapes
{
class Cube : public ShapeCommonInterface
{
public:
    explicit Cube(int id);

protected:
    AllAxis size_;
    std::string parent_frame_name_;
    std::shared_ptr<std::vector<visualization_msgs::msg::Marker>>
        shapes_list_ptr_;

    auto resize_imple(AllAxis new_size) -> void override;
    auto rescale_imple(AnyAxis factor) -> void override;

    // Colour methods
    auto set_colour_imple(Colour const new_colour) -> void override;
    [[nodiscard]] auto get_colour_imple(void) const -> Colour override;
    
    auto set_parent_frame_name_imple(std::string frame_name) -> void override;

    [[nodiscard]] auto get_location_imple(void) const
        -> std::tuple<XAxis, YAxis, ZAxis> override;

    // move_to_imple methods
    auto move_to_imple(XAxis x) -> void override;
    auto move_to_imple(YAxis y) -> void override;
    auto move_to_imple(ZAxis z) -> void override;
    auto move_to_imple(XAxis x, YAxis y, ZAxis z) -> void override;

    // move_by_imple methods
    auto move_by_imple(XAxis x) -> void override;
    auto move_by_imple(YAxis y) -> void override;
    auto move_by_imple(ZAxis z) -> void override;
    auto move_by_imple(XAxis x, YAxis y, ZAxis z) -> void override;

    // Display markers
    auto get_display_markers_imple(void) -> std::shared_ptr<
        std::vector<visualization_msgs::msg::Marker>> override;

    // Axis methods
    auto rotate_about_axis_to_imple(ZAxis radians) -> void override;
    [[nodiscard]] auto get_orientation_imple(void) const -> ZAxis override;
};
} // namespace shapes
#endif // CUBE_HPP_
