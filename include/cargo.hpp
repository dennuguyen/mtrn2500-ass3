#ifndef CARGO_HPP_
#define CARGO_HPP_

#include "interfaces.hpp"
#include "cone.hpp"
#include "cube.hpp"
#include "cylinder.hpp"
#include "point_list.hpp"
#include "sphere.hpp"
#include "triangle_list.hpp"
#include "assembly.hpp"
#include "uav.hpp"

#include <iostream>
#include <tuple>

namespace shapes
{
class Cargo
{
public:
    Cargo(std::shared_ptr<shapes::Sphere> sphere,
        std::shared_ptr<display::SingleShapeDisplay> sphere_display);
    ~Cargo(void);

    std::vector<std::shared_ptr<display::SingleShapeDisplay>>
        & get_display_list(void);

    std::vector<std::shared_ptr<shapes::Sphere>>
        & get_cargo_list(void);

    auto tick(UAV & uav) -> void;

    auto drop_shape(void) -> void;
    auto spawn(void) -> void;
    auto next_colour(auto & shape) -> void;
    auto clear_shapes(void) -> void;

private:
    int colour_count_;
    std::shared_ptr<shapes::Sphere> sphere_; // Cargo being held
    std::shared_ptr<display::SingleShapeDisplay> sphere_display_;
    std::vector<std::shared_ptr<display::SingleShapeDisplay>> display_list_;
    std::vector<std::shared_ptr<shapes::Sphere>> cargo_list_; // Dropped cargo
};
} // namespace shapes
#endif // CARGO_HPP_