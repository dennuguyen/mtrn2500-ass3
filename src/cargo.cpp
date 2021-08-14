#include "cargo.hpp"

#include "shape_factory.hpp"

int x_cargo, y_cargo, z_cargo, o_cargo;

namespace shapes
{
// Constructor
Cargo::Cargo(std::shared_ptr<shapes::Sphere> sphere,
    std::shared_ptr<display::SingleShapeDisplay> sphere_display)
    : colour_count_{1}
    , sphere_{sphere}
    , sphere_display_{sphere_display}
{
    display_list_.push_back(sphere_display);
}

// Destructor
Cargo::~Cargo(void) {}

// Get display list
std::vector<std::shared_ptr<display::SingleShapeDisplay>>
    & Cargo::get_display_list(void)
{
    return display_list_;
}

std::vector<std::shared_ptr<shapes::Sphere>>
    & Cargo::get_cargo_list(void)
{
    return cargo_list_;
}

// Update state of UAV cargo
auto Cargo::tick(UAV & uav) -> void
{
    // Update the position of the held block with respect to the UAV pose
    sphere_->move_to(XAxis{x_cargo}, YAxis{y_cargo}, ZAxis{z_cargo - 0.5});
    sphere_->rotate_about_axis_to(ZAxis{o_cargo});
}

// Drop a shape
auto Cargo::drop_shape(void) -> void
{
    // Release the shape
    //
    // Push shape into cargo_list vector to keep track of fallen cargo
    cargo_list_.push_back(sphere_);

    // Spawn the next shape
    spawn();
}

// Spawn a sphere
auto Cargo::spawn(void) -> void
{
    auto [sphere, sphere_display] = display::make_sphere();
    
    sphere_ = sphere;

    next_colour(sphere_);

    // To display the new shape
    display_list_.push_back(sphere_display);
}

// Iterate through colours
auto Cargo::next_colour(auto & shape) -> void
{
    switch (colour_count_)
    {
        case 0:
            shape->set_colour(ColourInterface::Colour::red);
            colour_count_++;
            break;

        case 1:
            shape->set_colour(ColourInterface::Colour::yellow);
            colour_count_++;
            break;

        case 2:
            shape->set_colour(ColourInterface::Colour::green);
            colour_count_++;
            break;

        case 3:
            shape->set_colour(ColourInterface::Colour::blue);
            colour_count_++;
            break;

        case 4:
            shape->set_colour(ColourInterface::Colour::black);
            colour_count_++;
            break;

        case 5:
            shape->set_colour(ColourInterface::Colour::white);
            colour_count_ = 0; // Reset the colour count
            break;
    }
}

// Delete the dropped shapes
auto Cargo::clear_shapes(void) -> void
{
    cargo_list_.clear();
    display_list_.clear();
}
} // namespace shapes
