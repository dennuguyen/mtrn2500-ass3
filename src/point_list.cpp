#include "point_list.hpp"

#include <cassert>
#include <iostream>
#include <math.h>

namespace shapes
{
PointList::PointList(FaceName face_name, BodyName body_name)
    : face_name_{face_name}
    , body_name_{body_name}
    , vertex_list_ptr_{
        std::make_shared<std::vector<geometry_msgs::msg::Point>>()}
{
    // Create references for use
    auto & vertex_list = *vertex_list_ptr_;
    auto & vertex = vertex_list[0];
    
    // Set the side number
    side_number_ = (int)face_name_;

    // FaceName::rectangular was assigned 5 to avoid duplicate with square
    if (get_face_name(face_name_) == get_face_name(FaceName::rectangular))
    {
        side_number_ = 4;
    }

    // Get the vertices of the 3D shape
    set_points();
}

// Gets the points of the face of a specified regular polygon
// set_points has void parameter as shape deformation is not a feature
auto PointList::set_points(void) -> void
{
    if (get_body_name(body_name_) == get_body_name(BodyName::pyramid) ||
        get_body_name(body_name_) == get_body_name(BodyName::flat_plane))
    {
        set_pyramid_points();
    }
    else if (get_body_name(body_name_) == get_body_name(BodyName::prism) ||
        get_body_name(body_name_) == get_body_name(BodyName::parallelepiped))
    {
        set_prism_points();
    }
    
    // Number of total points must be divisible by 3
    assert(vertex_list_ptr_->size() % 3 == 0);
}

// Creates the vertices for a regular polygonal pyramid
auto PointList::set_pyramid_points(void) -> void
{
    geometry_msgs::msg::Point A, B, C, D;

    for (int i = 0; i < side_number_;)
    {
        A.x = get_x(i);
        A.y = get_y(i);
        A.z = 0.0;
        
        i++;

        B.x = get_x(i);
        B.y = get_y(i);
        B.z = 0.0;

        C.x = 0.0;
        C.y = 0.0;
        C.z = 0.0;

        D.x = 0.0;
        D.y = 0.0;
        D.z = 1.0;

        vertex_list_ptr_->push_back(A);
        vertex_list_ptr_->push_back(B);
        vertex_list_ptr_->push_back(C);
        
        if (get_body_name(body_name_) == get_body_name(BodyName::flat_plane))
        {
            // Do not push in special case of flat_plane
        }
        else
        {
            vertex_list_ptr_->push_back(A);
            vertex_list_ptr_->push_back(B);
            vertex_list_ptr_->push_back(D);
        }
    }
}

// Creates the vertices for a regular polygonal prism
auto PointList::set_prism_points(void) -> void
{
    geometry_msgs::msg::Point A, B, C, AA, BB, CC;

    double slant = 0.0;

    if (get_body_name(body_name_) == get_body_name(BodyName::parallelepiped))
    {
        slant = 0.5;
    }

    for (int i = 0; i < side_number_;)
    {
        A.x = get_x(i);
        A.y = get_y(i);
        A.z = 0.0;
        
        i++;

        B.x = get_x(i);
        B.y = get_y(i);
        B.z = 0.0;

        C.x = 0.0;
        C.y = 0.0;
        C.z = 0.0;

        AA.x = A.x + slant;
        AA.y = A.y + slant;
        AA.z = 1.0;

        BB.x = B.x + slant;
        BB.y = B.y + slant;
        BB.z = 1.0;

        CC.x = C.x + slant;
        CC.y = C.y + slant;
        CC.z = 1.0;

        // Push the points for the polygonal face
        vertex_list_ptr_->push_back(C);
        vertex_list_ptr_->push_back(B);
        vertex_list_ptr_->push_back(A);

        vertex_list_ptr_->push_back(AA);
        vertex_list_ptr_->push_back(BB);
        vertex_list_ptr_->push_back(CC);

        // Push the points for the prism face
        vertex_list_ptr_->push_back(AA);
        vertex_list_ptr_->push_back(A);
        vertex_list_ptr_->push_back(B);

        vertex_list_ptr_->push_back(BB);
        vertex_list_ptr_->push_back(AA);
        vertex_list_ptr_->push_back(B);
    }
}

// Uses radial coordinates with centre origin and unit radius
// Multiplier elongates the conic along the x-axis
auto PointList::get_x(int i) -> double
{
    double multiplier = 1.0;
    double constant = 0.0;

    if (get_body_name(body_name_) == get_body_name(BodyName::flat_plane))
    {
        multiplier = 20.0;
    }
    else if (get_face_name(face_name_) == get_face_name(FaceName::rectangular))
    {
        multiplier = 1.5;
        constant = helper::pi/4;
    }

    return multiplier * cos(2 * helper::pi * i / side_number_ + constant);
}

// Uses radial coordinates with centre origin and unit radius
auto PointList::get_y(int i) -> double
{
    double multiplier = 1.0;
    double constant = 0.0;

    if (get_body_name(body_name_) == get_body_name(BodyName::flat_plane))
    {
        multiplier = 20.0;
    }
    else if (get_face_name(face_name_) == get_face_name(FaceName::rectangular))
    {
        constant = helper::pi/4;
    }

    return multiplier * sin(2 * helper::pi * i / side_number_ + constant);
}

// Returns the pointer to the vertex_list_
auto PointList::get_points(void)
    -> std::shared_ptr<std::vector<geometry_msgs::msg::Point>>
{
    assert(!vertex_list_ptr_->empty());
    return vertex_list_ptr_;
}

// Returns the FaceName enum as a string
auto PointList::get_face_name(FaceName name) -> std::string
{
    switch (name)
    {
        case FaceName::triangular:
            return "triangular";
        case FaceName::square:
            return "square";
        case FaceName::rectangular:
            return "rectangular";
        case FaceName::octagonal:
            return "octagonal";
    }
}

// Returns the BodyName enum as a string
auto PointList::get_body_name(BodyName name) -> std::string
{
    switch (name)
    {
        case BodyName::pyramid:
            return "pyramid";
        case BodyName::prism:
            return "prism";
        case BodyName::parallelepiped:
            return "parallelepiped";
        case BodyName::flat_plane:
            return "flat_plane";
    }
}
} // namespace shapes
