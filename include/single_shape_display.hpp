// Copyright 2019 Zhihao Zhang License MIT

#ifndef SINGLE_SHAPE_DISPLAY_HPP_
#define SINGLE_SHAPE_DISPLAY_HPP_

#include "interfaces.hpp"
#include "rclcpp/rclcpp.hpp" // http://docs.ros2.org/dashing/api/rclcpp/
#include "visualization_msgs/msg/marker.hpp"

#include <chrono>
#include <memory>
#include <string>

namespace display
{

/**
 * \brief SingleShapeDisplay class can display one shape to RVIZ. It will
 * periodically get the marker message from the shape object and send it to ros
 * topic `z0000000/marker`. RVIZ will display the shape when the topic is added
 * in RVIZ setting.
 */
class SingleShapeDisplay final : public rclcpp::Node,
                                 public DisplayOutputInterface
{
public:
    /**
     * \brief Constructor
     * \param node_name is the ROS node name of this object. Must be unique.
     * \param refresh_period is how often to refresh the shape message.
     */
    explicit SingleShapeDisplay(std::string const & node_name,
        std::chrono::milliseconds refresh_period);

private:
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
        marker_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<shapes::DisplayableInterface> object_to_be_displayed_;

    auto marker_publisher_callback() -> void;

    /**
     * \brief Implement sending marker from one object to display topic.
     * \param display_object is the object to be displayed.
     */
    auto display_object_imple(
        std::shared_ptr<shapes::DisplayableInterface> display_object)
        -> void final;
};
} // namespace display
#endif // SINGLE_SHAPE_DISPLAY_HPP_
