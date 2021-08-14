// SPDX-License-Identifier: MIT
/**
 *  \brief     Assignment 3 starter program
 *  \details   Program will show a sphere in RVIZ2
 *  \author    Zhihao Zhang
 *  \version   0.11.15
 *  \date      Nov 2019
 *  \copyright MIT
 **/

#include "interfaces.hpp"
#include "rclcpp/rclcpp.hpp" // http://docs.ros2.org/dashing/api/rclcpp/
#include "single_shape_display.hpp"
#include "shape_factory.hpp"

#include "cone.hpp"
#include "cube.hpp"
#include "cylinder.hpp"
#include "point_list.hpp"
#include "sphere.hpp"
#include "triangle_list.hpp"
#include "assembly.hpp"
#include "uav.hpp"
#include "environment.hpp"
#include "cargo.hpp"

#include "config_parser.hpp"
#include "joystick_listener.hpp"
#include "marker_broadcaster.hpp"
#include "pose_kinematic.hpp"
#include "student_helper.hpp"
#include "transform_broadcaster.hpp"
#include "velocity_kinematic.hpp"

#include <chrono>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

// Globals because I ran out of time
int id_global = 0; // Tracks id of instantiated shapes
int is_dropped = !(DROPPED);
int is_cleared = !(CLEARED);

// Helper function
static auto create_visualisation_node(
    std::string const & zid, std::chrono::milliseconds refresh_period)
    -> std::shared_ptr<assignment2::MarkerBroadcaster>
{
    auto shape_list =
        std::make_shared<std::vector<visualization_msgs::msg::Marker>>();

    return std::make_shared<assignment2::MarkerBroadcaster>(
        zid, refresh_period, shape_list);
}

// Shapes have gravity and falls at a constant velocity of 0.1 m/s
auto gravity(shapes::Sphere & shape) -> void
{
    if (std::get<2>(shape.get_location()).get_value() >= 0.5)
    {
        shape.move_by(shapes::ZAxis{-0.1});
    }
}

auto main(int argc, char * argv[]) -> int
{
    using namespace std::chrono_literals;

    try
    {
        rclcpp::init(argc, argv); // Initialise ROS2

        // Executor will handle running multiple nodes at the same time for us.
        auto ros_worker = rclcpp::executors::SingleThreadedExecutor{};

        // Check number of arguments as indicator of config file
        if (argc == 1)
        {
            // No specified config file
            std::cerr << "\"ros2 run assignment3 assignment3 <MISSING>\"" 
                << std::endl;
        }
        else if (argc > 1)
        {   
            std::cout << "\nDemo program starting, press enter to display shape.\n";
            std::cin.ignore();

            // Opening the config file
            assert(argv[1] != NULL);
            std::ifstream config_file (argv[1]);
            assert(config_file.is_open());

            // Reading and parsing the config file
            auto const config_strings = assignment2::ConfigReader{config_file};
            auto const config = assignment2::ConfigParser{config_strings};

            // Creating all nodes we need and register with executor that will
            // service those nodes.
            auto input_node = std::make_shared<assignment2::JoystickListener>(
                "z0000000", config.get_joystick_config());
            
            ros_worker.add_node(input_node);

            auto velocity_node = std::make_shared<
                assignment2::VelocityKinematic>("z0000000", 200ms,
                    config.get_kinematic_config());
            
            ros_worker.add_node(velocity_node);

            auto pose_node = std::make_shared<assignment2::PoseKinematic>(
                "z0000000", 100ms);
            
            ros_worker.add_node(pose_node);

            auto visual_node = create_visualisation_node("z0000000", 100ms);

            ros_worker.add_node(visual_node);

            auto transform_node =
                std::make_shared<assignment2::TransformBroadcaster>("z0000000");
                
            ros_worker.add_node(transform_node);

            /// Create the environment
            auto environment = std::make_shared<shapes::Environment>();
            for (const auto & display_object : environment->get_display_list())
            {
                ros_worker.add_node(display_object);
            }

            /// Assemble the UAV
            auto uav = std::make_shared<shapes::UAV>();
            for (const auto & display_object : uav->get_display_list())
            {
                ros_worker.add_node(display_object);
            }

            // Create a sphere
            auto [sphere, sphere_display] = display::make_sphere();
            // Create the cargo to be held by the UAV
            auto cargo = std::make_shared<shapes::Cargo>(sphere, sphere_display);
            ros_worker.add_node(cargo->get_display_list().back());

            auto previous_time = std::chrono::steady_clock::now();

            // Periodically do some work
            while (rclcpp::ok())
            {
                auto current_time = std::chrono::steady_clock::now();

                auto dt = std::chrono::duration_cast<std::chrono::duration<
                    double>>(current_time - previous_time).count();
            
                if (current_time - previous_time > 0.05s)
                {
                    // Update the state of the UAV
                    cargo->tick(*uav);

                    // Dropped objects fall except for last object held by UAV
                    for (auto & shape : cargo->get_cargo_list())
                    {
                        gravity(*shape);
                    }

                    // Check state flags
                    if (is_dropped)
                    {
                        cargo->drop_shape();
                        ros_worker.add_node(cargo->get_display_list().back());
                        is_dropped = !(DROPPED);
                    }
                    else if (is_cleared)
                    {
                        // Make the initial sphere invisible because I ran out of time
                        sphere->set_transparency(0);
                        cargo->clear_shapes();
                        
                        assert(cargo->get_display_list().empty());
                        assert(cargo->get_cargo_list().empty());
                        is_cleared = !(CLEARED);
                    }

                    // Update the time
                    previous_time = current_time;
                }

                // Run the executor, nodes take turns working.
                ros_worker.spin_once();
            }
        }
    }
    catch (std::exception & e)
    {
        // Something wrong occured, printing error message.
        std::cerr << "Error message:" << e.what() << "\n";
    }

    rclcpp::shutdown(); // Cleaning up before exiting.

    return 0;
}
