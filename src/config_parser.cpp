// Copyright 2019 Zhihao Zhang License MIT

// MTRN2500 Assignment 2 - Monday 3pm
// Completed By Dan Nguyen (z5206032)

#include "config_parser.hpp"

#include <algorithm>
#include <iostream>
#include <sstream>
#include <string>

namespace assignment2
{
ConfigReader::ConfigReader(std::istream & config_file)
{
    // Get length of file
    config_file.seekg(0, config_file.end);
    int config_file_length = config_file.tellg();
    config_file.seekg(0, config_file.beg);

    // Get buffer from config_file
    char * config_file_buffer = new char[config_file_length];
    config_file.read(config_file_buffer, config_file_length);

    // Cast buffer string as a istream
    std::istringstream config_file_stream(config_file_buffer);
    
    // Declare key and value variables
    std::string key;
    std::string value;
    
    // Declare other variables
    int line_number = 1;
    std::string line_text;
    
    // Get lines from buffer
    while (std::getline(config_file_stream, line_text))
    {
        // Empty line encountered
        if (line_text.empty()) {
            break;
        }

        // Print line to standard output
        std::cout << "Line " << line_number++ << ": " << line_text << "\n";

        // Remove all space from line
        line_text.erase(std::remove_if(line_text.begin(), line_text.end(),
            ::isspace), line_text.end());

        // Get key and value from line
        key = line_text.substr(0, line_text.find(':'));
        value = line_text.substr(line_text.find(':') + 1);
        
        // Store the key and value strings in config_
        config_[key] = value;
    }

    // After all lines have been read, iterate through config_
    for (auto token:config_)
    {
        // Print key and value pairs to standard output
        std::cout << "key: \"" << token.first << "\", " << "value: \""
            << token.second << "\"\n";
    }

    std::cout << "\n==========================================================\n\n";
}

/// find_config returns the value of the key if found and the default value
/// otherwise. refresh_rate is a special case where a reciprocal calculation
/// is required.
auto ConfigReader::find_config(std::string const & key,
    std::string const & default_value) const -> std::string
{
    if (!key.empty())
    {
        // refresh_rate key is special case as period is needed
        if (key.compare("refresh_rate") == 0)
        {
            // Convert value of key which is of type string to double
            double rate = std::stod(this->config_.at(key));

            // Return the period as a string
            return std::to_string(1/rate);
        }
        else
        {
            return this->config_.at(key);
        }
    }
    else
    {
        return default_value;
    }
}

ConfigParser::ConfigParser(ConfigReader const & config)
    : zid_
    {
        config.find_config(std::string{"zid"}, std::string{"z0000000"})
    }
    , refresh_period_
    {
        std::stol(config.find_config(std::string{"refresh_rate"},
            std::string{"100"}))
    }
    , joy_config_
    {
        // Members are implemented by order of definition
        std::stoul(config.find_config(std::string{"speed_plus_axis"},
            std::string{"1"})),
        std::stoul(config.find_config(std::string{"speed_minus_axis"},
            std::string{"-1"})),
        std::stoul(config.find_config(std::string{"steering_axis"},
            std::string{"0.5"})),
        std::stod(config.find_config(std::string{"steering_deadzone"},
            std::string{"0.1"})),
        std::stod(config.find_config(std::string{"speed_deadzone"},
            std::string{"0.1"}))
    }
    , kinematic_config_
    {
        // Members are implemented by order of definition
        std::stod(config.find_config(std::string{"max_linear_speed"},
            std::string{"10"})),
        std::stod(config.find_config(std::string{"max_angular_speed"},
            std::string{"10"})),
        std::stod(config.find_config(std::string{"max_linear_acceleration"},
            std::string{"10"})),
        std::stod(config.find_config(std::string{"max_angular_acceleration"},
            std::string{"10"}))
    }
{
}

auto ConfigParser::get_zid(void) const -> std::string
{
    return this->zid_;
}

auto ConfigParser::get_refresh_period(void) const -> std::chrono::milliseconds
{
    return this->refresh_period_;
}

auto ConfigParser::get_joystick_config(void) const -> JoystickConfig
{
    return this->joy_config_;
}

auto ConfigParser::get_kinematic_config(void) const -> KinematicLimits
{
    return this->kinematic_config_;
}
} // namespace assignment2
