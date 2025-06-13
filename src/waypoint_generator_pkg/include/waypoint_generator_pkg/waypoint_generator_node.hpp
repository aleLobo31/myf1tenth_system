#ifndef WAYPOINT_GENERATOR_NODE_HPP_
#define WAYPOINT_GENERATOR_NODE_HPP_

#include <fstream> // Required to work with csv files
#include <iostream>
#include <string>
#include <cmath>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"

class WayPointGenerator : public rclcpp::Node
{
    public:
        WayPointGenerator();

    private:
        // Core parameters
        std::string csv_path;
        double min_distance;
        double prev_x;
        double prev_y;
        std::ofstream csv_odom;
        std::string map_frame;
        std::string car_frame;

        // Transform handling
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        geometry_msgs::msg::TransformStamped current_transform_;

        rclcpp::TimerBase::SharedPtr timer_;
        void timer_callback();
};

#endif  // WAYPOINT_GENERATOR_NODE_HPP_
