#include <fstream> // Required to work with csv files
#include <iostream>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"

class WayPointGenerator : public rclcpp::Node
{
    public:
        WayPointGenerator();

    private:
        // Required Variables
        std::string csv_path; // csv file PATH
        std::string odom_topic; // Topic where CAR POSE is published
        double min_distance; // Minimum distance to save a point
        double prev_x; // Old point x coordinate
        double prev_y; // Old point y coordinate
        std::ofstream csv_odom; // Csv File

        // Required Objects
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;

        // Required Functions
        void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg);
};