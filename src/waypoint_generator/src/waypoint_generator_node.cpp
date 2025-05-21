#include "waypoint_generator_node.hpp"


WayPointGenerator::WayPointGenerator() : Node("waypoint_generator_node")
{
    this->declare_parameter("csv_path", "f1tenth_ws/src/pure_pursuit/racelines/waypoints_odom_v0.csv");
    this->declare_parameter("odom_topic", "/ego_racecar/odom");
    this->declare_parameter("min_distance", 0.5);
    this->declare_parameter("prev_x", 0.0);
    this->declare_parameter("prev_y", 0.0);

    csv_path = this-> get_parameter("csv_path").as_string();
    odom_topic = this->get_parameter("odom_topic").as_string();
    min_distance = this->get_parameter("min_distance").as_double();
    prev_x = this->get_parameter("prev_x").as_double();
    prev_y = this->get_parameter("prev_y").as_double();
    RCLCPP_INFO(this->get_logger(), "Waypoint Generator Node has started.");
    RCLCPP_INFO(this->get_logger(), "CSV Path: %s", csv_path.c_str());
    RCLCPP_INFO(this->get_logger(), "Odom Topic: %s", odom_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "Minimum Distance: %f", min_distance);
    RCLCPP_INFO(this->get_logger(), "Previous X: %f", prev_x);
    RCLCPP_INFO(this->get_logger(), "Previous Y: %f", prev_y);

    odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(odom_topic, 100, std::bind(&WayPointGenerator::odom_callback, this, std::placeholders::_1));
}

void WayPointGenerator::odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg)
{
    // Check whether the points are apart enough
    double diff = sqrt(pow((odom_msg->pose.pose.position.x - prev_x), 2) + pow((odom_msg->pose.pose.position.y - prev_y), 2));  
    RCLCPP_INFO(this->get_logger(), "Waypoint iteration.");

    if(diff >= min_distance)
    {
        // Open csv
csv_odom.open(csv_path, std::ios::out | std::ios::app);
    if (!csv_odom.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open CSV file at path: %s", csv_path.c_str());
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Saving waypoint to file: %s", csv_path.c_str());

        // Save the new point (x, y, theta, velocity, arc_length, curvature)
        csv_odom << "\n" << odom_msg->pose.pose.position.x << ", " << odom_msg->pose.pose.position.y;

        // Update the prev point (x, y)
        prev_x = odom_msg->pose.pose.position.x;
        prev_y = odom_msg->pose.pose.position.y;

        // Close csv
        csv_odom.close();
    }    
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node_ptr = std::make_shared<WayPointGenerator>();
    rclcpp::spin(node_ptr);
    rclcpp::shutdown();
    return 0;
}