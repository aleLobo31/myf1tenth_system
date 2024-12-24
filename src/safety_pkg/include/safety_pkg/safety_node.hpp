#ifndef SAFETY_NODE_HPP  // Header guard to prevent multiple inclusion
#define SAFETY_NODE_HPP

// ROS2 core functionality
#include <rclcpp/rclcpp.hpp>

// ROS2 message types we'll be using
#include <sensor_msgs/msg/laser_scan.hpp>      // For receiving laser scan data
#include <nav_msgs/msg/odometry.hpp>           // For receiving vehicle position/velocity 
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>  // For sending drive commands
#include "safety_pkg/ttc_calculator.hpp"       // Our custom Time-To-Collision calculator

// SafetyNode inherits from the ROS2 Node class to create a ROS2 node
class SafetyNode : public rclcpp::Node {
public:
    // Constructor - will initialize the node and set up publishers/subscribers
    SafetyNode();

private:
    // Time-To-Collision calculator instance
    // unique_ptr for automatic memory management
    std::unique_ptr<TTCCalculator> ttc_calculator_;
    
    // ROS2 Communication channels:
    
    // Publisher to send brake commands to the vehicle
    // Uses AckermannDriveStamped message type for vehicle control
    // Publisher that sends AckermannDriveStamped messages to control the vehicle's motion
    // - rclcpp::Publisher: ROS2 publisher class for sending messages
    // - <ackermann_msgs::msg::AckermannDriveStamped>: Message type for Ackermann steering commands
    // - ::SharedPtr: Smart pointer that allows shared ownership of the publisher
    // - brake_publisher_: Member variable storing the publisher instance
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr brake_publisher_;

    
    // Subscriber to receive laser scan data from sensors
    // LaserScan contains distance measurements in a planar sweep
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
    
    // Subscriber to receive odometry data (position/velocity) from vehicle
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;

    // Stores the current velocity of the vehicle
    double current_velocity_;

    // Callback functions that will be triggered when messages arrive:
    
    // Called when new laser scan data arrives
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg);
    
    // Called when new odometry data arrives
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    
    // Helper function to send brake commands
    void applyBrake();
};

#endif
