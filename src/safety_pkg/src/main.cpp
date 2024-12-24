// Include the header file that defines our SafetyNode class
#include "safety_pkg/safety_node.hpp"

// Main entry point of the program
// argc: Number of command line arguments
// argv: Array of command line argument strings
int main(int argc, char** argv) {
    // Initialize the ROS2 system
    // This sets up communication infrastructure and must be called before any other ROS2 operations
    rclcpp::init(argc, argv);

    // Create a shared pointer to our SafetyNode
    // std::make_shared automatically manages memory and creates a reference-counted pointer
    // SafetyNode will handle all our safety-critical functionality like TTC calculations
    auto node = std::make_shared<SafetyNode>();

    // Start processing data
    // rclcpp::spin() runs the node's event loop, handling:
    // - Incoming laser scan data through scan_subscriber_
    // - Incoming odometry data through odom_subscriber_ 
    // - Publishing brake commands when needed through brake_publisher_
    // This call blocks until the node is shut down
    rclcpp::spin(node);

    // Clean up ROS2 resources
    // This ensures proper shutdown of all ROS2 systems, including:
    // - Stopping all publishers and subscribers
    // - Cleaning up memory
    // - Terminating communications
    rclcpp::shutdown();

    // Return success code to operating system
    return 0;
}
