#include "safety_pkg/safety_node.hpp"

// Constructor for the SafetyNode class that inherits from rclcpp::Node
SafetyNode::SafetyNode() : Node("safety_node"), current_velocity_(0.0) {
    // Create a TTC (Time To Collision) Calculator with 1.0 second threshold
    // This will help determine if we're too close to obstacles
    this->declare_parameter<double>("ttc_threshold", 0.6);
    this->declare_parameter<double>("restricted_min_angle", 3.14);
    this->declare_parameter<double>("restricted_max_angle", 6.28);
    // this->declare_parameter<double>("min_speed_threshold", 1.0);
    // this->declare_parameter<double>("max_brake_speed", 1.0);
    this->declare_parameter<std::string>("scan_topic", "/scan");
    this->declare_parameter<std::string>("odom_topic", "/odom");
    this->declare_parameter<std::string>("drive_topic", "/drive");

    double ttc_threshold = this->get_parameter("ttc_threshold").as_double();
    double restricted_min_angle = this->get_parameter("restricted_min_angle").as_double();
    double restricted_max_angle = this->get_parameter("restricted_max_angle").as_double();
    // min_speed_threshold = this->get_parameter("min_speed_threshold").as_double();
    // max_brake_speed = this->get_parameter("max_brake_speed").as_double(); 
    scan_topic = this->get_parameter("scan_topic").as_string();
    odom_topic = this->get_parameter("odom_topic").as_string();
    drive_topic = this->get_parameter("drive_topic").as_string();

    ttc_calculator_ = std::make_unique<TTCCalculator>(ttc_threshold, restricted_min_angle, restricted_max_angle);  // threshold = 1.0s
    
    // Create a publisher that will send brake commands
    // Uses AckermannDriveStamped messages on the "/drive" topic with queue size 10
    brake_publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
        drive_topic, 10);

    // Create a subscriber for laser scan data
    // Subscribes to LaserScan messages on "/scan" topic with queue size 10
    // When a message arrives, it calls scanCallback function
    scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        scan_topic, 10,
        // std::bind creates a function object that calls scanCallback when invoked
        // &SafetyNode::scanCallback - pointer to the member function to be called 
        // this - pointer to the SafetyNode instance on which to call the function
        // std::placeholders::_1 - represents the LaserScan message argument that will be passed
        std::bind(&SafetyNode::scanCallback, this, std::placeholders::_1));

    // Create a subscriber for odometry data
    // Subscribes to Odometry messages on "/ego_racecar/odom" topic with queue size 10
    // When a message arrives, it calls odomCallback function
    odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic, 10,
        std::bind(&SafetyNode::odomCallback, this, std::placeholders::_1));

         RCLCPP_INFO(this->get_logger(), "<scan_topic>: %s", scan_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "<odom_topic>: %s", odom_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "<drive_topic>: %s", drive_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "<ttc_threshold>: %f", ttc_threshold);
    RCLCPP_INFO(this->get_logger(), "<restricted_min_angle>: %f", restricted_min_angle);
    RCLCPP_INFO(this->get_logger(), "<restricted_max_angle>: %f", restricted_max_angle);
    
    RCLCPP_INFO(this->get_logger(), "Safety node initialized");
}

// Callback function that processes incoming laser scan data
void SafetyNode::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg) {
    // std::cout << "Angle_min = " << scan_msg->angle_min << "Angle_max = " << scan_msg->angle_max << std::endl;
    // If we're not moving, no need to check for collisions
    if (current_velocity_ <= 0) return;

    // Calculate the minimum Time To Collision using the scan data and current velocity
    // This processes all laser scan points to find the most critical collision risk
    double min_ttc = ttc_calculator_->calculateMinTTC(scan_msg, current_velocity_);

    // If the minimum TTC is less than our threshold (1.0s), we need to brake
    if (min_ttc < ttc_calculator_->getThreshold()) {
        applyBrake();
    }
}

// Callback function that processes incoming odometry data
void SafetyNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // Update our stored velocity with the absolute value of the current linear velocity
    // We use absolute value because collision risk exists whether moving forward or backward
    current_velocity_ = std::abs(msg->twist.twist.linear.x);
    
    // Additional data available in the odometry message that could be useful:
    // - Angular velocity (msg->twist.twist.angular.z) - for tracking rotation speed
    // - Position (msg->pose.pose.position) - for absolute position tracking
    // - Orientation (msg->pose.pose.orientation) - for direction tracking
    // - Velocity covariance (msg->twist.covariance) - for uncertainty in velocity measurements
    // These could be used to enhance safety features if needed
}

// Function to send a brake command
void SafetyNode::applyBrake() {
    // Create a new AckermannDriveStamped message
    auto brake_cmd = std::make_unique<ackermann_msgs::msg::AckermannDriveStamped>();
    // Set the speed to 0.0 to stop the vehicle
    brake_cmd->drive.speed = 0.0;
    // Publish the brake command
    brake_publisher_->publish(std::move(brake_cmd));

    RCLCPP_INFO(this->get_logger(), "ALTOOOOOO");
}
