#ifndef MANUAL_CONTROL_NODE_HPP_
#define MANUAL_CONTROL_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <std_msgs/msg/int8.hpp>
#include <fstream>

class ManualControlNode : public rclcpp::Node {
public:
    ManualControlNode();

private:
    // ROS Parameters
    std::string joy_topic_;
    std::string drive_topic_;
    std::string ackermann_cmd_topic_;
    
    int lb_button_idx_;
    int rb_button_idx_;
    int rt_axis_idx_;
    int lt_axis_idx_;
    int left_horizontal_axis_idx_;
    
    double throttle_gain_;
    double throttle_multiplier_;
    double steering_gain_;
    double steering_offset_;
    double constant_throttle_;
    double drive_multiplier_;
    
    // State variables
    bool button_pressed_;
    double prev_drive_multiplier_button_value_;
    int kill_button_prev_;

    // ROS communication
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr ackermann_pub_;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr enable_button_pub_;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr enable_button1_pub_;

    // Callbacks
    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy);
    void driveCallback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr drive);

    // Helper functions
    float linear_map(float x, float in_min, float in_max, float out_min, float out_max);
    //void setDS4LED(int red, int green, int blue);
};

#endif // MANUAL_CONTROL_NODE_HPP_
