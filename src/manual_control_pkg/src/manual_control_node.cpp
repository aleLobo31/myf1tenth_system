#include "manual_control_pkg/manual_control_node.hpp"

ManualControlNode::ManualControlNode() : Node("manual_control_node"){
    // Declare and retrieve parameters
    this->declare_parameter<int>("lb_button_idx", 4);
    this->declare_parameter<int>("rb_button_idx", 5);
    this->declare_parameter<int>("rt_axis_idx", 5);
    this->declare_parameter<int>("lt_axis_idx", 2);
    this->declare_parameter<int>("left_horizontal_axis_idx", 0);
    this->declare_parameter<std::string>("joy_topic", "/joy");
    this->declare_parameter<std::string>("drive_topic", "/drive");
    this->declare_parameter<std::string>("ackermann_cmd_topic", "/ackermann_cmd");
    this->declare_parameter<double>("throttle_gain", 2);
    this->declare_parameter<double>("throttle_multiplier", 3);
    this->declare_parameter<double>("steering_gain", -0.37);
    this->declare_parameter<double>("steering_offset", 0.0);
    this->declare_parameter<double>("constant_throttle", 1);

    // Get parameters
    lb_button_idx_ = this->get_parameter("lb_button_idx").as_int();
    rb_button_idx_ = this->get_parameter("rb_button_idx").as_int();
    rt_axis_idx_ = this->get_parameter("rt_axis_idx").as_int();
    lt_axis_idx_ = this->get_parameter("lt_axis_idx").as_int();
    left_horizontal_axis_idx_ = this->get_parameter("left_horizontal_axis_idx").as_int();
    joy_topic_ = this->get_parameter("joy_topic").as_string();
    drive_topic_ = this->get_parameter("drive_topic").as_string();
    ackermann_cmd_topic_ = this->get_parameter("ackermann_cmd_topic").as_string();
    throttle_gain_ = this->get_parameter("throttle_gain").as_double();
    throttle_multiplier_ = this->get_parameter("throttle_multiplier").as_double();
    steering_gain_ = this->get_parameter("steering_gain").as_double();
    steering_offset_ = this->get_parameter("steering_offset").as_double();
    constant_throttle_ = this->get_parameter("constant_throttle").as_double();

    // Initialize publishers and subscribers
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        joy_topic_, 10, std::bind(&ManualControlNode::joyCallback, this, std::placeholders::_1));
    drive_sub_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
        drive_topic_, 10, std::bind(&ManualControlNode::driveCallback, this, std::placeholders::_1));
    ackermann_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(ackermann_cmd_topic_, 10);
    enable_button_pub_ = this->create_publisher<std_msgs::msg::Int8>("/enable_0", 10);
    enable_button1_pub_ = this->create_publisher<std_msgs::msg::Int8>("/enable_1", 10);

    drive_multiplier_ = 1.0;

    RCLCPP_INFO(get_logger(), "Manual control node initialized");
}

float ManualControlNode::linear_map(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void ManualControlNode::joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy) {
    std_msgs::msg::Int8 enable_button_publish;
    enable_button_publish.data = joy->buttons[0];
    enable_button_pub_->publish(enable_button_publish);
    enable_button_publish.data = joy->buttons[1];
    enable_button1_pub_->publish(enable_button_publish);

    button_pressed_ = joy->buttons[lb_button_idx_];

    if (button_pressed_ && !joy->buttons[rb_button_idx_]) {
        system("ds4led green");
        return;
    } else {
        system("ds4led blue");
    }

    auto ackermann_msg = ackermann_msgs::msg::AckermannDriveStamped();
    ackermann_msg.header.stamp = this->now();
    ackermann_msg.header.frame_id = "base_link";

    // Map joystick axes to servo and throttle values
    ackermann_msg.drive.speed = linear_map(joy->axes[rt_axis_idx_], 1, -1, 0, 1) * 
        throttle_gain_ * (joy->buttons[rb_button_idx_] && joy->buttons[lb_button_idx_] ? throttle_multiplier_ : 1);
    
    if (joy->axes[lt_axis_idx_] != 1.0) {
        ackermann_msg.drive.speed = -linear_map(joy->axes[lt_axis_idx_], 1, -1, 0, 1) * 
            throttle_gain_ * (joy->buttons[rb_button_idx_] && joy->buttons[lb_button_idx_] ? throttle_multiplier_ : 1);
    }
    
    if (joy->buttons[rb_button_idx_]) {
        ackermann_msg.drive.speed = constant_throttle_;
    }

    ackermann_msg.drive.steering_angle = -joy->axes[left_horizontal_axis_idx_] * steering_gain_ + steering_offset_;
    ackermann_pub_->publish(ackermann_msg);

    if (joy->axes[7] == 1.0 && prev_drive_multiplier_button_value_ == 0.0) {
        drive_multiplier_ += 0.05;
        RCLCPP_INFO(this->get_logger(), "multiplier changed to %f", drive_multiplier_);
    } else if (joy->axes[7] == -1.0 && prev_drive_multiplier_button_value_ == 0.0) {
        drive_multiplier_ -= 0.05;
        RCLCPP_INFO(this->get_logger(), "multiplier changed to %f", drive_multiplier_);
    }
    prev_drive_multiplier_button_value_ = joy->axes[7];

    if (kill_button_prev_ == 0 && joy->buttons[1] == 1) {
        system("pkill async_slam_tool && pkill vesc_to_odom_node");
        RCLCPP_INFO(this->get_logger(), "Killed async_slam_tool and vesc_to_odom_node");
    }
    kill_button_prev_ = joy->buttons[1];
}

void ManualControlNode::driveCallback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr drive) {
    if (button_pressed_) {
        drive->drive.speed *= drive_multiplier_;
        ackermann_pub_->publish(*drive);
    }
}

