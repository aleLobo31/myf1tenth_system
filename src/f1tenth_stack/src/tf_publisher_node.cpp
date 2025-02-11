#include "f1tenth_stack/tf_publisher_node.hpp"
#include <tf2/LinearMath/Quaternion.h>

TFPublisherNode::TFPublisherNode() : Node("tf_publisher_node") {
    // Declare and get parameters
    this->declare_parameter("base_link_frame", "base_link");
    this->declare_parameter("laser_frame", "laser");
    this->declare_parameter("laser_x", 0.0);
    this->declare_parameter("laser_y", 0.0);
    this->declare_parameter("laser_z", 0.0);
    this->declare_parameter("laser_roll", 0.0);
    this->declare_parameter("laser_pitch", 0.0);
    this->declare_parameter("laser_yaw", 0.0);

    base_link_frame_ = this->get_parameter("base_link_frame").as_string();
    laser_frame_ = this->get_parameter("laser_frame").as_string();
    laser_x_ = this->get_parameter("laser_x").as_double();
    laser_y_ = this->get_parameter("laser_y").as_double();
    laser_z_ = this->get_parameter("laser_z").as_double();
    laser_roll_ = this->get_parameter("laser_roll").as_double();
    laser_pitch_ = this->get_parameter("laser_pitch").as_double();
    laser_yaw_ = this->get_parameter("laser_yaw").as_double();

    // Initialize TF broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Create timer for periodic publishing
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&TFPublisherNode::timerCallback, this));

    RCLCPP_INFO(this->get_logger(), "TF Publisher Node has been initialized");
}

void TFPublisherNode::timerCallback() {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = this->get_clock()->now();
    transform.header.frame_id = base_link_frame_;
    transform.child_frame_id = laser_frame_;

    // Set translation
    transform.transform.translation.x = laser_x_;
    transform.transform.translation.y = laser_y_;
    transform.transform.translation.z = laser_z_;

    // Set rotation
    tf2::Quaternion q;
    q.setRPY(laser_roll_, laser_pitch_, laser_yaw_);
    transform.transform.rotation.x = q.x();
    transform.transform.rotation.y = q.y();
    transform.transform.rotation.z = q.z();
    transform.transform.rotation.w = q.w();

    // Send transform
    tf_broadcaster_->sendTransform(transform);
} 