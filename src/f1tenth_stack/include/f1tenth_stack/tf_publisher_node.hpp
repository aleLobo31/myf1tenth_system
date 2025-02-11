#ifndef TF_PUBLISHER_NODE_HPP_
#define TF_PUBLISHER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <string>

class TFPublisherNode : public rclcpp::Node {
public:
    TFPublisherNode();

private:
    // Parameters
    std::string base_link_frame_;
    std::string laser_frame_;
    double laser_x_;
    double laser_y_;
    double laser_z_;
    double laser_roll_;
    double laser_pitch_;
    double laser_yaw_;

    // TF broadcaster
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;

    // Timer callback
    void timerCallback();
};

#endif // TF_PUBLISHER_NODE_HPP_ 