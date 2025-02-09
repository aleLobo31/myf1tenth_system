#ifndef REACTIVE_FOLLOWER_NODE_HPP_
#define REACTIVE_FOLLOWER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <vector>
#include <utility>
#include <memory>

class ReactiveFollowerNode : public rclcpp::Node {
public:
    ReactiveFollowerNode();

private:
    // ROS Parameters
    std::string lidarscan_topic;
    std::string drive_topic;
    int bubble_radius;
    double max_speed;
    double min_speed;
    double lidar_angle;
    double max_lidar_distance;
    double weight_speed;
    double weight_steering;

    // Scan indices and angles
    size_t start_index;
    size_t end_index;
    double start_angle;
    double end_angle;

    // ROS communication
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_subscriber_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_publisher_;

    // LiDAR processing methods
    void preprocess_lidar(std::vector<float> &ranges);
    size_t find_closest_point(const std::vector<float> &ranges);
    void eliminate_bubble(std::vector<float> &ranges, size_t closest_idx, float bubble_radius);
    std::pair<size_t, size_t> find_max_gap(const std::vector<float> &ranges);
    size_t find_best_point(const std::vector<float> &ranges, size_t gap_start, size_t gap_end);

    // Callback
    void lidar_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg);
};

#endif //REACTIVE_FOLLOWER_NODE_HPP_