#ifndef REACTIVE_FOLLOW_GAP_HPP
#define REACTIVE_FOLLOW_GAP_HPP

#include "rclcpp/rclcpp.hpp"
#include <string>
#include <vector>
#include <algorithm>
#include <limits>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

class ReactiveFollowGap : public rclcpp::Node {
public:
    ReactiveFollowGap();

private:  
    std::string lidarscan_topic;
    std::string drive_topic;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_subscriber_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_publisher_;

    float bubble_radius;
    float max_speed;
    float min_speed;
    float max_lidar_range;
    int smoothing_window_size;
    
    void preprocess_lidar(std::vector<float> &ranges);
    size_t find_closest_point(const std::vector<float> &ranges) const;
    void eliminate_bubble(std::vector<float> &ranges, size_t closest_idx, float bubble_radius) ;
    std::pair<size_t, size_t> find_max_gap(const std::vector<float> &ranges) const;
    size_t find_best_point(const std::vector<float> &ranges, size_t gap_start, size_t gap_end) const;
    void lidar_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) ;
};

#endif // REACTIVE_FOLLOW_GAP_HPP
