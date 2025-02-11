#include "reactive_follower_pkg/reactive_follower_node.hpp"

ReactiveFollowerNode::ReactiveFollowerNode() : Node("reactive_follower") {
    
    // Declare and retrieve parameters
    this->declare_parameter("lidarscan_topic", "/scan");
    this->declare_parameter("drive_topic", "/drive");
    this->declare_parameter("bubble_radius", 15);
    this->declare_parameter("max_speed", 0.2);
    this->declare_parameter("min_speed", 0.2);
    this->declare_parameter("lidar_angle", 135.0);
    this->declare_parameter("max_lidar_distance", 12.0);
    this->declare_parameter("weight_speed", 0.5);
    this->declare_parameter("weight_steering", 0.5);

    lidarscan_topic = this->get_parameter("lidarscan_topic").as_string();
    drive_topic = this->get_parameter("drive_topic").as_string();
    bubble_radius = this->get_parameter("bubble_radius").as_int();
    max_speed = this->get_parameter("max_speed").as_double();
    min_speed = this->get_parameter("min_speed").as_double();
    lidar_angle = this->get_parameter("lidar_angle").as_double();
    max_lidar_distance = this->get_parameter("max_lidar_distance").as_double();
    weight_speed = this->get_parameter("weight_speed").as_double();
    weight_steering = this->get_parameter("weight_steering").as_double();

    // Initialize subscribers and publishers
    lidar_subscriber_ = create_subscription<sensor_msgs::msg::LaserScan>(
        lidarscan_topic, 10, std::bind(&ReactiveFollowerNode::lidar_callback, this, std::placeholders::_1));

    drive_publisher_ =  create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic, 10);

    start_angle = (270 - (lidar_angle / 2) ) * (M_PI / 180.0);  // Convert degrees to radians
    end_angle = (270 + (lidar_angle / 2) ) * (M_PI / 180.0);

    start_index = std::max(0, std::min(449, static_cast<int>(start_angle / ((360.0 / 450) * (M_PI / 180.0)))));
    end_index = std::max(0, std::min(449, static_cast<int>(end_angle / ((360.0 / 450) * (M_PI / 180.0)))));

    RCLCPP_INFO(get_logger(), "<lidarscan_topic>: %s", lidarscan_topic.c_str());
    RCLCPP_INFO(get_logger(), "<drive_topic>: %s", drive_topic.c_str());
    RCLCPP_INFO(get_logger(), "<bubble_radius>: %d", bubble_radius);
    RCLCPP_INFO(get_logger(), "<max_speed>: %f", max_speed);
    RCLCPP_INFO(get_logger(), "<min_speed>: %f", min_speed);
    RCLCPP_INFO(get_logger(), "<lidar_angle>: %f", lidar_angle);
    RCLCPP_INFO(get_logger(), "<max_lidar_distance>: %f", max_lidar_distance);
    RCLCPP_INFO(get_logger(), "<weight_speed>: %f", weight_speed);
    RCLCPP_INFO(get_logger(), "<weight_steering>: %f", weight_steering);

    RCLCPP_INFO(get_logger(), "Reactive follower initialized");
}

void ReactiveFollowerNode::preprocess_lidar(std::vector<float> &ranges) {
    
    float range = 0.0;
    float last_range = 0.0;

    // Simply filter out readings beyond max distance
    for (size_t i = 0; i < ranges.size(); i++) 
    {
        range = ranges[i];

        if(std::isnan(range))
        {
            ranges[i] = last_range;
        } else if (range > max_lidar_distance) {
            ranges[i] = 0.0;
        } else
        {
            last_range = range;
        }
        
    }
}

size_t ReactiveFollowerNode::find_closest_point(const std::vector<float> &ranges) {
    size_t min_index = 0;
    float min_value = std::numeric_limits<float>::max();

    for (size_t i = 0; i < ranges.size(); i++) {
        if (ranges[i] > 0.0 && ranges[i] < min_value) {
            min_value = ranges[i];
            min_index = i;
        }
    }
    return min_index;
}

void ReactiveFollowerNode::eliminate_bubble(std::vector<float> &ranges, size_t closest_idx, float bubble_radius) {
    size_t bubble_start = (closest_idx >= static_cast<size_t>(bubble_radius)) ? closest_idx - static_cast<size_t>(bubble_radius) : 0;
    size_t bubble_end = std::min(closest_idx + static_cast<size_t>(bubble_radius), ranges.size() - 1);
    std::fill(ranges.begin() + bubble_start, ranges.begin() + bubble_end + 1, 0.0);
}

std::pair<size_t, size_t> ReactiveFollowerNode::find_max_gap(const std::vector<float> &ranges) {
    size_t min_gap = 0;
    size_t longest_gap = 0;
    size_t curr_gap = 0;
    for (size_t i = 0; i < ranges.size(); i++) {
        if (ranges[i] < 0.5) {
            curr_gap = 0;
        } else {
            curr_gap++;
            if (curr_gap > longest_gap) {  // Update the largest gap
                min_gap = i - curr_gap;
                longest_gap = curr_gap;
            }
        }
    }
    size_t max_gap = min_gap + longest_gap;
    return std::make_pair(min_gap, max_gap);
}

size_t ReactiveFollowerNode::find_best_point(const std::vector<float> &ranges, size_t gap_start, size_t gap_end) {
    if (gap_start > gap_end) return gap_start;
    
    // Find furthest point in gap
    size_t best_idx = gap_start;
    float max_range = ranges[gap_start];
    
    for (size_t i = gap_start; i <= gap_end; i++) {
        if (ranges[i] > max_range) {
            max_range = ranges[i];
            best_idx = i;
        }
    }
    
//    return best_idx;

  size_t best_aux = (gap_start + gap_end)/2;
  return best_aux;


}

void ReactiveFollowerNode::lidar_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) {
    std::vector<float> ranges = scan_msg->ranges;
    std::vector<float> cropped_ranges(end_index - start_index + 1);
    
    // Get only the front section
    for (size_t i = 0; i < cropped_ranges.size(); ++i) {
        cropped_ranges[i] = ranges[i + start_index];
    }

    //preprocess_lidar(cropped_ranges);
    size_t closest_idx = find_closest_point(cropped_ranges);
    eliminate_bubble(cropped_ranges, closest_idx, bubble_radius);

    auto [gap_start, gap_end] = find_max_gap(cropped_ranges);
    size_t best_idx = find_best_point(cropped_ranges, gap_start, gap_end);

    // Convert back to full scan index for angle calculation
    size_t full_best_idx = best_idx + start_index;
    float best_angle = (full_best_idx) * ((360.0 / 450) * (M_PI / 180.0));
    float steering_angle = - (best_angle - ((3 * M_PI)/2));

    RCLCPP_INFO(get_logger(), "close index: %zu", closest_idx);
    RCLCPP_INFO(get_logger(), "gap start: %zu", gap_start);
    RCLCPP_INFO(get_logger(), "gap end: %zu", gap_end);
    RCLCPP_INFO(get_logger(), "Steering angle: %f", steering_angle);

    auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
    double spd = std::min(std::abs(max_speed * (weight_speed * ranges[full_best_idx] - weight_steering * std::abs(steering_angle))), max_speed);
    drive_msg.drive.speed = std::max(spd, min_speed);
    drive_msg.drive.steering_angle = steering_angle;
    drive_publisher_->publish(drive_msg);
}
