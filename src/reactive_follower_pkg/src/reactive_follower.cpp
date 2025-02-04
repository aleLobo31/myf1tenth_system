#include "reactive_follower_pkg/reactive_follower_node.hpp"

ReactiveFollowerNode::ReactiveFollowerNode() : Node("reactive_follower") {
    // Declare and retrieve parameters
    this->declare_parameter("lidarscan_topic", "/scan");
    this->declare_parameter("drive_topic", "/drive");
//    this->declare_parameter("lidarscan_filtered_topic","/scan_filtered");
    this->declare_parameter("bubble_radius", 3.0);
    this->declare_parameter("max_speed", 5.0);
    this->declare_parameter("min_speed", 0.2);
    this->declare_parameter("lidar_angle",135.0);
    this->declare_parameter("max_lidar_distance", 12.0);
    this->declare_parameter("smoothing_window_size", 5);

    lidarscan_topic = this->get_parameter("lidarscan_topic").as_string();
    drive_topic = this->get_parameter("drive_topic").as_string();
//    lidarscan_filtered_topic = this->get_parameter("lidarscan_filtered_topic").as_string();
    bubble_radius = this->get_parameter("bubble_radius").as_double();
    max_speed = this->get_parameter("max_speed").as_double();
    min_speed = this->get_parameter("min_speed").as_double();
    lidar_angle = this->get_parameter("lidar_angle").as_double();
    max_lidar_distance = this->get_parameter("max_lidar_distance").as_double();
    smoothing_window_size = this->get_parameter("smoothing_window_size").as_int();

    // Initialize subscribers and publishers
    lidar_subscriber_ = create_subscription<sensor_msgs::msg::LaserScan>(
        lidarscan_topic, 10, std::bind(&ReactiveFollowerNode::lidar_callback, this, std::placeholders::_1));

    drive_publisher_ =  create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic, 10);
//    lidar_filtered_publisher_ = create_publisher<sensor_msgs::msg::LaserScan>(lidarscan_filtered_topic, 10);

    start_angle = (270 - (lidar_angle / 2) ) * (M_PI / 180.0);  // Convert degrees to radians
    end_angle = (270 + (lidar_angle / 2) ) * (M_PI / 180.0);

    start_index = std::max(0, std::min(449, static_cast<int>(start_angle / ((360.0 / 450) * (M_PI / 180.0)))));
    end_index = std::max(0, std::min(449, static_cast<int>(end_angle / ((360.0 / 450) * (M_PI / 180.0)))));

    RCLCPP_INFO(get_logger(), "Reactive follower initialized");
}
void ReactiveFollowerNode::preprocess_lidar(std::vector<float> &ranges) {
    // Set all ranges to NaN (ignoring irrelevant values)
    std::vector<float> cropped_ranges(end_index - start_index + 1, std::numeric_limits<float>::quiet_NaN());

    for (int i = start_index; i <= end_index; ++i) {
        if (ranges[i] > max_lidar_distance){
            cropped_ranges[i - start_index] = std::numeric_limits<float>::quiet_NaN();
        } else {
            cropped_ranges[i - start_index] = ranges[i];
        }
    }

    std::vector<float> smoothed_ranges(end_index - start_index + 1, std::numeric_limits<float>::quiet_NaN());

    for (size_t i = 0; i < cropped_ranges.size(); ++i) {
        int count = 0;
        float sum = 0.0;
        for (int j = -smoothing_window_size / 2; j <= smoothing_window_size / 2; ++j) {
            int idx = std::clamp(static_cast<int>(i) + j, 0, static_cast<int>(cropped_ranges.size() - 1));
            if (ranges[idx] <= max_lidar_distance) {
                sum += ranges[idx];
                ++count;
            }
        }
        smoothed_ranges[i] = (count > 0) ? sum / count : std::numeric_limits<float>::quiet_NaN();
    }
    ranges = smoothed_ranges;

//    auto filtered_scan_msg = *scan_msg;
//    filtered_scan_msg.ranges = ranges;
//    lidar_filtered_publisher_->publish(diltered_scan_msg);
}
size_t ReactiveFollowerNode::find_closest_point(const std::vector<float> &ranges) const {

    size_t min_index = 0;
    float min_value = std::numeric_limits<float>::max();  // Start with the largest possible value

    for (size_t i = 0; i < ranges.size(); i++) {
        if (ranges[i] > 0 && ranges[i] < min_value) {  // Ignore zero and negative values
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

std::pair<size_t, size_t> ReactiveFollowerNode::find_max_gap(const std::vector<float> &ranges) const {
    size_t max_start = 0, max_end = 0, current_start = 0;
    size_t max_length = 0, current_length = 0;

    for (size_t i = 0; i < ranges.size(); ++i) {
        if (ranges[i] > 0) {
            if (current_length == 0) {
                current_start = i;
            }
            ++current_length;
        } else {
            if (current_length > max_length) {
                max_length = current_length;
                max_start = current_start;
                max_end = i - 1;
            }
            current_length = 0;
        }
    }
if (current_length > max_length) {
        max_start = current_start;
        max_end = ranges.size() - 1;
    }

    return {max_start, max_end};
}

size_t ReactiveFollowerNode::find_best_point(const std::vector<float> &ranges, size_t gap_start, size_t gap_end) const {
    if (gap_start > gap_end) return gap_start;  // Default to start
    return std::distance(ranges.begin() + gap_start, std::max_element(ranges.begin() + gap_start, ranges.begin() + gap_end + 1)) + gap_start;
//     return ((gap_start + gap_end)/2);
}

void ReactiveFollowerNode::lidar_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) {

    std::vector<float> ranges = scan_msg->ranges;

    preprocess_lidar(ranges);
//    publish_filtered_lidar(scan_msg, ranges);  // Publish the filtered LiDAR data

    size_t closest_idx = find_closest_point(ranges);
    eliminate_bubble(ranges, closest_idx, bubble_radius);
    auto [gap_start, gap_end] = find_max_gap(ranges);
    size_t best_idx = find_best_point(ranges, gap_start, gap_end);

//    float angle_to_goal = (start_angle) + (best_idx * scan_msg->angle_increment);
    float best_angle = (best_idx) * ((360.0 / 450) * (M_PI / 180.0));
    float steering_angle = best_angle - ((3 * M_PI)/2 - start_angle);

    RCLCPP_INFO(get_logger(), "Start angle: %f", ranges[start_index]);
    RCLCPP_INFO(get_logger(), "Mid angle: %f", ranges[start_index + (end_index - start_index)/2]);
    RCLCPP_INFO(get_logger(), "End angle: %f", ranges[end_index]);

    RCLCPP_INFO(get_logger(), "Start index: %i", start_index);
    RCLCPP_INFO(get_logger(), "End index: %i", end_index);

    RCLCPP_INFO(get_logger(), "close index: %zu", closest_idx);

    RCLCPP_INFO(get_logger(), "gap start: %zu", gap_start);
    RCLCPP_INFO(get_logger(), "gap end: %zu", gap_end);


    RCLCPP_INFO(get_logger(), "Best index: %zu", best_idx);
    RCLCPP_INFO(get_logger(), "best angle: %f", best_angle);

    RCLCPP_INFO(get_logger(), "Steering angle: %f", steering_angle);

//    float gap_distance = ranges[best_idx];
//    float speed = std::clamp(gap_distance / 2.0, min_speed, max_speed);
    float speed = min_speed;

    auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
    drive_msg.drive.speed = speed;
    drive_msg.drive.steering_angle = steering_angle;
    drive_publisher_->publish(drive_msg);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ReactiveFollowerNode>());
    rclcpp::shutdown();
    return 0;
}

