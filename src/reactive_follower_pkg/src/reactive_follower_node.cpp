#include "reactive_follower_pkg/reactive_follower_node.hpp"

ReactiveFollowerNode::ReactiveFollowerNode() : Node("reactive_follower") {
    latest_scan_msg_ = nullptr;
    // Declare and retrieve parameters
    this->declare_parameter("lidarscan_topic", "/scan");
    this->declare_parameter("drive_topic", "/drive");
    this->declare_parameter("goalpoint_topic", "/goalpoint");
    this->declare_parameter("laser_frame", "laser");
    this->declare_parameter("car_frame", "base_link");
    this->declare_parameter("bubble_radius", 15);
    this->declare_parameter("max_speed", 0.2);
    this->declare_parameter("min_speed", 0.2);
    this->declare_parameter("lidar_angle", 135.0);
    this->declare_parameter("max_lidar_distance", 12.0);
    this->declare_parameter("weight_speed", 0.5);
    this->declare_parameter("weight_steering", 0.5);

    lidarscan_topic = this->get_parameter("lidarscan_topic").as_string();
    drive_topic = this->get_parameter("drive_topic").as_string();
    goalpoint_topic = this->get_parameter("goalpoint_topic").as_string();
    laser_frame = this->get_parameter("laser_frame").as_string();
    car_frame = this->get_parameter("car_frame").as_string();
    bubble_radius = this->get_parameter("bubble_radius").as_int();
    max_speed = this->get_parameter("max_speed").as_double();
    min_speed = this->get_parameter("min_speed").as_double();
    lidar_angle = this->get_parameter("lidar_angle").as_double();
    max_lidar_distance = this->get_parameter("max_lidar_distance").as_double();
    weight_speed = this->get_parameter("weight_speed").as_double();
    weight_steering = this->get_parameter("weight_steering").as_double();

    // Initialize subscribers and publishers
    lidar_subscriber_ = create_subscription<sensor_msgs::msg::LaserScan>(
        lidarscan_topic, 10, 
        std::bind(&ReactiveFollowerNode::lidar_callback, this, std::placeholders::_1));

    goal_subscriber_ = create_subscription<interfaces_pkg::msg::GoalPoint>(
        goalpoint_topic, 10, 
        std::bind(&ReactiveFollowerNode::goal_callback, this, std::placeholders::_1));

    drive_publisher_ =  create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic, 10);

    start_angle = (270 - (lidar_angle / 2) ) * (M_PI / 180.0);  // Convert degrees to radians
    end_angle = (270 + (lidar_angle / 2) ) * (M_PI / 180.0);

    start_index = std::max(0, std::min(449, static_cast<int>(start_angle / ((360.0 / 450) * (M_PI / 180.0)))));
    end_index = std::max(0, std::min(449, static_cast<int>(end_angle / ((360.0 / 450) * (M_PI / 180.0)))));

    // Initialize transform buffer and listener
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    RCLCPP_INFO(get_logger(), "<lidarscan_topic>: %s", lidarscan_topic.c_str());
    RCLCPP_INFO(get_logger(), "<drive_topic>: %s", drive_topic.c_str());
    RCLCPP_INFO(get_logger(), "<goalpoint_topic>: %s", goalpoint_topic.c_str());
    RCLCPP_INFO(get_logger(), "<laser_frame>: %s", laser_frame.c_str());
    RCLCPP_INFO(get_logger(), "<car_frame>: %s", car_frame.c_str());
    RCLCPP_INFO(get_logger(), "<bubble_radius>: %d", bubble_radius);
    RCLCPP_INFO(get_logger(), "<max_speed>: %f", max_speed);
    RCLCPP_INFO(get_logger(), "<min_speed>: %f", min_speed);
    RCLCPP_INFO(get_logger(), "<lidar_angle>: %f", lidar_angle);
    RCLCPP_INFO(get_logger(), "<max_lidar_distance>: %f", max_lidar_distance);
    RCLCPP_INFO(get_logger(), "<weight_speed>: %f", weight_speed);
    RCLCPP_INFO(get_logger(), "<weight_steering>: %f", weight_steering);

    RCLCPP_INFO(get_logger(), "Reactive follower initialized");
}

void ReactiveFollowerNode::lidar_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr& scan_msg) {
    latest_scan_msg_ = scan_msg;
}

void ReactiveFollowerNode::preprocess_lidar(std::vector<float> &ranges) {
    
    float range = 0.0;
    float last_range = 0.0;

    // Filter out readings beyond max distance
    // Nan reading get the last valid measure
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

// returns safety distance based on speed
double ReactiveFollowerNode::calculate_safety_distance(double speed){
    if (speed < 0.7) {
        return 0.4; // Safety distance for low speeds
    } else {
        return 0.4 + (speed - 0.7) * 0.3; // Proportional increase for higher speeds
    }
}
    // Calculate the minimum number of LiDAR beams for a safe gap
size_t ReactiveFollowerNode::calculate_min_gap_size(double safety_distance) {
    double alpha = 2*(atan2((0.3 / 2) / safety_distance)); //geometry cacl
    size_t min_gap = static_cast<size_t>(std::ceil((alpha * 450) / (2 * M_PI))); // Number of scans in alpha radians (450 scans in 360º)
    return min_gap;
}

int ReactiveFollowerNode::point_to_lidar_index(){
    
    double x = goal_msg_->x;
    double y = goal_msg_->y;
    try {
        // Get car's goal in laser frame
        current_transform_ = tf_buffer_->lookupTransform(
            laser_frame,          // target frame
            car_frame,          // source frame
            tf2::TimePointZero, 
            std::chrono::milliseconds(100)
        );
    } catch (const tf2::TransformException &ex) {
        RCLCPP_WARN(
            this->get_logger(),
            "Failed to get %s → %s transform: %s",
            laser_frame.c_str(), car_frame.c_str(), ex.what()
        );
        return;
    }
    x = current_transform_.transform.translation.x;
    y = current_transform_.transform.translation.y;
        // Calculate the angle from the x and y coordinates
    double angle = std::atan2(y, x); // angle in radians, CCW from x-axis

    if (angle < 0) angle += 2*M_PI;
    int idx = std::round(angle * 450.0/(2*M_PI));
    return std::clamp(idx, 0, 449);
}    

std::vector<Gap> ReactiveFollowerNode::find_gaps(const std::vector<float> &ranges, size_t min_gap, double safety_distance) {
    // Find gaps in the LiDAR scan data based on the safety distance and minimum gap size
    // Return a vector of Gap structs containing start and end indices of each gap
    std::vector<Gap> gaps;
    bool in_gap = false;
    size_t start = 0;
    for (size_t i = 0; i < ranges.size(); ++i) {
        if (ranges[i] > safety_distance) {
            if (!in_gap) { start = i; in_gap = true; }
        } else if (in_gap) {
            size_t end = i-1;
            if (end - start +1 >= min_gap) gaps.push_back({start,end});
            in_gap = false;
        }
    }
    if (in_gap) {
        size_t end = ranges.size()-1;
        if (end - start +1 >= min_gap) gaps.push_back({start,end});
    }
    return gaps;

}

bool ReactiveFollowerNode::gp_in_gaps(const std::vector<Gap> &gaps){
    //Si indice_pp esta dentro de un gap, return true. si no está en ninguno, return false
    for (auto &gap : gaps) {
        if (gp_index >= static_cast<int>(gap.start) && gp_index <= static_cast<int>(gap.end)) {
            return true;
        }
    }
    return false;
}

std::pair<double, double> ReactiveFollowerNode::alternative_commands(
    const std::vector<Gap>& gaps, 
    const std::vector<float>& ranges, 
    int gp_index) 
{
    // Find the gap edge (start or end) closest to gp_index
    int best_idx = -1;
    int min_dist = std::numeric_limits<int>::max();
    for (const auto& gap : gaps) {
        int dist_start = std::abs(static_cast<int>(gap.start) - gp_index);
        int dist_end = std::abs(static_cast<int>(gap.end) - gp_index);
        if (dist_start < min_dist) {
            min_dist = dist_start;
            best_idx = gap.start;
        }
        if (dist_end < min_dist) {
            min_dist = dist_end;
            best_idx = gap.end;
        }
    }
    if (best_idx == -1) best_idx = 0; // fallback

    double best_angle = (best_idx) * ((360.0 / 450) * (M_PI / 180.0));
    double steering_angle = - (best_angle - ((3 * M_PI)/2));
    double speed = std::min(std::abs(max_speed * (weight_speed * ranges[best_idx] - weight_steering * std::abs(steering_angle))), max_speed);

    return std::make_pair(steering_angle, speed);
}


void ReactiveFollowerNode::goal_callback(const interfaces_pkg::msg::GoalPoint::ConstSharedPtr& goal_msg) {

    goal_msg_ = goal_msg; // store

    if (!latest_scan_msg_) {
    RCLCPP_WARN(get_logger(), "No LiDAR data yet, skipping goal");
    return;
    }

    std::vector<float> ranges = latest_scan_msg_->ranges;
    std::vector<float> cropped_ranges(end_index - start_index + 1);
    if (!gp_in_gaps(gaps)) {
        auto [alt_steering_angle, alt_speed] = alternative_commands(gaps, cropped_ranges, gp_index);
        steering_angle = alt_steering_angle;
        speed = alt_speed;
        RCLCPP_INFO(get_logger(), "Alternative commands.");
    } else {
        RCLCPP_INFO(get_logger(), "Pure Pursuit commands.");
    }
    preprocess_lidar(cropped_ranges);
    size_t closest_idx = find_closest_point(cropped_ranges);
    eliminate_bubble(cropped_ranges, closest_idx, bubble_radius);
    
    double speed = goal_msg->v;
    double steering_angle = goal_msg->s;

    safety_distance = calculate_safety_distance(speed);
    min_gap_size = calculate_min_gap_size(safety_distance);
    gp_index = point_to_lidar_index();

    std::vector<Gap> gaps = find_gaps(cropped_ranges, min_gap_size, safety_distance);
    
    if (!gp_in_gaps(gaps)) {
        auto [steering_angle, speed] = alternative_commands(gaps);
        RCLCPP_INFO(get_logger(), "Alternative commands.");
    } else {
        RCLCPP_INFO(get_logger(), "Pure Pursuit commands.");
    }

    RCLCPP_INFO(get_logger(), "close index: %zu", closest_idx);
    RCLCPP_INFO(get_logger(), "Velocity: %f", speed);
    RCLCPP_INFO(get_logger(), "Steering angle: %f", steering_angle);

    auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
    drive_msg.drive.speed = std::max(speed, min_speed);
    drive_msg.drive.steering_angle = steering_angle;
    drive_publisher_->publish(drive_msg);
}
