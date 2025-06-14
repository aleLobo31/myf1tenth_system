#include "waypoint_generator_pkg/waypoint_generator_node.hpp"

WayPointGenerator::WayPointGenerator()
: Node("waypoint_generator_node")
{
    this->declare_parameter("csv_path", "coche_ws/src/pure_pursuit_pkg/racelines/waypoints_odom_v1.csv");
    this->declare_parameter("min_distance", 0.5);
    this->declare_parameter("prev_x", 0.0);
    this->declare_parameter("prev_y", 0.0);
    this->declare_parameter("map_frame", "map");
    this->declare_parameter("car_frame", "base_link");

    csv_path = this->get_parameter("csv_path").as_string();
    min_distance = this->get_parameter("min_distance").as_double();
    prev_x = this->get_parameter("prev_x").as_double();
    prev_y = this->get_parameter("prev_y").as_double();
    map_frame = this->get_parameter("map_frame").as_string();
    car_frame = this->get_parameter("car_frame").as_string();
    
    RCLCPP_INFO(this->get_logger(), "Waypoint Generator Node has started.");
    RCLCPP_INFO(this->get_logger(), "CSV Path: %s", csv_path.c_str());
    RCLCPP_INFO(this->get_logger(), "Minimum Distance: %f", min_distance);
    RCLCPP_INFO(this->get_logger(), "Previous X: %f", prev_x);
    RCLCPP_INFO(this->get_logger(), "Previous Y: %f", prev_y);
    RCLCPP_INFO(this->get_logger(), "Map Frame: %s", map_frame.c_str());
    RCLCPP_INFO(this->get_logger(), "Car Frame: %s", car_frame.c_str());

    // Initialize transform buffer and listener
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Create timer for periodic waypoint generation
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&WayPointGenerator::timer_callback, this)
    );
}

void WayPointGenerator::timer_callback()
{
    try {
        // Get car's pose in map frame
        current_transform_ = tf_buffer_->lookupTransform(
            map_frame,          // target frame
            car_frame,          // source frame
            tf2::TimePointZero, 
            std::chrono::milliseconds(100)
        );
    } catch (const tf2::TransformException &ex) {
        RCLCPP_WARN(
            this->get_logger(),
            "Failed to get %s → %s transform: %s",
            map_frame.c_str(), car_frame.c_str(), ex.what()
        );
        return;
    }

    // Extract current position
    double x = current_transform_.transform.translation.x;
    double y = current_transform_.transform.translation.y;
    double diff = sqrt(pow((x - prev_x), 2) + pow((y - prev_y), 2));

    if(diff >= min_distance)
    {
        csv_odom.open(csv_path, std::ios::out | std::ios::app);
        if (!csv_odom.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open CSV file at path: %s", csv_path.c_str());
            return;
        }
        csv_odom << "\n" << x << ", " << y;
        prev_x = x;
        prev_y = y;
        csv_odom.close();
    }
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node_ptr = std::make_shared<WayPointGenerator>();
    rclcpp::spin(node_ptr);
    rclcpp::shutdown();
    return 0;
}