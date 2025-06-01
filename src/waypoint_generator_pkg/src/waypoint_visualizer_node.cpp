#include "waypoint_generator_pkg/waypoint_visualizer_node.hpp"
#include <memory>
#include <sstream>

WaypointVisualizerNode::WaypointVisualizerNode() 
    : Node("waypoint_visualizer_node") 
{
    // Declare parameters
    this->declare_parameter<std::string>("csv_file_path", "");
    this->declare_parameter<std::string>("frame_id", "map");
    this->declare_parameter<double>("marker_scale", 0.2);
    this->declare_parameter<std::vector<double>>("marker_color", {1.0, 0.0, 0.0, 1.0}); // Red by default

    // Get parameters
    csv_file_path_ = this->get_parameter("csv_file_path").as_string();
    frame_id_ = this->get_parameter("frame_id").as_string();
    marker_scale_ = this->get_parameter("marker_scale").as_double();
    marker_color_ = this->get_parameter("marker_color").as_double_array();

    // Create publisher
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "waypoint_markers", 10);

    // Load waypoints from CSV
    if (!csv_file_path_.empty()) {
        loadWaypoints(csv_file_path_);
    } else {
        RCLCPP_ERROR(this->get_logger(), "No CSV file path provided!");
        return;
    }

    // Create timer for periodic publishing
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&WaypointVisualizerNode::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "Waypoint visualizer node initialized");
}

void WaypointVisualizerNode::loadWaypoints(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open CSV file: %s", filename.c_str());
        return;
    }

    std::string line;
    // Skip header line
    std::getline(file, line);

    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string value;
        std::vector<double> values;

        while (std::getline(ss, value, ',')) {
            values.push_back(std::stod(value));
        }

        if (values.size() >= 2) {
            geometry_msgs::msg::Point point;
            point.x = values[0];
            point.y = values[1];
            point.z = 0.0;
            waypoints_.push_back(point);
        }
    }

    RCLCPP_INFO(this->get_logger(), "Loaded %zu waypoints", waypoints_.size());
}

void WaypointVisualizerNode::publishMarkers() {
    visualization_msgs::msg::MarkerArray marker_array;
    
    // Create points marker
    visualization_msgs::msg::Marker points_marker;
    points_marker.header.frame_id = frame_id_;
    points_marker.header.stamp = this->now();
    points_marker.ns = "waypoints";
    points_marker.id = 0;
    points_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    points_marker.action = visualization_msgs::msg::Marker::ADD;
    points_marker.scale.x = marker_scale_;
    points_marker.scale.y = marker_scale_;
    points_marker.scale.z = marker_scale_;
    points_marker.color.r = marker_color_[0];
    points_marker.color.g = marker_color_[1];
    points_marker.color.b = marker_color_[2];
    points_marker.color.a = marker_color_[3];
    points_marker.points = waypoints_;

    // Create line strip marker
    visualization_msgs::msg::Marker line_marker;
    line_marker.header.frame_id = frame_id_;
    line_marker.header.stamp = this->now();
    line_marker.ns = "waypoint_path";
    line_marker.id = 1;
    line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line_marker.action = visualization_msgs::msg::Marker::ADD;
    line_marker.scale.x = marker_scale_ / 2.0;  // Line width
    line_marker.color = points_marker.color;
    line_marker.color.a = 0.5;  // Semi-transparent
    line_marker.points = waypoints_;

    marker_array.markers.push_back(points_marker);
    marker_array.markers.push_back(line_marker);
    marker_pub_->publish(marker_array);
}

void WaypointVisualizerNode::timer_callback() {
    publishMarkers();
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WaypointVisualizerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
