#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <fstream>
#include <vector>
#include <string>

class WaypointVisualizerNode : public rclcpp::Node {
public:
    explicit WaypointVisualizerNode();

private:
    void loadWaypoints(const std::string& filename);
    void publishMarkers();
    void timer_callback();

    std::vector<geometry_msgs::msg::Point> waypoints_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::string csv_file_path_;
    std::string frame_id_;
    double marker_scale_;
    std::vector<double> marker_color_;
};
