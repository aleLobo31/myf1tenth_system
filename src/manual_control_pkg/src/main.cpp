#include "manual_control_pkg/manual_control_node.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ManualControlNode>());
    rclcpp::shutdown();
    return 0;
} 