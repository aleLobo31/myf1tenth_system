#include "f1tenth_stack/tf_publisher_node.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TFPublisherNode>());
    rclcpp::shutdown();
    return 0;
} 