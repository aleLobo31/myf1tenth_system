#include "reactive_follower_pkg/reactive_follower_node.hpp"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ReactiveFollowerNode>());
    rclcpp::shutdown();
    return 0;
}
