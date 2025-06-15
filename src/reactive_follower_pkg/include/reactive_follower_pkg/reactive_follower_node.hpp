#ifndef REACTIVE_FOLLOWER_NODE_HPP_
#define REACTIVE_FOLLOWER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <interfaces_pkg/msg/goal_point.hpp>
#include <vector>
#include <utility>
#include <memory>

class ReactiveFollowerNode : public rclcpp::Node {
public:
    ReactiveFollowerNode();

    Struct Gap {
        size_t start;
        size_t end;

        Gap() : start(0.0), end(0.0){}
        Gap(size_t start, size_t end)
            : start(start), end(end){}
    };

private:
    // ROS Parameters
    std::string lidarscan_topic;
    std::string goalpoint_topic;
    std::string drive_topic;
    std::string laser_frame;
    std::string car_frame;

    int bubble_radius;
    double max_speed;
    double min_speed;
    double lidar_angle;
    double max_lidar_distance;
    double weight_speed;
    double weight_steering;

    // Scan indices and angles
    size_t start_index;
    size_t end_index;
    double start_angle;
    double end_angle;
    int pg_index;
    double safety_distance;
    size_t min_gap_size;

    std::string map_frame;
    std::string car_frame;

    // Transform handling
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    geometry_msgs::msg::TransformStamped current_transform_;

    // ROS communication
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_subscriber_;
    rclcpp::Subscription<interfaces_pkg::msg::GoalPoint>::SharedPtr goal_subscriber_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_publisher_;

    // LiDAR processing methods
    void preprocess_lidar(std::vector<float> &ranges);
    size_t find_closest_point(const std::vector<float> &ranges);
    void eliminate_bubble(std::vector<float> &ranges, size_t closest_idx, float bubble_radius);

    double calculate_safety_distance(double speed); // returns safety distance based on speed
        //calculo de safety_distance en funcion de la velocidad
        //si la velocidad es < 0.7, devolver 40
        //si la velocidad es mayor que 0.7 calcular proporcionalmente safety_distance 
    int calculate_min_gap_size(double safety_distance); 
        // calculates minimum number of LiDAR beams for a safe gap
    int point_to_lidar_index(); 
        // converts goal point coordinates to LiDAR index
    void find_gaps(struct gaps);
        //buscar gaps con nº minimo de indices y utilizando safety_distance
        //tf statica de baselink->laser
        //calculo de indice proximo con coordenadas transformadas
    bool gp_in_gaps(indice_pp, gaps)
        //si indice_pp esta dentro de un gap, publicar commandos de pp
        //si no está en ninguno, buscar el gap más cercano(min dist a extremo sde gaps) 
        //y publicar logica existente
    
    void alternative_commands(gaps);

    // Callback
    void goal_callback(const interfaces_pkg::msg::GoalPoint::ConstSharedPtr msg);
};

#endif //REACTIVE_FOLLOWER_NODE_HPP_