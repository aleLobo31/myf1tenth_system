#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <cmath>
#include <limits>
#include <eigen3/Eigen/Dense>

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/exceptions.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

class PurePursuit : public rclcpp::Node
{
    public:
        PurePursuit();

        // Required Structures
        struct PathPoint
        {
            double x, y, l;

            PathPoint() : x(0.0), y(0.0), l(0.0){}
            PathPoint(double x, double y) : x(x), y(y), l(std::sqrt(std::pow(x, 2) + std::pow(y, 2))){}

            // PathPoint(const PathPoint& other) : x(other.x), y(other.y), l(other.l)
            // {
            //     std::cout << "Copied!" << std::endl;
            // }
        };

    private:
        // Pathpoints
        int n_pathpoints;
        int start_index;
        int window_size;
        PathPoint curr_pose;
        std::vector<PathPoint> pathpoints;

        // Parameters
        double lookahead_dist;
        double Kp;
        bool exe;
         
        // Topics and Paths
        std::string csv_path;
        std::string odom_topic;
        std::string ack_topic;
        std::string map_frame;
        std::string car_frame;

        // Ros_2 Objects
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
        rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr ack_pub_;

        // Linear Algebra Objects
        Eigen::Vector3d v_global;
        Eigen::Vector3d v_local;

        // tf_2 Objects
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};

        // Required Functions/Methods
        int load_pathpoints2memory();
        void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg);
        void get_closest_pathpoint();
        void map2car();
        void steering_angle_calculation();
};

/*

1. Extraigo posición actual del vehículo (En el Simulador ya está en el marco de referencia del mapa)

2. Itero sobre el set del spline y encuentro el punto más cercano (goal point)
    - GOAL POINT: "Is a point on the path that is one lookahead distance from the current vehicle position"
    - Satisface min(goal-point - x) que pertenece a DF

3. Expreso el goal point en el marco de referencia del coche

4. Calculo el arco que une la posición actual del vehículo con el GOAL POINT

5. Actualizo el current pose 

VARIABLES
- LookAhead Distance
- Current Pose
- Suscriptor a la odometría
- Publisher ackermann 

DISEÑO:
- Función para MAP -> tf -> EGO_RACECAR tf(x, y)
- Función para comparar distancias
- Función para minimizar.

¿Qué pasa si el punto más cercano me dice que está detrás mío? Corre en sentido contrario

*/