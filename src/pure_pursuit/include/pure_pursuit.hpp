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
#include "visualization_msgs/msg/marker.hpp"

class PurePursuit : public rclcpp::Node
{
    public:
        PurePursuit();

        // Required Structures
        struct PathPoint
        {
            double x, y;

            PathPoint() : x(0.0), y(0.0){}
            PathPoint(double x, double y) : x(x), y(y){}

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
        double max_steering_angle;
         
        // Topics and Paths
        std::string csv_path;
        std::string odom_topic;
        std::string ack_topic;
        std::string graph_topic;
        std::string map_frame;
        std::string car_frame;

        // Ros_2 Objects
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
        rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr ack_pub_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr graph_pub_;

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
        void graph_closest_pathpoint();
        void map2car();
        void steering_angle_calculation();
};