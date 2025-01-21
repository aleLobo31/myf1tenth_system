#include "pure_pursuit.hpp"

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

PurePursuit::PurePursuit() : Node("pure_pursuit_node")
{
    // Establish some private variables as parameters
    this->declare_parameter("lookahead_dist", 1.5);
    this->declare_parameter("Kp", 0.2);

    // Retrieve parameter values
    lookahead_dist = this->get_parameter("lookahead_dist").as_double();
    Kp = this->get_parameter("Kp").as_double();

    odom_topic = "/ego_racecar/odom";
    ack_topic = "/drive";
    csv_path = "/sim_ws/src/pure_pursuit/racelines/smooth_trajectory.csv";
    map_frame = "map";
    car_frame = "ego_racecar/base_link";

    n_pathpoints = 100;
    start_index = 0;
    window_size = 25;
    exe = true;

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(odom_topic, 100, std::bind(&PurePursuit::odom_callback, this, std::placeholders::_1));
    ack_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(ack_topic, 10);
    
    // Buffer para guardar Transformaciones entre Coordinate Frames
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());

    // Creamos un objeto de tipo Listener para que automáticamente guarde la Transformación en el Buffer
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // We load the path into memory
    load_pathpoints2memory();
}

int PurePursuit::load_pathpoints2memory()
{
    // Open the csv
    std::ifstream csv(csv_path);

    if(!csv.is_open())
    {
        std::cerr << "Error: Could Not Open the File" << std::endl;
        return -1;
    }
 
    // Create a vector to hold PathPoints
    pathpoints.reserve(n_pathpoints);

    std::string row, x_str, y_str;

    for(int i = 0; i < n_pathpoints; i++)
    {
        // Read one line (x, y)
        std::getline(csv, row, '\n');
        std::stringstream ss(row);

        for(int j = 0; j < 2; j++)
        {
            // Extract x and y in two iterations
            if(j == 0)
            {
                std::getline(ss, x_str, ',');
            }
            else if (j == 1)
            {
                std::getline(ss, y_str);
            }
        }

        // Push the new element into the vector
        pathpoints.emplace_back(std::stod(x_str), std::stod(y_str));
    }

    // std::cout << "Elements: " << pathpoints[0].x << ", " << pathpoints[0].y << ", " << pathpoints[0].l << std::endl;
    // std::cout << "Size: " << pathpoints.size() << std::endl;

    return 0;
}

void PurePursuit::get_closest_pathpoint()
{
    int i = start_index;
    double aux;
    // double l = 0.0;
    double closest_distance = std::numeric_limits<double>::max();
    
    // First we are going to do it using a naive approach (iterate whole array)
    /*THREAD IMPLEMENTATION IN THE FUTURE*/

    // std::cout << "L = " << curr_pose.l << std::endl;
    for(int n = 0; n < window_size; n++)
    {
        // Calculate pathpoint i to current pose distance
        aux = std::sqrt(std::pow(pathpoints[i].x - curr_pose.x, 2) + std::pow(pathpoints[i].y - curr_pose.y, 2));

        // std::cout << "Closest_Distance: " << closest_distance << std::endl;

        // Access to i pathpoint and compare it (First Exclude the points that are not in range)
        if(aux >= lookahead_dist && aux < closest_distance)
        {
            // dot_product = pathpoints[i].x * curr_pose.x + pathpoints[i].y * curr_pose.y;
            
            closest_distance = aux;
            start_index = i;
            // max_dot_product = dot_product;
        
            // Use an Eigen Vector to express the closest point (from Map frame perspective)
            v_global << pathpoints[i].x, pathpoints[i].y, 0.0;

            // l = pathpoints[i].l;
        }

        // Iterate
        i = (i+1)%n_pathpoints;
    }

    // std::cout << "Closest Point: " << v_global[0] << " " << v_global[1] << " " << l << std::endl;

    return;
}

void PurePursuit::map2car()
{
    // Get the Transformation (Rotation + Translation) between Map Frame and Car Frame
    geometry_msgs::msg::TransformStamped t;
    
    // Define a timeout to avoid Frame Not Found Errors
    tf2::Duration timeout = std::chrono::duration<int64_t>(2); 

    try 
    {
        t = tf_buffer_->lookupTransform(car_frame, map_frame, tf2::TimePointZero, timeout);
    } catch (const tf2::TransformException & ex) {
        RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s", car_frame.c_str(), map_frame.c_str(), ex.what());
        return;
    }

    // Transform the Translation into a 3d Vector and the Quaternion into a Rotation Matrix
    Eigen::Vector3d translation;
    translation << t.transform.translation.x, t.transform.translation.y, t.transform.translation.z;

    double q0 = t.transform.rotation.w;
    double q1 = t.transform.rotation.x;
    double q2 = t.transform.rotation.y;
    double q3 = t.transform.rotation.z;

    Eigen::Matrix3d R;
    R << 2 * (q0*q0 + q1*q1) - 1,
        2 * (q1*q2 - q0*q3),
        2 * (q1*q3 + q0*q2),
        2 * (q1*q2 + q0*q3),
        2 * (q0*q0 + q2*q2) - 1,
        2 * (q2*q3 - q0*q1),
        2 * (q1*q3 - q0*q2),
        2 * (q2*q3 + q0*q1),
        2 * (q0*q0 + q3*q3) -1;

    // Express v_global in Car Reference Frame (First Rotation and Then Translation)
    v_local = (R * v_global) + translation;
    
    // std::cout << "Coordenadas Globales:\n" << v_global << std::endl;
    // std::cout << "Coordenadas Locales:\n" << v_local << std::endl;

    return;
}

void PurePursuit::steering_angle_calculation()
{
    auto cmd = ackermann_msgs::msg::AckermannDriveStamped();

    // Calculte the Curvature (or Steering Angle) that connects to the Closest Point (expressed in Car Frame)
    float k =  Kp * ((2 * abs(v_local[1])) / std::sqrt(std::pow(v_local[0], 2) + std::pow(v_local[1], 2)));

    // Determine speed depending on the value of k
    if(k > 0.5)
    {
        cmd.drive.speed = 1.0;
    } else
    {
        cmd.drive.speed = 0.5;
    }

    cmd.drive.steering_angle = k;

    std::cout << "Steering Angle: " << cmd.drive.steering_angle << "\n" <<  "Speed: "  << cmd.drive.speed << std::endl;

    // Command the car
    ack_pub_->publish(cmd);
    
    return;
}

void PurePursuit::odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg)
{
    // Retrieve current pose
    curr_pose.x = odom_msg->pose.pose.position.x;
    curr_pose.y = odom_msg->pose.pose.position.y;
    curr_pose.l = std::sqrt(std::pow(curr_pose.x, 2) + std::pow(curr_pose.y, 2));

    // Get the closest pathpoint
    get_closest_pathpoint();
    
    // Transform the closest pathpoint to Car Reference Frame
    map2car();

    // Calculate the steering angle and publish it
    steering_angle_calculation();
}

int main(int argc, char**argv)
{
    rclcpp::init(argc, argv);
    auto node_ptr = std::make_shared<PurePursuit>();
    rclcpp::spin(node_ptr);
    rclcpp::shutdown();
}