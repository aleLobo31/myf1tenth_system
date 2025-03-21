#include "pure_pursuit.hpp"


PurePursuit::PurePursuit() : Node("pure_pursuit_node")
{
    // Establish some private variables as parameters
    this->declare_parameter<double>("lookahead_dist", 1.5);
    this->declare_parameter<double>("min_lookahead_dist", 0.5);
    this->declare_parameter<double>("max_lookahead_dist", 4.0);
    this->declare_parameter<double>("lookahead_ratio", 8.0);
    this->declare_parameter<double>("max_speed", 4.0);
    this->declare_parameter<double>("Kp", 0.3);
    this->declare_parameter<double>("max_steering_angle", 0.7);
    this->declare_parameter<int>("n_pathpoints", 123);
    this->declare_parameter<int>("window_size", 25);
    this->declare_parameter<std::string>("csv_path", "/sim_ws/src/pure_pursuit/racelines/waypoints_odom_3.csv");
    this->declare_parameter<std::string>("map_frame", "map");
    this->declare_parameter<std::string>("car_frame", "ego_racecar/base_link");
    this->declare_parameter<std::string>("odom_topic", "/ego_racecar/odom");
    this->declare_parameter<std::string>("drive_topic", "/drive");

    // Retrieve parameter values
    lookahead_dist = this->get_parameter("lookahead_dist").as_double();
    min_lookahead_dist = this->get_parameter("min_lookahead_dist").as_double();
    max_lookahead_dist = this->get_parameter("max_lookahead_dist").as_double();
    lookahead_ratio = this->get_parameter("lookahead_ratio").as_double();
    max_speed = this->get_parameter("max_speed").as_double();
    Kp = this->get_parameter("Kp").as_double();
    max_steering_angle = this->get_parameter("max_steering_angle").as_double();
    n_pathpoints = this->get_parameter("n_pathpoints").as_int();
    window_size = this->get_parameter("window_size").as_int();
    csv_path = this->get_parameter("csv_path").as_string();
    map_frame = this->get_parameter("map_frame").as_string();
    car_frame = this->get_parameter("car_frame").as_string();
    odom_topic = this->get_parameter("odom_topic").as_string();
    ack_topic = this->get_parameter("drive_topic").as_string();

    // Other required member variables
    graph_topic = "visualization_marker";
    start_index = 0;

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(odom_topic, 100, std::bind(&PurePursuit::odom_callback, this, std::placeholders::_1));
    ack_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(ack_topic, 10);
    graph_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(graph_topic, 10);

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

void PurePursuit::graph_closest_pathpoint()
{
    auto marker = visualization_msgs::msg::Marker();

    marker.header.frame_id = "map";
    marker.header.stamp = rclcpp::Clock().now();

    marker.ns = "basic_shapes";
    marker.id = 0;

    marker.type = visualization_msgs::msg::Marker::SPHERE;

    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.scale.x = 0.15;
    marker.scale.y = 0.15;
    marker.scale.z = 0.15;
    marker.color.a = 1.0;
    marker.color.r = 1.0;

    marker.pose.position.x = v_global[0];
    marker.pose.position.y = v_global[1];
    marker.pose.position.z = 0.0;

    graph_pub_->publish(marker);

    return;
}

void PurePursuit::get_closest_pathpoint()
{
    int i = start_index;
    double aux;
    double closest_distance = std::numeric_limits<double>::max();
    
    // Iterate through window_size
    for(int n = 0; n < window_size; n++)
    {
        // Calculate pathpoint i to current pose distance
        aux = std::sqrt(std::pow(pathpoints[i].x - curr_pose.x, 2) + std::pow(pathpoints[i].y - curr_pose.y, 2));

        // std::cout << "Closest_Distance: " << closest_distance << std::endl;

        // Access to i pathpoint and compare it (First Exclude the points that are not in range)
        if(aux >= lookahead_dist && aux < closest_distance)
        {            
            closest_distance = aux;
            start_index = i;
        
            // Use an Eigen Vector to express the closest point (from Map frame perspective)
            v_global << pathpoints[i].x, pathpoints[i].y, 0.0;
        }

        // Iterate
        i = (i+1)%n_pathpoints;
    }

    // std::cout << "Closest Point: " << v_global[0] << " " << v_global[1] << " " << l << std::endl;

    graph_closest_pathpoint();

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

    // Calculate the Curvature (or Steering Angle) that connects to the Closest Point (expressed in Car Frame)
    float k =  Kp * (2 * v_local[1]) / std::pow(std::sqrt(std::pow(v_local[0], 2) + std::pow(v_local[1], 2)), 2);

    if(k > max_steering_angle)
    {
        k = max_steering_angle;    
    } else if(k < -max_steering_angle)
    {
        k = -max_steering_angle;
    }

    // Determine speed depending on the value of k
    cmd.drive.speed = max_speed/(1 + k/max_steering_angle);  
    // std::cout << "Speed: " << cmd.drive.speed << std::endl;

    cmd.drive.steering_angle = k;

    // std::cout << "Steering Angle: " << cmd.drive.steering_angle << "\n" <<  "Speed: "  << cmd.drive.speed << std::endl;

    // Command the car
    ack_pub_->publish(cmd);
    
    return;
}

void PurePursuit::odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg)
{
    // Retrieve current pose
    curr_pose.x = odom_msg->pose.pose.position.x;
    curr_pose.y = odom_msg->pose.pose.position.y;
    double curr_vel = std::sqrt(std::pow(odom_msg->twist.twist.linear.x, 2) + std::pow(odom_msg->twist.twist.linear.y, 2));

    // Calculate lookahead_dist dynamically
    lookahead_dist = std::min(std::max(max_lookahead_dist * curr_vel /lookahead_ratio, min_lookahead_dist), max_lookahead_dist);
    // std::cout << "Lookahead_dist: " << lookahead_dist << std::endl;

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