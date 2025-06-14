#include "pure_pursuit_pkg/pure_pursuit_node.hpp"


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
    this->declare_parameter<std::string>("car_frame", "base_link");
    this->declare_parameter<std::string>("odom_topic", "/odom");
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

    RCLCPP_INFO(this->get_logger(), "Pure Pursuit Node has started.");
    RCLCPP_INFO(this->get_logger(), "CSV Path: %s", csv_path.c_str());
    RCLCPP_INFO(this->get_logger(), "Odom Topic: %s", odom_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "Drive Topic: %s", ack_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "Map Frame: %s", map_frame.c_str());    
    RCLCPP_INFO(this->get_logger(), "Car Frame: %s", car_frame.c_str());
    RCLCPP_INFO(this->get_logger(), "Lookahead Distance: %f", lookahead_dist);
    RCLCPP_INFO(this->get_logger(), "Minimum Lookahead Distance: %f", min_lookahead_dist);
    RCLCPP_INFO(this->get_logger(), "Maximum Lookahead Distance: %f", max_lookahead_dist);  
    RCLCPP_INFO(this->get_logger(), "Lookahead Ratio: %f", lookahead_ratio);
    RCLCPP_INFO(this->get_logger(), "Max Speed: %f", max_speed);
    RCLCPP_INFO(this->get_logger(), "Kp: %f", Kp);
    RCLCPP_INFO(this->get_logger(), "Max Steering Angle: %f", max_steering_angle);
    RCLCPP_INFO(this->get_logger(), "Number of Pathpoints: %d", n_pathpoints);
    RCLCPP_INFO(this->get_logger(), "Window Size: %d", window_size);

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

    marker.header.frame_id = map_frame;  // Use parameter instead of hardcoded "map"
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

    // Add logging for waypoint information
    RCLCPP_INFO(this->get_logger(), "Using waypoint %d at position (%.2f, %.2f)", 
                start_index, v_global[0], v_global[1]);

    graph_pub_->publish(marker);

    return;
}

void PurePursuit::get_closest_pathpoint()
{
    if (window_size > n_pathpoints) {
        RCLCPP_ERROR(this->get_logger(), "Window size (%d) larger than path points (%d)", window_size, n_pathpoints);
        return;
    }

    int i = start_index;
    double distance_to_pose;
    double closest_distance = std::numeric_limits<double>::max();
    
    // Iterate through window_size
    for(int n = 0; n < window_size; n++)
    {
        // Calculate pathpoint i to current pose distance
        distance_to_pose = std::sqrt(std::pow(pathpoints[i].x - curr_pose.x, 2) + std::pow(pathpoints[i].y - curr_pose.y, 2));
        RCLCPP_INFO(this->get_logger(), "Point: %i, Closest_distance: %f", i, distance_to_pose);


        // Transform point to check if it's in front of the car
        Eigen::Vector3d point;
        point << pathpoints[i].x, pathpoints[i].y, 0.0;
        Eigen::Vector3d point_local = transform_to_car_frame(point);

        // Only consider points that are in front of the car and beyond lookahead distance
        if (distance_to_pose >= lookahead_dist && distance_to_pose < closest_distance)
        {            
            closest_distance = distance_to_pose;
            start_index = i;
            v_global << pathpoints[i].x, pathpoints[i].y, 0.0;
        }

        i = (i+1)%n_pathpoints;
    }
        RCLCPP_INFO(this->get_logger(), "VENTANA TERMINADA");


    graph_closest_pathpoint();
}

Eigen::Matrix3d PurePursuit::quaternionToMatrix(const geometry_msgs::msg::Quaternion& q)
{
    // Matrix will represent R_car2map (car to map rotation)
    double q0 = q.w;
    double q1 = q.x;
    double q2 = q.y;
    double q3 = q.z;

    Eigen::Matrix3d R_car2map;
    R_car2map << 2 * (q0*q0 + q1*q1) - 1,
         2 * (q1*q2 - q0*q3),
         2 * (q1*q3 + q0*q2),
         2 * (q1*q2 + q0*q3),
         2 * (q0*q0 + q2*q2) - 1,
         2 * (q2*q3 - q0*q1),
         2 * (q1*q3 - q0*q2),
         2 * (q2*q3 + q0*q1),
         2 * (q0*q0 + q3*q3) - 1;
    return R_car2map;
}

Eigen::Vector3d PurePursuit::transform_to_car_frame(const Eigen::Vector3d& point_map)
{
    // Use cached transform instead of looking it up again
    Eigen::Vector3d t_car_in_map(
        current_transform_.transform.translation.x,
        current_transform_.transform.translation.y,
        current_transform_.transform.translation.z
    );

    // Get car→map rotation and transpose for map→car
    Eigen::Matrix3d R_car2map = quaternionToMatrix(current_transform_.transform.rotation);
    Eigen::Matrix3d R_map2car = R_car2map.transpose();
    
    // First subtract translation, then rotate
    return R_map2car * (point_map - t_car_in_map);
}

void PurePursuit::map2car()
{
    v_local = transform_to_car_frame(v_global);
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
    std::cout << "Speed: " << cmd.drive.speed << std::endl;

    cmd.drive.steering_angle = k;
    std::cout << "Steering Angle: " << cmd.drive.steering_angle << "\n" <<  "Speed: "  << cmd.drive.speed << std::endl;

    // Command the car
    ack_pub_->publish(cmd);
    
    return;
}

void PurePursuit::odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg)
{
    // Get forward speed from odom
    double curr_vel = std::hypot(
        odom_msg->twist.twist.linear.x,
        odom_msg->twist.twist.linear.y
    );

    // Cache current transform for this cycle
    try {
        current_transform_ = tf_buffer_->lookupTransform(
            map_frame,          // target frame
            car_frame,          // source frame
            tf2::TimePointZero, 
            std::chrono::milliseconds(100)
        );
    } catch (const tf2::TransformException &ex) {
        RCLCPP_WARN(
            this->get_logger(),
            "Failed to get %s → %s transform: %s",
            map_frame.c_str(), car_frame.c_str(), ex.what()
        );
        return;
    }

    // Update current pose
    curr_pose.x = current_transform_.transform.translation.x;
    curr_pose.y = current_transform_.transform.translation.y;

    // Rest of the pipeline uses cached transform
    get_closest_pathpoint();
    map2car();
    steering_angle_calculation();
}

int main(int argc, char**argv)
{
    rclcpp::init(argc, argv);
    auto node_ptr = std::make_shared<PurePursuit>();
    rclcpp::spin(node_ptr);
    rclcpp::shutdown();
}