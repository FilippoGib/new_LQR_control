#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <chrono>
#include <filesystem>
#include <math.h>
#include <eigen3/Eigen/Geometry>
#include <nav_msgs/msg/odometry.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include "lqr/lqr.hpp"
#include "nanoflann/nanoflann.hpp"

Eigen::Vector3f crossProduct(const Eigen::Vector3f& A, const Eigen::Vector3f& B) {
    return { 
        0, 
        0, 
        A[0] * B[1] - A[1] * B[0]
    };
}

Eigen::Vector2f subtract(const Eigen::Vector2f &a, const Eigen::Vector2f &b) {
    return {a[0] - b[0], a[1] - b[1]};
}

Eigen::Vector2f normalize(const Eigen::Vector2f &p) {
    double len = std::sqrt(p[0] * p[0] + p[1] * p[1]);
    if (len == 0) return {0, 0};  // Prevent division by zero
    return {p[0] / len, p[1] / len};
}

PointCloud get_trajectory(const std::string& trajectory_csv) 
{
    std::ifstream file(trajectory_csv);
    if (!file.is_open()) {
        throw std::runtime_error("Error opening the file " + trajectory_csv);
    }
    
    std::string line;
    PointCloud cloud;
    
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::vector<std::string> tokens;
        std::string token;
        
        // Read all tokens from the line separated by commas.
        while (std::getline(ss, token, ',')) {
            tokens.push_back(token);
        }
        
        // Skip lines with fewer than 2 columns.
        if (tokens.size() < 2) {
            continue;
        }
        
        // Skip the header row if the first token is "x"
        if (tokens[0] == "x") {
            continue;
        }
        
        try {
            double x = std::stod(tokens[0]);
            double y = std::stod(tokens[1]);
            cloud.pts.push_back({x, y});
        } catch (const std::exception& e) {
            std::cerr << "Error parsing line: " << line << " (" << e.what() << ")\n";
        }
    }
    
    file.close();
    
    if (cloud.pts.empty()) {
        throw std::runtime_error("Error empty trajectory.");
    }
    
    return cloud;
}

size_t get_closest_point(const PointCloud& cloud, const Eigen::Vector2f& odometry_pose)
{
    if (cloud.pts.empty()) {
        throw std::runtime_error("Error empty pointcloud");
    }    
    
    //set query point = odometry_point
    double query_point[2] = { odometry_pose[0], odometry_pose[1] };
    
    // Build KD-Tree
    using KDTree = nanoflann::KDTreeSingleIndexAdaptor<
    nanoflann::L2_Simple_Adaptor<double, PointCloud>, PointCloud, 2>;
    
    KDTree tree(2, cloud, nanoflann::KDTreeSingleIndexAdaptorParams(10));
    tree.buildIndex();
    
    // Query point
    std::vector<size_t> ret_index(1);
    std::vector<double> out_dist_sqr(1);
    nanoflann::KNNResultSet<double> resultSet(1);
    resultSet.init(&ret_index[0], &out_dist_sqr[0]);
    tree.findNeighbors(resultSet, query_point, nanoflann::SearchParameters(10));
    
    return ret_index[0];
}

double distance(const Eigen::Vector2f &a, const Eigen::Vector2f &b) {
    double dx = a[0] - b[0];
    double dy = a[1] - b[1];
    return std::sqrt(dx * dx + dy * dy);
}

double signed_distance(double Ax, double Ay, double Bx, double By, double theta) {
    
    Eigen::Vector3f A(cos(theta), sin(theta), 0);
    
    Eigen::Vector3f B(Bx - Ax, By - Ay, 0);
    
    Eigen::Vector3f cross = crossProduct(A, B);
    
    return cross[2]/std::abs(cross[2]);
}

double get_angular_deviation(double angle1, double angle2) {
    // Compute the raw difference, then shift by π.
    double diff = angle2 - angle1 + M_PI;
    
    // Use fmod to wrap the value into the range [0, 2π)
    diff = std::fmod(diff, 2 * M_PI);
    
    // fmod can return a negative result; adjust if necessary.
    if (diff < 0)
        diff += 2 * M_PI;
    
    // Shift back by π to get a value in [-π, π]
    diff -= M_PI;
    
    // Return the absolute value to get the magnitude in [0, π]
    //return std::abs(diff);
    return diff;
}

double get_yaw(const nav_msgs::msg::Odometry::SharedPtr msg) {
    tf2::Quaternion q;
    q.setX(msg->pose.pose.orientation.x);
    q.setY(msg->pose.pose.orientation.y);
    q.setZ(msg->pose.pose.orientation.z);
    q.setW(msg->pose.pose.orientation.w);
  
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    return yaw;
}

std::vector<double> get_tangent_angles(std::vector<Eigen::Vector2f> points)
{
    std::vector<double> tangent_angles(points.size());
    
    if (points.size() >= 2) {
        // First point: forward difference.
        Eigen::Vector2f diff = subtract(points[1], points[0]);
        Eigen::Vector2f tanVec = normalize(diff);
        tangent_angles[0] = std::atan2(tanVec[1], tanVec[0]);

        // Last point: backward difference.
        diff = subtract(points.back(), points[points.size() - 2]);
        tanVec = normalize(diff);
        tangent_angles.back() = std::atan2(tanVec[1], tanVec[0]);
    }

     for (size_t i = 1; i < points.size() - 1; ++i) {
        double d1 = distance(points[i], points[i - 1]);
        double d2 = distance(points[i + 1], points[i]);
        double ds = d1 + d2;  // Total distance over the two segments

        if (ds == 0) {
            tangent_angles[i] = 0;  // Fallback if points coincide
        } else {
            // Compute the central difference divided by the total arc length.
            Eigen::Vector2f diff = {
                (points[i + 1][0] - points[i - 1][0]) / ds,
                (points[i + 1][1] - points[i - 1][1]) / ds
            };
            Eigen::Vector2f tanVec = normalize(diff);
            tangent_angles[i] = std::atan2(tanVec[1], tanVec[0]);
        }
    }

    return tangent_angles;
}

std::vector<double> get_csv_column(const std::string& trajectory_csv, int column)
{
    std::vector<double> values;
    std::ifstream file(trajectory_csv);
    
    if (!file.is_open()) {
        throw std::runtime_error("Error: Could not open file " + trajectory_csv);
    }
    
    std::string line;
    while (std::getline(file, line)) {
        std::istringstream lineStream(line);
        std::string token;
        int currentColumn = 0;
        
        // Process each token separated by commas
        while (std::getline(lineStream, token, ',')) {
            if (currentColumn == column) {
                try {
                    double value = std::stod(token);
                    values.push_back(value);
                } catch (const std::exception& e) {
                    std::cerr << "Conversion error for token: " << token << "\n";
                }
                break; // We only need the specified column, so break out of the inner loop.
            }
            ++currentColumn;
        }
    }
    
    return values;
}

// double get_longitudinal_speed(double yaw, const nav_msgs::msg::Odometry::SharedPtr msg)
// {
//     Eigen::Rotation2D<double> rot(yaw);    // rotation transformation local -> global
//     Eigen::Vector2d v(msg->twist.twist.linear.x, msg->twist.twist.linear.y);
//     Eigen::Vector2d v_new = rot.inverse() * v;  // with the inverse rotation matrix global -> local
//     return v_new.x();
// }

std::tuple<double, Eigen::Vector2d> get_lateral_deviation_components(const double angular_dev, const double closest_point_tangent, const nav_msgs::msg::Odometry::SharedPtr msg)
{
    double Vx = msg->twist.twist.linear.x;
    double Vy = msg->twist.twist.linear.y;
    double lateral_deviation_speed = Vy*std::cos(angular_dev) + Vx*std::sin(angular_dev); // this is the speed of the car in the direction of the tangent line

    // Now reconstruct the perpendicular component into the original frame of reference
    Eigen::Vector2d d_perp(-std::sin(closest_point_tangent), std::cos(closest_point_tangent));
    return {lateral_deviation_speed, lateral_deviation_speed * d_perp};
}

double get_feedforward_term(const double K_3, const double mass, const double long_speed, const double curvature, const double frontal_lenght, const double rear_lenght, const double C_alpha_rear, const double C_alpha_front){
    double df_c1 = (mass*std::pow(long_speed,2))/(curvature*(rear_lenght+frontal_lenght));
    double df_c2 = (frontal_lenght / (2*C_alpha_front))-(rear_lenght / (2*C_alpha_rear)) + (frontal_lenght / (2*C_alpha_rear))*K_3;
    double df_c3 = (rear_lenght+frontal_lenght)/curvature;
    double df_c4 = (rear_lenght/curvature)*K_3;
    return df_c1*df_c2+df_c3-df_c4;
}

Eigen::Vector4f LQR::find_optimal_control_vector(double speed_in_module)
{
    Eigen::Vector4f optimal_control_vector;

    int closest_velocity_index = 0;
    double smallest_velocity_gap = 10e4;

    for (size_t i = 0; i < m_k_pair.size(); i++) 
        {
            // calculate the difference between speed_in_module and the velocity associated to the current control vector
            double velocity_gap = std::abs(speed_in_module - m_k_pair[i].first);
            if (velocity_gap < smallest_velocity_gap) 
            {
                closest_velocity_index = i;
                smallest_velocity_gap = velocity_gap;
            }
        }

    std::vector<double> v = m_k_pair[closest_velocity_index].second;
    optimal_control_vector << v[0], v[1], v[2], v[3];
    return optimal_control_vector;
}

double LQR::calculate_torque(double speed_in_module, double target_speed)
{
    double speed_difference = target_speed - speed_in_module;
    m_cumulative_error += speed_difference;
    return speed_difference * m_p + m_i * m_cumulative_error; // m_d * (speed_difference - m_previous_speed_difference); lets just use a PI for now
}

void LQR::load_parameters()
{
    // topics
    this->declare_parameter<std::string>("odom_topic", "");
    m_odom_topic = this->get_parameter("odom_topic").get_value<std::string>();

    this->declare_parameter<std::string>("control_topic", "");
    m_control_topic = this->get_parameter("control_topic").get_value<std::string>();

    this->declare_parameter<std::string>("partial_traj_topic", "");
    m_partial_traj_topic = this->get_parameter("partial_traj_topic").get_value<std::string>();

    this->declare_parameter<std::string>("debug_topic", "");
    m_debug_topic = this->get_parameter("debug_topic").get_value<std::string>();

    this->declare_parameter<std::string>("debug_odom_topic", "");
    m_debug_odom_topic = this->get_parameter("debug_odom_topic").get_value<std::string>();

    // settings
    this->declare_parameter<bool>("is_first_lap", false);
    m_is_first_lap = this->get_parameter("is_first_lap").get_value<bool>();

    this->declare_parameter<bool>("is_constant_speed", true);
    m_is_constant_speed = this->get_parameter("is_constant_speed").get_value<bool>();

    this->declare_parameter<std::string>("trajectory_filename", "");
    m_csv_filename = this->get_parameter("trajectory_filename").get_value<std::string>();

    this->declare_parameter<bool>("is_debug_mode", true);
    m_is_DEBUG = this->get_parameter("is_debug_mode").get_value<bool>();

    // car physical parameters
    this->declare_parameter<std::vector<std::string>>("vectors_k", std::vector<std::string>{});
    m_raw_vectors_k = this->get_parameter("vectors_k").as_string_array();

    this->declare_parameter<double>("target_speed", 0.0);
    m_target_speed = this->get_parameter("target_speed").get_value<double>();

    this->declare_parameter<double>("mass", 0.0);
    m_mass = this->get_parameter("mass").get_value<double>();

    this->declare_parameter<double>("front_length", 0.0);
    front_length = this->get_parameter("front_length").get_value<double>();

    this->declare_parameter<double>("rear_length", 0.0);
    rear_length = this->get_parameter("rear_length").get_value<double>();

    this->declare_parameter<double>("C_alpha_front", 0.0);
    C_alpha_front = this->get_parameter("C_alpha_front").get_value<double>();

    this->declare_parameter<double>("C_alpha_rear", 0.0);
    C_alpha_rear = this->get_parameter("C_alpha_rear").get_value<double>();

    // PID parameters
    this->declare_parameter<double>("PID_p", 0.0);
    m_p = this->get_parameter("PID_p").get_value<double>();

    this->declare_parameter<double>("PID_i", 0.0);
    m_i = this->get_parameter("PID_i").get_value<double>();

    this->declare_parameter<double>("PID_d", 0.0);
    m_d = this->get_parameter("PID_d").get_value<double>();

}

void LQR::initialize()
{
    // Load parameters
    this->load_parameters();

    rclcpp::QoS qos_rel(rclcpp::KeepLast(1));
    qos_rel.reliable();

    rclcpp::QoS qos_be(rclcpp::KeepLast(1));
    qos_be.best_effort();
    // Initialize pubs and subs
    m_odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(m_odom_topic, qos_rel, std::bind(&LQR::odometry_callback, this, std::placeholders::_1));
    m_partial_traj_sub = this->create_subscription<visualization_msgs::msg::Marker>(m_partial_traj_topic, qos_rel, std::bind(&LQR::partial_trajectory_callback, this, std::placeholders::_1));
    m_control_pub = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(m_control_topic, qos_rel);

    if(m_is_DEBUG){
        /* Define QoS for Best Effort messages transport */
        auto qos_d = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);
        // Create odom publisher
        m_debug_pub = this->create_publisher<nav_msgs::msg::Odometry>(m_debug_topic, qos_d);
        m_debug_odom_pub = this->create_publisher<nav_msgs::msg::Odometry>(m_debug_odom_topic, qos_d);
    }

    m_is_loaded=false;   
    m_cumulative_error = 0.0;
}


LQR::LQR() : Node("lqr_node") 
{
    this->initialize();

    // extract optimal control vectors k from params file (each one associated to its relative velocity)
    for (const auto& vec_str : m_raw_vectors_k) {
        std::stringstream ss(vec_str);
        double first_value;
        std::vector<double> values;
        if (ss >> first_value) {
            double num;
            while (ss >> num) {
                values.push_back(num);
            }
            m_k_pair.emplace_back(first_value, values);
        }
    }

    for (size_t i = 0; i < m_k_pair.size(); i++) {
        if (m_k_pair[i].second.size() == 4) {
            RCLCPP_INFO(this->get_logger(), "k%zu: %f, %f, %f, %f, %f ", i + 1, m_k_pair[i].first, m_k_pair[i].second[0], m_k_pair[i].second[1], m_k_pair[i].second[2], m_k_pair[i].second[3]);
        } else {
            RCLCPP_WARN(this->get_logger(), "k%zu: %f, each k vector MUST have EXACTLY 4 elements", i + 1, m_k_pair[i].first);
        }
    }
}
    
void LQR::odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg) 
{
    // Save the actual time to compute the time needed for the execution later
    auto start = std::chrono::high_resolution_clock::now();

    // Get Data
    Eigen::Vector2f odometry_pose(msg->pose.pose.position.x, msg->pose.pose.position.y);
    Odometry odometry = {odometry_pose, get_yaw(msg)};
    
    if(!m_is_loaded && !m_is_first_lap) // if instead we are in the first lap the trajectory should come from the callback. We don't care about it for now
    {
        std::string package_share_directory = ament_index_cpp::get_package_share_directory("lqr");
        std::string trajectory_csv = m_csv_filename;
        m_cloud = get_trajectory(package_share_directory+trajectory_csv);
    }

    // Find closest point to trajectory using KD-Tree from NanoFLANN
    size_t closest_point_index = get_closest_point(m_cloud, odometry_pose);
    Eigen::Vector2f closest_point = m_cloud.pts[closest_point_index];

    // Calculate lateral deviation as distance between two points
    double lateral_deviation = distance(odometry_pose, closest_point);

    // I have found the closest point on the trajectory to the odometry pose but I don't trust the result so I check if the previous or next point are closer to the odometry
    while(1)
    {
        if(closest_point_index > 0 && closest_point_index < m_cloud.pts.size() - 1)
        {
            double previuous_point_lateral_deviation = distance(odometry_pose, m_cloud.pts[closest_point_index - 1]);
            double next_point_lateral_deviation = distance(odometry_pose, m_cloud.pts[closest_point_index + 1]);
            if(previuous_point_lateral_deviation < lateral_deviation)
            {
                closest_point_index = closest_point_index - 1;
                closest_point = m_cloud.pts[closest_point_index];
                lateral_deviation = previuous_point_lateral_deviation;
            }
            else if(next_point_lateral_deviation < lateral_deviation)
            {
                closest_point_index = closest_point_index + 1;
                closest_point = m_cloud.pts[closest_point_index];
                lateral_deviation = next_point_lateral_deviation;
            }
            else if(next_point_lateral_deviation > lateral_deviation && previuous_point_lateral_deviation > lateral_deviation)
            {
                break;
            }
        }
        else
        {
            break;
        }
    }

    // At this point I have the closest point on the trajectory
    // Now I need to find the components of the state vector of the car

    // Compute for every point on the trajectory the tangent angle
    if (!m_is_loaded) // We only want to do it once
    {
        m_points_tangents = get_tangent_angles(m_cloud.pts); 

        std::string package_share_directory = ament_index_cpp::get_package_share_directory("lqr");
        std::string trajectory_csv = m_csv_filename;
        m_points_curvature_radius = get_csv_column(package_share_directory+trajectory_csv, 2); // for now let's say that the curvature is stored in the 3rd column of the csv file
        m_points_target_speed = get_csv_column(package_share_directory+trajectory_csv, 3); // for now let's say that the target speed is stored in the 4th column of the csv file

        m_is_loaded = true;
    }

    double closest_point_tangent = m_points_tangents[closest_point_index];

    // Now i can use the dot product to compute a signed distance and use this sign to establish where I am w.r.t. the race line
    double lateral_position = signed_distance(closest_point[0], closest_point[1], odometry_pose[0], odometry_pose[1], closest_point_tangent);
    lateral_deviation*=lateral_position;

    // Finally calculate the angular deviation between the odometry and the closest point on the trajectory
    double angular_deviation = get_angular_deviation(closest_point_tangent, odometry.yaw);

    // Lastly compute the lateral deviation speed and lateral deviation vector
    auto [lateral_deviation_speed, v_ld] = get_lateral_deviation_components(angular_deviation, closest_point_tangent, msg);

    // The angular deviation speed is free and comes from the odometry
    double angular_deviation_speed = msg->twist.twist.angular.z;

    // NOW WE HAVE ALL THE COMPONENTS OF THE STATE VECTOR OF THE CAR
    Eigen::Vector4f x;
    x << lateral_deviation, lateral_deviation_speed, angular_deviation, angular_deviation_speed;

    // Now we find the optimal control vector k based on the current speed
    double speed_in_module = std::sqrt(std::pow(msg->twist.twist.linear.x, 2) + std::pow(msg->twist.twist.linear.y, 2));
    Eigen::Vector4f optimal_control_vector = find_optimal_control_vector(speed_in_module);
    double K_3 = optimal_control_vector[2];

    // Now we compute the theoretical steering
    double steering = -optimal_control_vector.dot(x); 

    // Now we need to calculate Vx and and the curvature radious
    double Vx = msg->twist.twist.linear.x;
    double R_c = m_points_curvature_radius[closest_point_index];

    // Now we compute the feedforward term
    double delta_f = get_feedforward_term(K_3, m_mass, Vx, 1/R_c, front_length, rear_length, C_alpha_rear, C_alpha_front);
    
    steering = steering - delta_f; // this is my actual steering target

    // NOW WE HAVE TO CALCULATE THE LONGITUDINAL CONTROL
    double target_speed = 0.0;
    if (m_is_constant_speed) // if we are in constant speed mode
    {
        target_speed = m_target_speed;
    }
    else
    {
        target_speed = m_points_target_speed[closest_point_index];
    }
    
    if (speed_in_module < 1/*ms*/)
    {
        m_cumulative_error = 0; // I don't want my integral term to accumulate when I am stationary eg. at the start of the mission
    }

    double throttle = calculate_torque(speed_in_module, target_speed); // It is delegated to the simulator to map torque in [-1,1] 

    // Now we have the steering and the throttle, we can create a message and publish it
    ackermann_msgs::msg::AckermannDriveStamped control_msg;
    control_msg.header.stamp = this->get_clock()->now();
    control_msg.header.frame_id = "map"; 
    control_msg.drive.steering_angle = steering;
    control_msg.drive.speed = throttle;
    m_control_pub->publish(control_msg);
    
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();

    if(m_is_DEBUG){
        RCLCPP_INFO(this->get_logger(), "odometry_pose: x=%.2f, y=%.2f", odometry_pose[0], odometry_pose[1]);
        RCLCPP_INFO(this->get_logger(), "yaw: %.2f", odometry.yaw);
        RCLCPP_INFO(this->get_logger(), "Closest Point: x=%.2f, y=%.2f", closest_point[0], closest_point[1]);
        RCLCPP_INFO(this->get_logger(), "steering: %.4f, delta_f = %.4f", steering, delta_f);
        RCLCPP_INFO(this->get_logger(), "x: [%.2f,%.2f,%.2f,%.2f]", x[0],x[1],x[2],x[3]);
        RCLCPP_INFO(this->get_logger(), "duration: %ld ns", duration);
        // RCLCPP_INFO(this->get_logger(), "k: [%.2f,%.2f,%.2f,%.2f]", optimal_control_vector[0],optimal_control_vector[1],optimal_control_vector[2],optimal_control_vector[3]);
    }

    if(m_is_DEBUG) // publish state vector data
    {
        nav_msgs::msg::Odometry debby;
        debby.header.frame_id = "debug";
        debby.child_frame_id = "imu_link";
        debby.header.stamp = msg->header.stamp;
        debby.pose.pose.position.y = x[0]; // here we put the lateral deviation on the y axis
        debby.twist.twist.linear.y = x[1]; // here we put the lateral deviation speed on the y axis
        debby.pose.pose.orientation.z = x[2]; // here we put the angular deviation 
        debby.twist.twist.angular.z = x[3]; // here we put the angular deviation speed
        m_debug_pub->publish(debby);

        nav_msgs::msg::Odometry nn_debby;
        nn_debby.header.frame_id = "debug";
        nn_debby.child_frame_id = "imu_link";
        nn_debby.header.stamp = msg->header.stamp;
        nn_debby.pose.pose.position.x = closest_point[0];
        nn_debby.pose.pose.position.y = closest_point[1];
        nn_debby.twist.twist.linear.x = v_ld.x();
        nn_debby.twist.twist.linear.y = v_ld.y();
        m_debug_odom_pub->publish(nn_debby);
    }
}

void LQR::partial_trajectory_callback(const visualization_msgs::msg::Marker traj)
{
    if(!m_is_first_lap) // only execute this if we are in the first lap
    {
        return;
    }
    // TODO: logic to use partial trajectory instead of the trajectory from .csv file
    RCLCPP_INFO(this->get_logger(), "Partial trajectory callback entered");

    return;
}

void LQR::global_trajectory_callback()
{
    return;
}
