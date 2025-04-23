#include "lqr/lqr.hpp"

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

size_t LQR::get_closest_point_from_KD_Tree(const Eigen::Vector2f& odometry_pose, double radius)
{
    if (!m_kdtree) {
        throw std::runtime_error("KD‑tree not initialized");
    }

    // if the cloud has been updated elsewhere, rebuild:
    if (m_cloud_has_changed) {
        m_kdtree->~KDTreeType();  // destroy old
        m_kdtree = std::make_unique<KDTreeType>(
            2, m_cloud, nanoflann::KDTreeSingleIndexAdaptorParams(10)
        );
        m_kdtree->buildIndex();
        m_cloud_has_changed = false;
    }

    double query_pt[2] = { odometry_pose.x(), odometry_pose.y() };
    const double sq_radius = radius * radius;

    // now just search the existing tree
    std::vector< nanoflann::ResultItem<unsigned int, double> > matches;
    nanoflann::SearchParameters params;
    const size_t n_matches =
        m_kdtree->radiusSearch(query_pt, sq_radius, matches, params);

    if (n_matches == 0) {
        throw std::runtime_error("No points within radius");
    }

    // pick the closest among them
    size_t best = matches[0].first;
    double best_d2 = matches[0].second;
    for (size_t i = 1; i < n_matches; ++i) {
        if (matches[i].second < best_d2) {
            best_d2 = matches[i].second;
            best = matches[i].first;
        }
    }
    return best;
}

size_t LQR::get_closest_point_along_S(const Eigen::Vector2f& odometry_pose, double window, double& out_S)
{
    // 1) Find S-range [S_prev – window, S_prev + window]
    double lowS  = m_S_prev - window;
    double highS = m_S_prev + window;

    // 2) Binary-search in m_points_s (must be sorted ascending)
    auto it_low  = std::lower_bound(m_points_s.begin(), m_points_s.end(), lowS);
    auto it_high = std::upper_bound(m_points_s.begin(), m_points_s.end(), highS);
    size_t idx_low  = std::distance(m_points_s.begin(), it_low);
    size_t idx_high = std::distance(m_points_s.begin(), it_high);

    if (idx_low >= m_cloud.pts.size() || idx_low >= idx_high)
        throw std::runtime_error("No points in S-window");

    // 3) Brute-force scan for the absolute closest in Euclidean space
    double best_d2 = std::numeric_limits<double>::infinity();
    size_t best_i  = idx_low;
    for (size_t i = idx_low; i < idx_high; ++i) {
        double dx = odometry_pose.x() - m_cloud.pts[i].x();
        double dy = odometry_pose.y() - m_cloud.pts[i].y();
        double d2 = dx*dx + dy*dy;
        if (d2 < best_d2) {
            best_d2 = d2;
            best_i  = i;
        }
    }

    out_S = m_points_s[best_i];
    return best_i;
}

double points_distance(const Eigen::Vector2f &a, const Eigen::Vector2f &b) {
    double dx = a[0] - b[0];
    double dy = a[1] - b[1];
    return std::sqrt(dx * dx + dy * dy);
}

double line_distance(const Eigen::Vector2f &point, const Eigen::Vector2f &nearest_point, double line_angle) 
{
    Eigen::Vector2f diff = point - nearest_point;
    
    double sinTheta = std::sin(line_angle);
    double cosTheta = std::cos(line_angle);
    
    double distance = std::abs(diff.x() * (-sinTheta) + diff.y() * cosTheta);
    return distance;
}

double get_sign(double Ax, double Ay, double Bx, double By, double theta) {
    
    Eigen::Vector3f A(cos(theta), sin(theta), 0);
    
    Eigen::Vector3f B(Bx - Ax, By - Ay, 0);
    
    Eigen::Vector3f cross = crossProduct(A, B);

    const double epsilon = 1e-9;
    if (std::abs(cross[2]) < epsilon) {
        return 0.0; // Avoid division by zero
    }
    
    return cross[2]/std::abs(cross[2]);
}

double get_angular_deviation(double a, double b) 
{
        double diff = std::fmod(b - a, 2.0 * M_PI);
        if (diff > M_PI)
            diff -= 2.0 * M_PI;
        else if (diff < -M_PI)
            diff += 2.0 * M_PI;
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

std::vector<double> get_csv_column(const std::string& trajectory_csv, int column)
{
    int number_of_tokens_I_was_not_able_to_convert_to_double = 0;
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
                    if(number_of_tokens_I_was_not_able_to_convert_to_double == 0) // It's alright, It's the header
                    {
                        number_of_tokens_I_was_not_able_to_convert_to_double++;
                    }
                    else
                    {
                        std::cerr << "Conversion error for token: " << token << "\n";
                    }
                }
                break; // We only need the specified column, so break out of the inner loop.
            }
            ++currentColumn;
        }
    }
    
    return values;
}

std::tuple<double, Eigen::Vector2d> get_lateral_deviation_components(const double angular_dev, const double closest_point_tangent, const nav_msgs::msg::Odometry::SharedPtr msg)
{
    double Vx = msg->twist.twist.linear.x;
    double Vy = msg->twist.twist.linear.y;
    double lateral_deviation_speed = Vy*std::cos(angular_dev) + Vx*std::sin(angular_dev); // this is the speed of the car in the direction of the tangent line

    // Now reconstruct the perpendicular component into the original frame of reference in order to output it as a vector
    Eigen::Vector2d d_perp(-std::sin(closest_point_tangent), std::cos(closest_point_tangent));
    return {lateral_deviation_speed, lateral_deviation_speed * d_perp};
}

double get_feedforward_term(const double K_3, const double mass, const double long_speed, const double radius, const double frontal_lenght, const double rear_lenght, const double C_alpha_rear, const double C_alpha_front)
{
    double df_c1 = (mass*std::pow(long_speed,2))/(radius*(rear_lenght+frontal_lenght));
    double df_c2 = (frontal_lenght / (2*C_alpha_front))-(rear_lenght / (2*C_alpha_rear)) + (frontal_lenght / (2*C_alpha_rear))*K_3;
    double df_c3 = (rear_lenght+frontal_lenght)/radius;
    double df_c4 = (rear_lenght/radius)*K_3;
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
    return speed_difference * m_p + m_i * m_cumulative_error; // + m_d * (speed_difference - m_previous_speed_difference); lets just use a PI for now
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

    this->declare_parameter<int>("trajectory_oversampling_factor", 1);
    m_trajectory_oversampling_factor = this->get_parameter("trajectory_oversampling_factor").get_value<int>();

    this->declare_parameter<double>("search_radius", 1);
    m_param_search_radius = this->get_parameter("search_radius").get_value<double>();

    this->declare_parameter<double>("s_window", 5.0);
    m_param_s_window = this->get_parameter("s_window").get_value<double>();

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

    this->declare_parameter<double>("steering_ratio", 0.0);
    steering_ratio = this->get_parameter("steering_ratio").get_value<double>();

    // PID parameters
    this->declare_parameter<double>("PID_p", 0.0);
    m_p = this->get_parameter("PID_p").get_value<double>();

    this->declare_parameter<double>("PID_i", 0.0);
    m_i = this->get_parameter("PID_i").get_value<double>();

    this->declare_parameter<double>("PID_d", 0.0);
    m_d = this->get_parameter("PID_d").get_value<double>();
}

void LQR::load_trajectory()
{
    std::string package_share_directory = ament_index_cpp::get_package_share_directory("lqr");
    std::string trajectory_csv = m_csv_filename;

    // Build pointcloud from waypoints
    m_cloud = get_trajectory(package_share_directory+trajectory_csv); 

    // Build the KD-Tree once from pointcloud
    m_kdtree = std::make_unique<KDTreeType>(
            2, m_cloud, nanoflann::KDTreeSingleIndexAdaptorParams(10)
        );  
    m_kdtree->buildIndex();
    m_cloud_has_changed = false;

    // Get correspoinding columns from the csv file, the second parameter of the function is the column number
    m_points_s = get_csv_column(package_share_directory + trajectory_csv, 5);
    m_points_target_speed = get_csv_column(package_share_directory+trajectory_csv, 4);
    m_points_tangents = get_csv_column(package_share_directory+trajectory_csv, 3); 
    m_points_radii = get_csv_column(package_share_directory+trajectory_csv, 2); 
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
        auto qos_d = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);
        // Create odom publisher
        m_debug_pub = this->create_publisher<nav_msgs::msg::Odometry>(m_debug_topic, qos_d);
        m_debug_odom_pub = this->create_publisher<nav_msgs::msg::Odometry>(m_debug_odom_topic, qos_d);
    }
    m_cumulative_error = 0.0;
}


LQR::LQR() : Node("lqr_node") 
{
    this->initialize();
    RCLCPP_INFO(this->get_logger(), "INITIALIZED");
    this->load_trajectory();
    RCLCPP_INFO(this->get_logger(), "TRAJECTORY IS LOADED");

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
            exit(1);
        }
    }
}
    
void LQR::odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg) 
{
    // Save the actual time to compute the time needed for the execution later
    auto start = std::chrono::high_resolution_clock::now();

    Eigen::Vector2f odometry_pose(msg->pose.pose.position.x, msg->pose.pose.position.y);
    Odometry odometry = {odometry_pose, get_yaw(msg)};
    
    // Find closest point to trajectory using KD-Tree from NanoFLANN
    // Log to console how long it took to find the closest point
    size_t closest_point_index;
    double new_S;

    std::chrono::high_resolution_clock::time_point s = std::chrono::high_resolution_clock::now();

    try // using binary search along S
    {
        closest_point_index = get_closest_point_along_S(odometry_pose, m_param_s_window, new_S);
    }
    catch(const std::exception& e) // fallback to KD_tree search if binary search fails OSS: this approach will now work for skidpad
    {
        closest_point_index = get_closest_point_from_KD_Tree(odometry_pose, m_param_search_radius);
        new_S = m_points_s[closest_point_index];
    }

    std::chrono::high_resolution_clock::time_point e = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> seconds = e - s;
    RCLCPP_INFO(this->get_logger(), "Closest point search took %.2f ms", seconds.count() * 1000);

    m_S_prev = new_S; // update S of the car

    Eigen::Vector2f closest_point = m_cloud.pts[closest_point_index];

    // Calculate lateral deviation as distance between two points just to check if the closest point is correct
    double lateral_deviation = points_distance(odometry_pose, closest_point);

    // I have found the closest point on the trajectory to the odometry pose but I don't trust the result so I check if the previous or next point are closer to the odometry
    while(1)
    {
        if(closest_point_index > 0 && closest_point_index < m_cloud.pts.size() - 1)
        {
            double previuous_point_lateral_deviation = points_distance(odometry_pose, m_cloud.pts[closest_point_index - 1]);
            double next_point_lateral_deviation = points_distance(odometry_pose, m_cloud.pts[closest_point_index + 1]);
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

    // if(m_is_DEBUG)
    // {
    //     // dump the current odometry to a file with columns: odom.x, odom.y, odom.yaw
    //     // std::stringstream filename;
    //     // filename << "odometry_log.csv";

    //     // std::ofstream file(filename.str(), std::ios::app);
    //     // if (file.is_open()) {
    //     //     file << odometry_pose[0] << "," << odometry_pose[1] << "," << odometry.yaw << "\n";
    //     //     file.close();
    //     // } else {
    //     //     RCLCPP_ERROR(this->get_logger(), "Could not open %s for writing", filename.str().c_str());
    //     // }
    // }

    double closest_point_tangent = m_points_tangents[closest_point_index];

    // Now I want to calculate the lateral deviation again but this time not as the distance between two points but as the distance between the odometry pose and tangengt line of the closest point
    lateral_deviation = line_distance(odometry_pose, closest_point, closest_point_tangent);

    // Now i can use the dot product to compute a signed distance and use this sign to establish where I am w.r.t. the race line
    double lateral_position = get_sign(closest_point[0], closest_point[1], odometry_pose[0], odometry_pose[1], closest_point_tangent);
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

    // Now we compute the theoretical steering at the wheel
    double steering = -optimal_control_vector.dot(x); 

    // Now we need to calculate Vx and and the curvature radious
    double Vx = msg->twist.twist.linear.x;
    double R_c = m_points_radii[closest_point_index];

    // Now we compute the feedforward term
    double delta_f = get_feedforward_term(K_3, m_mass, Vx, R_c, front_length, rear_length, C_alpha_rear, C_alpha_front);
    
    steering = steering - delta_f; // this is my actual steering target at the wheel

    // NOTICE: the steering angle we just computed is the steering angle requested at the wheels. We need to actuate the steering wheel
    // How much do we have to turn the steering wheel to get the desired steering angle at the wheels?
    steering = steering * steering_ratio;

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
        RCLCPP_INFO(this->get_logger(), "overall duration: %ld ns", duration);
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
