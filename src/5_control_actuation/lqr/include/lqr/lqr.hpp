#ifndef LQR_HPP_
#define LQR_HPP_

#include <rclcpp/rclcpp.hpp>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <chrono>
#include <filesystem>
#include <math.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <nav_msgs/msg/odometry.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <eigen3/Eigen/Geometry>
#include <vector>

#include "nanoflann/nanoflann.hpp"

#define _USE_MATH_DEFINES

struct Odometry {
    Eigen::Vector2f pose;
    double yaw;
};

struct Waypoint {
    Eigen::Vector2f pose;
    double tangent;
};

// KD-tree adaptor for nanoflann
struct PointCloud {
    std::vector<Eigen::Vector2f> pts;

    inline size_t kdtree_get_point_count() const { return pts.size(); }

    inline double kdtree_get_pt(const size_t idx, const size_t dim) const {
        return (dim == 0) ? pts[idx][0] : pts[idx][1];
    }

    template <class BBOX>
    bool kdtree_get_bbox(BBOX& /*bb*/) const { return false; }
};

class LQR : public rclcpp::Node {
public:
    LQR();

private:

    using KDTreeType = nanoflann::KDTreeSingleIndexAdaptor<
    nanoflann::L2_Simple_Adaptor<double, PointCloud>,
    PointCloud,
    2,                // dimensionality
    unsigned int      // index type
    >;

    // callbacks
    void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void partial_trajectory_callback(const visualization_msgs::msg::Marker traj); // during the first lap we work with the partial trajectory published by the local planner
    void global_trajectory_callback(); // same as what partial_trajectory_callback does but is subscribed to a different topic

    // methods
    void initialize();
    void load_parameters();
    void load_trajectory(); 
    size_t get_closest_point_from_KD_Tree(const Eigen::Vector2f& odometry_pose, double radius);
    size_t get_closest_point_along_S(const Eigen::Vector2f& odometry_pose, double window, double& out_S);
    Eigen::Vector4f find_optimal_control_vector(double speed);
    double calculate_torque(double speed, double target_speed, double dt);    

    // pubs and subs
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr m_odom_sub;
    rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr m_partial_traj_sub; // we will use it in the first_lap mode
    rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr m_global_traj_sub; // get the full trajectory once at the start of the second lap

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr m_debug_pub; // for visualization
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr m_debug_odom_pub; // actual state vector
    // N.B. this message is what the simulator needs, the actual message that will be used int the car might be different
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr m_control_pub; 
    
    //topics
    std::string m_odom_topic;
    std::string m_control_topic;
    std::string m_partial_traj_topic;
    std::string m_debug_topic;
    std::string m_debug_odom_topic;
   
    // class members
    std::vector<std::string> m_raw_vectors_k;
    std::vector<std::pair<double, std::vector<double>>> m_k_pair;
    std::vector<double> m_points_target_speed; // calculated offline on matlab always
    PointCloud m_cloud;
    std::unique_ptr<KDTreeType> m_kdtree;
    std::vector<double> m_u;
    std::vector<double> m_points_tangents;
    std::vector<double> m_points_radii;
    std::vector<double> m_points_s;
    double m_S_prev; // previus iteration S
    size_t m_old_closest_point_index; // previus iterazion closest point
    double m_param_s_window; // size of sliding window
    double m_param_max_dist; // maximum distance at which a point is considered the closest
    bool m_is_first_lap; // for now we statically decide if we want to use the partial trajectory or we want to use the global trajectory
    bool m_is_loaded;
    bool m_is_DEBUG;
    bool m_is_constant_speed;
    bool m_cloud_has_changed{false};
    double m_target_speed;
    double m_param_search_radius;
    int m_trajectory_oversampling_factor;
    double m_ds;
    std::string m_csv_filename;

    // car physical parameters
    double m_mass;
    double front_length;
    double rear_length;
    double C_alpha_front;
    double C_alpha_rear;
    double steering_ratio;

    // PID parameters
    double m_p;
    double m_i;
    double m_d;
    double m_cumulative_error;
    double m_max_integral; // limits the cumulative error so that it never explodes
    double m_dt;
};

#endif
