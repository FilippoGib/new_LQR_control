#ifndef LIMO_WRAPPER_HPP
#define LIMO_WRAPPER_HPP

#include "ROSutils.hpp"

namespace ros2wrap {

class LimoWrapper : public rclcpp::Node {
public:
    // Constructor
    LimoWrapper();

    // Public member variables
    std::string world_frame;
    std::string body_frame;

private:
    // Subscriber objects
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

    // Main publishers
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_pub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr state_pub;

    // Debug publishers
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr orig_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr desk_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr match_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr finalraw_pub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr body_pub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr map_bb_pub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr match_points_pub;

    // TF Broadcaster
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // Callback methods
    void lidar_callback(const sensor_msgs::msg::PointCloud2& msg);
    void imu_callback(const sensor_msgs::msg::Imu& msg);

    // Configuration loading method
    void loadConfig(fast_limo::Config* config);
    // Visualization methods
    visualization_msgs::msg::Marker getLocalMapMarker(BoxPointType bb);
    visualization_msgs::msg::MarkerArray getMatchesMarker(Matches& matches, 
                                                          std::string frame_id);
};

// Conversion utility methods
void fromROStoLimo(const sensor_msgs::msg::Imu& in, fast_limo::IMUmeas& out);
void fromROStoLimo(const nav_msgs::msg::Odometry::SharedPtr& in, fast_limo::State& out);
void fromLimoToROS(const fast_limo::State& in, nav_msgs::msg::Odometry& out);
void fromLimoToROS(const fast_limo::State& in, 
                    const std::vector<double>& cov_pose,
                    const std::vector<double>& cov_twist, 
                    nav_msgs::msg::Odometry& out);

// TF broadcasting method
geometry_msgs::msg::TransformStamped broadcastTF(const fast_limo::State& in, 
                    std::string parent_name, 
                    std::string child_name, 
                    bool now);


} // namespace ros2wrap

#endif // LIMO_WRAPPER_HPP