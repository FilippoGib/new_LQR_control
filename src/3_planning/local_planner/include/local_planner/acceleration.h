#ifndef ACCELERATION_H
#define ACCELERATION_H

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "common_msgs/msg/race_status.hpp"
#include <unistd.h>

using Point = geometry_msgs::msg::Point;

class AccelerationPlanner
{
	public:
		AccelerationPlanner(rclcpp::Node::SharedPtr nh,
							rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr bordersPub,
							rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr centerLinePub);

		void loadParameters();

		void odometryCallback(nav_msgs::msg::Odometry::SharedPtr odometry);

		void slamConesCallback(visualization_msgs::msg::Marker::SharedPtr slamCones);

		std::vector<Point> generateDiscretizedLine(const Point &pointStart,
										           const Point &pointEnd,
												   const double &horizon);

		std::vector<Point> generateBorder(const std::vector<Point> &slamCones,
										  const bool &borderType,
										  const double &horizon);

		void generateCenterLine(const std::vector<Point> &borderL,
								const std::vector<Point> &borderR);

	private:
		rclcpp::Node::SharedPtr nh;
		rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr bordersPub;
		rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr centerLinePub;

		visualization_msgs::msg::MarkerArray borders;
		Point odometry;

		double param_lineWidth;
};

#endif //ACCELERATION_H