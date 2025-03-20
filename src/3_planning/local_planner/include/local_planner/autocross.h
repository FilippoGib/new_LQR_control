#ifndef AUTOCROSS_H
#define AUTOCROSS_H

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "common_msgs/msg/race_status.hpp"
#include "utils/Params.hpp"
#include "modules/WayComputer.hpp"
#include "modules/Visualization.hpp"
#include <unistd.h>

// #define DEBUG 1

using Point = geometry_msgs::msg::Point;

class AutocrossPlanner
{
	public:
		AutocrossPlanner(const rclcpp::Node::SharedPtr &nh,
						 const rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr &bordersPub,
						 const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr &centerLinePub,
						 const rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr &bordersCompletedPub,
						 const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr &centerLineCompletedPub);

		void raceStatusCallBack(common_msgs::msg::RaceStatus::SharedPtr raceStatus);

		void odometryCallback(nav_msgs::msg::Odometry::SharedPtr odometry);

		void slamConesCallback(visualization_msgs::msg::Marker::SharedPtr slamCones);


	private:
		rclcpp::Node::SharedPtr nh;
		rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr bordersPub;
		rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr centerLinePub;
		rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr bordersCompletedPub;
		rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr centerLineCompletedPub;

		visualization_msgs::msg::MarkerArray borders;
		visualization_msgs::msg::MarkerArray bordersCompleted;
		int currentLap;
		bool idle = false; //when true the node is idle and only publishes centerline_completed in a transient local fashion 
		
		Params *params;
		WayComputer *wayComputer;
		Visualization *visualization;

}

#endif //AUTOCROSS_H