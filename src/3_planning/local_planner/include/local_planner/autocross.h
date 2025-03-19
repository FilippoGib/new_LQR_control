#ifndef AUTOCROSS_H
#define AUTOCROSS_H

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "modules/DelaunayTri.hpp"
#include "modules/Visualization.hpp"
#include "modules/WayComputer.hpp"
#include "utils/Time.hpp"
#include "utils/Params.hpp"
#include "mmr_base/msg/race_status.hpp"
#include "mmr_common_functions/mmr_common_functions.h"
#include <unistd.h>

// #define DEBUG 1

class AutocrossPlanner
{
	public:
		AutocrossPlanner(const rclcpp::Node::SharedPtr &nh,
						 const rclcpp::Publisher<visualization_msgs::MarkerArray>::SharedPtr &markerCenterLinePub,
						 const rclcpp::Publisher<visualization_msgs::MarkerArray>::SharedPtr &markerCenterLineCompletedPub,
						 const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr &centerLinePub,
						 const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr &centerLineCompletedPub);

		
		void initialize();

		void raceStatusCallBack(mmr_base::msg::RaceStatus::SharedPtr raceStatus);

		void odometryCallback(nav_msgs::msg::Odometry::SharedPtr odometry);

		void slamConesCallback(mmr_base::msg::Marker::SharedPtr slamCones);

	private:
		rclcpp::Node::SharedPtr nh;
		rclcpp::Publisher<visualization_msgs::MarkerArray>::SharedPtr markerCenterLinePub;
		rclcpp::Publisher<visualization_msgs::MarkerArray>::SharedPtr markerCenterLineCompletedPub;
		rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr centerLinePub;
		rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr centerLineCompletedPub;

		int currentLap;
		bool idle = false; //when true the node is idle and only publishes centerline_completed in a transient local fashion 
		WayComputer *wayComputer;
		Params *params;
		Visualization *visualization;

		

};

#endif //AUTOCROSS_H