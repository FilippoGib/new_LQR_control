#include "autocross.h"
#include <iomanip>
#include <sstream>
#include <cstdio>

/// @brief constructor
/// @param nh node handler
/// @param bordersPub borders publisher
/// @param centerLinePub centerline publisher
/// @param bordersCompletedPub complete borders publisher
/// @param centerLineCompletedPub complete centerline publisher
AutocrossPlanner::AutocrossPlanner(const rclcpp::Node::SharedPtr &nh,
								   const rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr &bordersPub,
								   const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr &centerLinePub,
								   const rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr &bordersCompletedPub,
								   const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr &centerLineCompletedPub)
{
	this->nh = nh;

	this->bordersPub = bordersPub;
	this->centerLinePub = centerLinePub;
	this->bordersCompletedPub = bordersCompletedPub;
	this->centerLineCompletedPub = centerLineCompletedPub;

	this->currentLap = 0;

	params = new Params(nh);
  
	wayComputer = new WayComputer(params->wayComputer);
	Visualization::getInstance().init(nh, params->visualization);
}


/// @brief callback to get the race status from the subscription
/// @param raceStatus custom message
void AutocrossPlanner::raceStatusCallBack(common_msgs::msg::RaceStatus::SharedPtr raceStatus)
{
	this->currentLap = raceStatus->current_lap;
}

/// @brief callback to get the odometry from the subscription and initialize class parameters
/// @param odometry car position
void AutocrossPlanner::odometryCallback(nav_msgs::msg::Odometry::SharedPtr odometry)
{
	this->odometry.x = odometry->pose.pose.position.x;
	this->odometry.y = odometry->pose.pose.position.y;
}

void AutocrossPlanner::slamConesCallback(visualization_msgs::msg::Marker::SharedPtr slamCones)
{
	return;
}


