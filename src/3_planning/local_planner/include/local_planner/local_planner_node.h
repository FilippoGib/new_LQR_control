#ifndef LOCAL_PLANNER_NODE_H
#define LOCAL_PLANNER_NODE_H

#include <rclcpp/rclcpp.hpp>
#include <rmw/qos_profiles.h>
#include "rcutils/time.h"
#include "rmw/macros.h"
#include "rmw/visibility_control.h"
#include "rmw/types.h"
#include "acceleration.h"
#include "autocross.h"
#include "skidpad.h"

using namespace std::chrono_literals;

class LocalPlannerNode : public rclcpp::Node
{
	public:
		LocalPlannerNode();
		void loadParameters();
		void initialization();

	private:
		rclcpp::TimerBase::SharedPtr timer;

		rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr  bordersPub;
		rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr  centerLinePub;
		rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr  bordersCompletedPub;
		rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr  centerLineCompletedPub;

		rclcpp::Subscription<common_msgs::msg::RaceStatus>::SharedPtr raceStatusSub;
    	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometrySub;
		rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr slamConesSub;

		std::shared_ptr<AccelerationPlanner> accelerationPlanner;
		std::shared_ptr<AutocrossPlanner> autocrossPlanner;
		std::shared_ptr<SkidpadPlanner> skidpadPlanner;

		std::string param_eventType;
		std::string param_topicBorders;
		std::string param_topicCenterLine;
		std::string param_topicBordersCompleted;
		std::string param_topicCenterLineCompleted;
		std::string param_topicRaceStatus;
		std::string param_topicOdometry;
		std::string param_topicSlamCones;
};

#endif //LOCAL_PLANNER_NODE_H