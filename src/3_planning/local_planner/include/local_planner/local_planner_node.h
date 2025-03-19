#ifndef LOCAL_PLANNER_NODE_H
#define LOCAL_PLANNER_NODE_H

#include <rclcpp/rclcpp.hpp>
#include <rmw/qos_profiles.h>
#include "rcutils/time.h"
#include "rmw/macros.h"
#include "rmw/visibility_control.h"
#include "rmw/types.h"
#include <nav_msgs/msg/path.hpp>
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

		// La centerline e la centerline completed sono usati dal controllo quindi gli mandiamo dei nav_msgs::msg::Path
		rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr  centerLinePub;
		rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr  centerLineCompletedPub;
		
		// Questa roba invece è solo per la visualizzazione
		rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr  markerBordersPub;
		rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr  markerBordersCompletedPub;
		rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr  markerCenterLinePub;
		rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr  markerCenterLineCompletedCompletedPub;

		// Questa roba viene fatta a monte del local planner quindi laciamo questi tipi di messaggi
		rclcpp::Subscription<mmr_base::msg::RaceStatus>::SharedPtr raceStatusSub;
    	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometrySub;
		rclcpp::Subscription<mmr_base::msg::Marker>::SharedPtr slamConesSub;

		// Classi di utility
		std::shared_ptr<AccelerationPlanner> accelerationPlanner;
		std::shared_ptr<AutocrossPlanner> autocrossPlanner;
		std::shared_ptr<SkidpadPlanner> skidpadPlanner;

		// Missione
		std::string param_eventType;

		// Nomi dei topic
		std::string param_topicCenterLine;
		std::string param_topicCenterLineCompleted;

		std::string param_topicMarkerBorders;
		std::string param_topicMarkerBordersCompleted;
		std::string param_topicMarkerCenterLine;
		std::string param_topicMarkerCenterLineCompleted;

		std::string param_topicRaceStatus;
		std::string param_topicOdometry;
		std::string param_topicSlamCones;
};

#endif //LOCAL_PLANNER_NODE_H