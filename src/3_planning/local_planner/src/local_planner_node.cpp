#include "local_planner_node.h"

/// @brief node constructor. It creates a timer to try to call initialization() every 500 ms.
LocalPlannerNode::LocalPlannerNode() : Node("local_planner")
{
	RCLCPP_INFO(this->get_logger(), "LOCAL PLANNER NODE CREATED");

	this->timer = this->create_wall_timer(500ms, std::bind(&LocalPlannerNode::initialization, this));
}

/// @brief loads the class parameters from the .yaml file
void LocalPlannerNode::loadParameters()
{
	this->declare_parameter<std::string>("node/eventType", "");
	this->param_eventType = this->get_parameter("node/eventType").get_value<std::string>();

	this->declare_parameter<std::string>("node/topicBorders", "");
	this->param_topicBorders = this->get_parameter("node/topicBorders").get_value<std::string>();

	this->declare_parameter<std::string>("node/topicCenterLine", "");
	this->param_topicCenterLine = this->get_parameter("node/topicCenterLine").get_value<std::string>();

	this->declare_parameter<std::string>("node/topicBordersCompleted", "");
	this->param_topicBordersCompleted = this->get_parameter("node/topicBordersCompleted").get_value<std::string>();

	this->declare_parameter<std::string>("node/topicCenterLineCompleted", "");
	this->param_topicCenterLineCompleted = this->get_parameter("node/topicCenterLineCompleted").get_value<std::string>();

	this->declare_parameter<std::string>("node/topicRaceStatus", "");
	this->param_topicRaceStatus = this->get_parameter("node/topicRaceStatus").get_value<std::string>();

	this->declare_parameter<std::string>("node/topicOdometry", "");
	this->param_topicOdometry = this->get_parameter("node/topicOdometry").get_value<std::string>();

	this->declare_parameter<std::string>("node/topicSlamCones", "");
	this->param_topicSlamCones = this->get_parameter("node/topicSlamCones").get_value<std::string>();
}

/// @brief initializes the node. It creates the selected planner based on the eventType parameter with its publishers and its subscribers
void LocalPlannerNode::initialization()
{
	this->loadParameters();

	if (this->param_eventType.empty())
	{
		RCLCPP_INFO(this->get_logger(), "NO EVENT TYPE SELECTED");

		this->timer->cancel();

		rclcpp::shutdown();
	}

	rclcpp::QoS qos(rclcpp::KeepLast(1));
	qos.reliable();
	qos.transient_local();
	// qos.deadline(rclcpp::Duration::from_seconds(1e9));
	// qos.lifespan(rclcpp::Duration::from_seconds(1e9));
	// qos.liveliness(RMW_QOS_POLICY_LIVELINESS_AUTOMATIC);
	// qos.liveliness_lease_duration(rclcpp::Duration::from_seconds(1e9));

	this->bordersPub = this->create_publisher<visualization_msgs::msg::MarkerArray>(this->param_topicBorders, 1);
	this->centerLinePub = this->create_publisher<visualization_msgs::msg::Marker>(this->param_topicCenterLine, 1);
	this->bordersCompletedPub = this->create_publisher<visualization_msgs::msg::MarkerArray>(this->param_topicBordersCompleted, 1);
	this->centerLineCompletedPub = this->create_publisher<visualization_msgs::msg::Marker>(this->param_topicCenterLineCompleted, qos);

	if (this->param_eventType == "acceleration")
	{
		RCLCPP_INFO(this->get_logger(), "INSTANTIATING ACCELERATION PLANNER");

		this->accelerationPlanner = std::make_shared<AccelerationPlanner>(shared_from_this(), this->bordersPub, this->centerLinePub);

		this->odometrySub = this->create_subscription<nav_msgs::msg::Odometry>(this->param_topicOdometry, 1, std::bind(&AccelerationPlanner::odometryCallback, this->accelerationPlanner, std::placeholders::_1));
		this->slamConesSub = this->create_subscription<visualization_msgs::msg::Marker>(this->param_topicSlamCones, 1, std::bind(&AccelerationPlanner::slamConesCallback, this->accelerationPlanner, std::placeholders::_1));
	}
	else if (this->param_eventType == "autocross")
	{
		RCLCPP_INFO(this->get_logger(), "INSTANTIATING AUTOCROSS PLANNER");

		this->autocrossPlanner = std::make_shared<AutocrossPlanner>(shared_from_this(), this->centerLinePub, this->centerLineCompletedPub);

		this->raceStatusSub = this->create_subscription<common_msgs::msg::RaceStatus>(this->param_topicRaceStatus, 1, std::bind(&AutocrossPlanner::raceStatusCallBack, this->autocrossPlanner, std::placeholders::_1));
		this->odometrySub = this->create_subscription<nav_msgs::msg::Odometry>(this->param_topicOdometry, 1, std::bind(&WayComputer::stateCallback, this->autocrossPlanner->wayComputer, std::placeholders::_1));
		this->slamConesSub = this->create_subscription<visualization_msgs::msg::Marker>(this->param_topicSlamCones, 1, std::bind(&AutocrossPlanner::slamConesCallback, this->autocrossPlanner, std::placeholders::_1));
	}
	else if (this->param_eventType == "skidpad")
	{
		RCLCPP_INFO(this->get_logger(), "INSTANTIATING SKIDPAD PLANNER");

		this->skidpadPlanner = std::make_shared<SkidpadPlanner>(shared_from_this(), this->bordersPub, this->centerLinePub);

		this->raceStatusSub = this->create_subscription<common_msgs::msg::RaceStatus>(this->param_topicRaceStatus, 1, std::bind(&SkidpadPlanner::raceStatusCallBack, this->skidpadPlanner, std::placeholders::_1));
		this->odometrySub = this->create_subscription<nav_msgs::msg::Odometry>(this->param_topicOdometry, 1, std::bind(&SkidpadPlanner::odometryCallback, this->skidpadPlanner, std::placeholders::_1));
		this->slamConesSub = this->create_subscription<visualization_msgs::msg::Marker>(this->param_topicSlamCones, 1, std::bind(&SkidpadPlanner::slamConesCallback, this->skidpadPlanner, std::placeholders::_1));
	}
	else
	{
		RCLCPP_INFO(this->get_logger(), "INVALID EVENT TYPE SELECTED");

		this->timer->cancel();

		rclcpp::shutdown();
	}

	this->timer->cancel();
}

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);

	auto localPlannerNode = std::make_shared<LocalPlannerNode>();

	rclcpp::spin(localPlannerNode);

	return 0;
}