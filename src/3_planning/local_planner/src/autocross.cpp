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
								   const rclcpp::Publisher<mmr_base::msg::MarkerArray>::SharedPtr &bordersPub,
								   const rclcpp::Publisher<mmr_base::msg::Marker>::SharedPtr &centerLinePub,
								   const rclcpp::Publisher<mmr_base::msg::MarkerArray>::SharedPtr &bordersCompletedPub,
								   const rclcpp::Publisher<mmr_base::msg::Marker>::SharedPtr &centerLineCompletedPub)
{
	this->nh = nh;

	this->bordersPub = bordersPub;
	this->centerLinePub = centerLinePub;
	this->bordersCompletedPub = bordersCompletedPub;
	this->centerLineCompletedPub = centerLineCompletedPub;

	this->currentLap = 0;

	this->initialize();

	this->loadParameters();
}

/// @brief inizialize utility classes
void AutocrossPlanner::initialize()
{
	this->params = new Params(this->nh); // no need for loadparameters()
	this->wayComputer = new WayComputer(this->params->wayComputer);
	this->visualization = new Visualization(this->nh, this->params->visualization);
	Visualization::getInstance().init(nh, params->visualization);
	// Since the odometry subscription is binded to a function of the WayComputer class, it is created here and not in the node
	auto subOdometry = nh->create_subscription<nav_msgs::msg::Odometry>(params->main.input_pose_topic, 1, std::bind(&WayComputer::stateCallback, wayComputer, std::placeholders::_1));
}

/// @brief callback to get the race status from the subscription
/// @param raceStatus custom message
void AutocrossPlanner::raceStatusCallBack(mmr_base::msg::RaceStatus::SharedPtr raceStatus)
{
	this->currentLap = raceStatus->current_lap;
}

/// @brief callback to get cones from subscription
/// @param slamCones cones from odometry  (x,y,z,red,green,blue)
void AutocrossPlanner::slamConesCallback(mmr_base::msg::Marker::SharedPtr slamCones)
{
	RCLCPP_INFO(rclcpp::get_logger(""), "[local_planner] slamConesCallBack callback");

	if (!wayComputer->isLocalTfValid())
	{
	  RCLCPP_INFO(rclcpp::get_logger(""), "[local_planner] ERROR: Odometry not valid");
	  return;
	}
	if (data->points.empty())
	{
	  RCLCPP_INFO(rclcpp::get_logger(""), "[local_planner] ERROR: No cones received");
	  return;
	}

	Time::tick("computation"); // Start measuring time
	int id = 0;
	//Convert to Node vector
	std::vector<Node> nodes;
	nodes.reserve(data->points.size());
	for (const geometry_msgs::msg::Point c: data->points)
	{
	  // if (c.confidence >= params->main.min_cone_confidence)
	  RCLCPP_INFO(rclcpp::get_logger(""), "[local_planner] point from slam cones: x = %f, y = %f", c.x, c.y);
	  Node n = Node(static_cast<double>(c.x), static_cast<double>(c.y),static_cast<double>(c.x), static_cast<double>(c.y), id++);
	  nodes.push_back(n);
	}

	RCLCPP_INFO(rclcpp::get_logger(""), "[local_planner] the size of nodes is %ld", nodes.size()); //nodes is not empty

	// Update local coordinates of Nodes (makes original local coords unnecessary)
	for (const Node &n : nodes)
	{
	  n.updateLocal(wayComputer->getLocalTf());
	  RCLCPP_INFO(rclcpp::get_logger(""), "[local_planner] node: x = %f, y = %f", n.x(), n.y());
	}

	// Delaunay triangulation
	TriangleSet triangles = DelaunayTri::compute(nodes);

	RCLCPP_INFO(rclcpp::get_logger(""), "[local_planner] the size of triangles is %ld", triangles.size()); //triangles is empty

	// Update the way with the new triangulation
	wayComputer->update(triangles, data->header.stamp);

	// Publish loop and write tracklimits to a file
	if (wayComputer->isLoopClosed())
	{
	  this->centerLineCompletedPub->publish(wayComputer->getPathCenterLine());
	//   pubFullBorderLeft->publish(wayComputer->getPathBorderLeft());
	//   pubFullBorderRight->publish(wayComputer->getPathBorderRight());

	  RCLCPP_INFO(rclcpp::get_logger(""), "[local_planner] Tanco loop");
	  std::string loopDir = params->main.package_path + "/loops";
	  mkdir(loopDir.c_str(), 0777);
	  wayComputer->writeWayToFile(loopDir + "/loop.unay");
	  if (params->main.shutdown_on_loop_closure)
	  {
	    Time::tock("computation"); // End measuring time
	    rclcpp::shutdown();
	  }
	}
	// Publish partial
	else
	{
	  this->centerLinePub->publish(wayComputer->getPathCenterLine());
	//   pubPartialBorderLeft->publish(wayComputer->getPathBorderLeft());
	//   pubPartialBorderRight->publish(wayComputer->getPathBorderRight());
	}

	Time::tock("computation"); // End measuring time
}