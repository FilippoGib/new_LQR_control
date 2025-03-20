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

	this->params = new Params(nh);
  
	this->wayComputer = new WayComputer(params->wayComputer);
	// Visualization::getInstance().init(nh, params->visualization);

	// auto subPose = nh->create_subscription<nav_msgs::msg::Odometry>("/Odometry", 1, std::bind(&WayComputer::stateCallback, wayComputer, std::placeholders::_1));
  // RCLCPP_INFO(rclcpp::get_logger(""), "[local_planner] Subscribed to Odometry");
}


/// @brief callback to get the race status from the subscription
/// @param raceStatus custom message
void AutocrossPlanner::raceStatusCallBack(common_msgs::msg::RaceStatus::SharedPtr raceStatus)
{
	this->currentLap = raceStatus->current_lap;
}

void AutocrossPlanner::slamConesCallback(visualization_msgs::msg::Marker::SharedPtr slamCones)
{
	RCLCPP_INFO(rclcpp::get_logger(""), "[local_planner] CCAT callback");

  if (!this->wayComputer->isLocalTfValid())
  {
    RCLCPP_INFO(rclcpp::get_logger(""), "[local_planner] CarState not being received.");
    return;
  }
  if (slamCones->points.empty())
  {
    RCLCPP_INFO(rclcpp::get_logger(""), "[local_planner] reading empty set of cones.");
    return;
  }

  Time::tick("computation"); // Start measuring time
  int id = 0;
  //Convert to Node vector
  std::vector<Node> nodes;
  nodes.reserve(slamCones->points.size());
  for (const geometry_msgs::msg::Point c: slamCones->points)
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
  this->wayComputer->update(triangles, slamCones->header.stamp);

  // Publish loop and write tracklimits to a file
 if (false)
	{
		// TODO: implement full trajectory publishing
//     this->centerLineCompletedPub->publish(wayComputer->getPathCenterLine());
//     // pubFullBorderLeft->publish(wayComputer->getPathBorderLeft());
//     // pubFullBorderRight->publish(wayComputer->getPathBorderRight());

//     RCLCPP_INFO(rclcpp::get_logger(""), "[local_planner] Tanco loop");
//     std::string loopDir = params->main.package_path + "/loops";
//     mkdir(loopDir.c_str(), 0777);
//     wayComputer->writeWayToFile(loopDir + "/loop.unay");
//     if (params->main.shutdown_on_loop_closure)
//     {
//       Time::tock("computation"); // End measuring time
//       rclcpp::shutdown();
//     }
		return;
	}
  // Publish partial
  else
  {
    this->centerLinePub->publish(this->wayComputer->getPathCenterLine());
    // pubPartialBorderLeft->publish(wayComputer->getPathBorderLeft());
    // pubPartialBorderRight->publish(wayComputer->getPathBorderRight());
  }

  Time::tock("computation"); // End measuring time
}


