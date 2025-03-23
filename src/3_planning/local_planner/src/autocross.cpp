#include "autocross.h"
#include <iomanip>
#include <sstream>
#include <cstdio>

/// @brief constructor
/// @param nh node handler
/// @param centerLinePub centerline publisher
/// @param centerLineCompletedPub complete centerline publisher
AutocrossPlanner::AutocrossPlanner(const rclcpp::Node::SharedPtr &nh,
								   const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr &centerLinePub,
								   const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr &centerLineCompletedPub)
{
	this->nh = nh;

  // Notice how we don't care about the borders
	this->centerLinePub = centerLinePub;
	this->centerLineCompletedPub = centerLineCompletedPub;

	this->currentLap = 0;

	this->params = new Params(nh);
	this->wayComputer = new WayComputer(params->wayComputer);
	Visualization::getInstance().init(nh, params->visualization);

}


/// @brief callback to get the race status from the subscription
/// @param raceStatus custom message
void AutocrossPlanner::raceStatusCallBack(common_msgs::msg::RaceStatus::SharedPtr raceStatus)
{

	this->currentLap = raceStatus->current_lap;
}

void AutocrossPlanner::slamConesCallback(visualization_msgs::msg::Marker::SharedPtr slamCones)
{
	RCLCPP_INFO(rclcpp::get_logger(""), "[local_planner] slamConesCallback");

  if(this->idle)
  {
    RCLCPP_INFO(rclcpp::get_logger(""), "[local_planner] idle");
    return;
  }

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

  // Publish full trajectory and become idle
 if (this->currentLap > 1)
	{
		
		this->idle = true;
    for (int i = 0; i < 5; i ++)
		{
			this->centerLineCompletedPub->publish(this->wayComputer->getPathCenterLine()); // trancientlocal topic
    }
    return;
	}
  // Publish partial
  else
  {
    this->centerLinePub->publish(this->wayComputer->getPathCenterLine());
  }

  Time::tock("computation"); // End measuring time
}


