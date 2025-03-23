#include "acceleration.h"

/// @brief constructor
/// @param nh node handler
/// @param bordersPub borders publisher
/// @param centerLinePub centerline publisher
AccelerationPlanner::AccelerationPlanner(rclcpp::Node::SharedPtr nh,
										 rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr bordersPub,
										 rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr centerLinePub)
{
	this->nh = nh;

	this->bordersPub = bordersPub;
	this->centerLinePub = centerLinePub;

	loadParameters();
}

/// @brief loads the class parameters from the .yaml file
void AccelerationPlanner::loadParameters()
{
	nh->declare_parameter<double>("acceleration/lineWidth", 0.0);
	this->param_lineWidth = nh->get_parameter("acceleration/lineWidth").get_value<double>();
}

/// @brief callback to get the odometry from the subscription and initialize class parameters
/// @param odometry
void AccelerationPlanner::odometryCallback(nav_msgs::msg::Odometry::SharedPtr odometry)
{
	this->odometry.x = odometry->pose.pose.position.x;
	this->odometry.y = odometry->pose.pose.position.y;
}

/// @brief callback to get cones from subscription
/// @param slamCones cones from odometry  (x,y,z,red,green,blue)
void AccelerationPlanner::slamConesCallback(visualization_msgs::msg::Marker::SharedPtr slamCones)
{
	std::vector<geometry_msgs::msg::Point> slamConesL;
	std::vector<geometry_msgs::msg::Point> slamConesR;

	for (int i = 0; i < slamCones->points.size(); i ++)
	{
		geometry_msgs::msg::Point point;
		point.x = slamCones->points[i].x;
		point.y = slamCones->points[i].y;

		if (slamCones->points[i].y > 0)
			slamConesL.push_back(point);

		if(slamCones->points[i].y < 0)
			slamConesR.push_back(point);
	}

	if (slamConesL.size() < 2)
	{
		std::cout<<"Not enough left slamCones"<<std::endl;
		return;
	}
	if (slamConesR.size() < 2)
	{
		std::cout<<"Not enough right slamCones"<<std::endl;
		return;
	}

	double horizon;
	for (int i = 0; i < slamCones->points.size(); i ++)
	{
		if (i == 0 || slamCones->points[i].x > horizon)
			horizon = slamCones->points[i].x;
	}

	this->borders.markers.clear();

	std::vector<geometry_msgs::msg::Point> borderL = this->generateBorder(slamConesL,
													  true,
													  horizon);

	std::vector<geometry_msgs::msg::Point> borderR = this->generateBorder(slamConesR,
													  false,
													  horizon);

	this->bordersPub->publish(this->borders);

	this->generateCenterLine(borderL,
							 borderR);
}

/// @brief generates a list of equally distanced points on the line
/// @param pointStart beginning of the line
/// @param pointEnd end of the line
/// @param horizon furthest point (x coordinate) the car can see
/// @return list of points discretized from the line
std::vector<geometry_msgs::msg::Point> AccelerationPlanner::generateDiscretizedLine(const geometry_msgs::msg::Point &pointStart,
										           			    const geometry_msgs::msg::Point &pointEnd,
																const double &horizon)
{
	std::vector<geometry_msgs::msg::Point> discretizedPoints;

	double slope = (pointStart.y - pointEnd.y) / (pointStart.x - pointEnd.x); // m in the line equation
	double intercept = (pointEnd.x * pointStart.y - pointStart.x * pointEnd.y) / (pointEnd.x - pointStart.x); // q in the line equation

	int displacement = 0;
	// creating the border by populating the "virtual" line generated before
	do
	{
		geometry_msgs::msg::Point point;
		point.x = odometry.x + displacement;
		point.y = slope * point.x + intercept;

		discretizedPoints.push_back(point);

		displacement ++;
	}
	while (odometry.x + displacement < horizon);

	return discretizedPoints;
}

/// @brief it finds the points that can better approximate the borders via a RANSAC algorithm
/// @param slamCones only left cones | only right cones from slamCones
/// @param borderType left | right [true | false]
/// @param horizon furthest point (x coordinate) the car can see
/// @return pair of points that approximates the border the best
std::vector<geometry_msgs::msg::Point> AccelerationPlanner::generateBorder(const std::vector<geometry_msgs::msg::Point> &slamCones,
													   const bool &borderType,
													   const double &horizon)
{
	visualization_msgs::msg::Marker border;
	border.id = borderType;
	border.ns = "border";
	border.header.frame_id = "track";
	border.header.stamp = this->nh->get_clock()->now();
	border.type = visualization_msgs::msg::Marker::LINE_STRIP;
	border.action = visualization_msgs::msg::Marker::ADD;
	border.scale.x = 0.1;
	border.scale.y = 0.1;
	border.scale.z = 0.1;
	border.pose.orientation.w = 1.0;
	border.pose.orientation.x = 0.0;
	border.pose.orientation.y = 0.0;
	border.pose.orientation.z = 0.0;
	border.color.a = 1.0;

	if (borderType)
	{
		border.color.r = 0.0;
		border.color.g = 0.0;
		border.color.b = 1.0;
	}
	else
	{
		border.color.r = 1.0;
		border.color.g = 1.0;
		border.color.b = 0.0;
	}

	std::pair<geometry_msgs::msg::Point, geometry_msgs::msg::Point> borderPair; // pair of points generating the border line

	int minLineCost = slamCones.size();

	// ransack algorithm to find the best straight line that approximate the current border
	for (int i = 0; i < slamCones.size() - 1; i ++)
	{
		geometry_msgs::msg::Point p1 = slamCones[i];

		for (int j = i + 1; j < slamCones.size(); j ++)
		{
			geometry_msgs::msg::Point p2 = slamCones[j];

			int lineCost = 0;

			for (int k = 0; k < slamCones.size(); k ++)
			{
				if (k == i || k == j)
					continue;

				geometry_msgs::msg::Point p3 = slamCones[k];

				double numerator = std::abs(((p2.x - p1.x) * (p1.y - p3.y)) - ((p1.x - p3.x) * (p2.y - p1.y)));
            	double denominator = std::sqrt((p2.x - p1.x) * (p2.x - p1.x) + (p2.y - p1.y) * (p2.y - p1.y));
				double distance = numerator / denominator;

				if (distance > this->param_lineWidth)
				{
					lineCost ++;

					if (lineCost > minLineCost)
						break;
				}
			}
			if (lineCost < minLineCost)
			{
				borderPair.first = p1;
				borderPair.second = p2;

				minLineCost = lineCost;
			}
		}
	}

	border.points = this->generateDiscretizedLine(borderPair.first,
												  borderPair.second,
												  horizon);

	this->borders.markers.push_back(border);

	return border.points; //ritorno un vettore di punti anzichè la coppia di punti
}

/// @brief generates a pair of points that approximates the centerline the best
/// @param borderL left border
/// @param borderR right border
/// @param horizon furthest point (x coordinate) the car can see
void AccelerationPlanner::generateCenterLine(const std::vector<geometry_msgs::msg::Point> &borderL,
											 const std::vector<geometry_msgs::msg::Point> &borderR)
{
	visualization_msgs::msg::Marker centerLine;
	centerLine.id = 0;
	centerLine.ns = "centerLine";
	centerLine.header.frame_id = "track";
	centerLine.header.stamp = this->nh->get_clock()->now();
	centerLine.type = visualization_msgs::msg::Marker::LINE_STRIP;
	centerLine.action = visualization_msgs::msg::Marker::ADD;
	centerLine.scale.x = 0.1;
	centerLine.scale.y = 0.1;
	centerLine.scale.z = 0.1;
	centerLine.color.a = 1.0;
	centerLine.color.r = 0.0;
	centerLine.color.g = 1.0;
	centerLine.color.b = 0.0;
	centerLine.pose.orientation.w = 1.0;
	centerLine.pose.orientation.x = 0.0;
	centerLine.pose.orientation.y = 0.0;
	centerLine.pose.orientation.z = 0.0;

	std::vector<geometry_msgs::msg::Point> longerBorder;
	std::vector<geometry_msgs::msg::Point> shorterBorder;

	if (borderL.size() > borderR.size())
	{
		longerBorder = borderL;
		shorterBorder = borderR;
	}
	else
	{
		longerBorder = borderR;
		shorterBorder = borderL;
	}

	std::vector<geometry_msgs::msg::Point> centerPoints;
	centerPoints.push_back(this->odometry);

	double minOdometryDistance; // distance of the closest odometry point on the longerBorder
	double minOdometryIndex;

	for (int i = 0; i < longerBorder.size(); i ++)
	{
		double distance = (this->odometry.x - longerBorder[i].x) * (this->odometry.x - longerBorder[i].x) +
						  (this->odometry.y - longerBorder[i].y) * (this->odometry.y - longerBorder[i].y);

		if (i == 0 || distance < minOdometryDistance)
		{
			minOdometryDistance = distance;
			minOdometryIndex = i;
		}
	}

	if (longerBorder[minOdometryIndex].x < this->odometry.x)
	{
		minOdometryIndex ++;
	}

	for (int i = minOdometryIndex; i < longerBorder.size(); i ++)
	{
		double minDistance;
		double minIndex;

		for (int j = 0; j < shorterBorder.size(); j ++)
		{
			double distance = (longerBorder[i].x - shorterBorder[j].x) * (longerBorder[i].x - shorterBorder[j].x) +
						  	  (longerBorder[i].y - shorterBorder[j].y) * (longerBorder[i].y - shorterBorder[j].y);

			if (j == 0 || distance < minDistance)
			{
				minDistance = distance;
				minIndex = j;
			}
		}

		geometry_msgs::msg::Point point;
		point.x = (longerBorder[i].x + shorterBorder[minIndex].x) / 2.0;
		point.y = (longerBorder[i].y + shorterBorder[minIndex].y) / 2.0;

		centerPoints.push_back(point);
	}

	centerLine.points  = centerPoints;
	this->centerLinePub->publish(centerLine);

	return;
}