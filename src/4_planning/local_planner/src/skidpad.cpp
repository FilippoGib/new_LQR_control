#include "skidpad.h"

/// @brief constructor
/// @param nh node handler
/// @param bordersPub borders publisher
/// @param centerLinePub centerline publisher
SkidpadPlanner::SkidpadPlanner(const rclcpp::Node::SharedPtr &nh,
                               const rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr &bordersPub,
                               const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr &centerLinePub)
{
    this->nh = nh;

    this->bordersPub = bordersPub;
    this->centerLinePub = centerLinePub;

    this->raceStatus.current_lap = 0;

	loadParameters();

	this->firstAlign = false;
    this->lastOdometry = 0;

	this->circleL.x = this->param_centerX;
	this->circleL.y = this->param_centerY;

	this->circleR.x = this->param_centerX;
	this->circleR.y = -this->param_centerY;
}

/// @brief loads the class parameters from the .yaml file
void SkidpadPlanner::loadParameters()
{
	nh->declare_parameter<double>("skidpad/centerX", 0.0);
	this->param_centerX = nh->get_parameter("skidpad/centerX").get_value<double>();

	nh->declare_parameter<double>("skidpad/centerY", 0.0);
	this->param_centerY = nh->get_parameter("skidpad/centerY").get_value<double>();

	nh->declare_parameter<double>("skidpad/innerRadius", 0.0);
	this->param_innerRadius = nh->get_parameter("skidpad/innerRadius").get_value<double>();

	nh->declare_parameter<double>("skidpad/centerRadius", 0.0);
	this->param_centerRadius = nh->get_parameter("skidpad/centerRadius").get_value<double>();

	nh->declare_parameter<double>("skidpad/outerRadius", 0.0);
	this->param_outerRadius = nh->get_parameter("skidpad/outerRadius").get_value<double>();

	nh->declare_parameter<double>("skidpad/firstAlignCenterError", 0.0);
	this->param_firstAlignCenterError = nh->get_parameter("skidpad/firstAlignCenterError").get_value<double>();

	nh->declare_parameter<double>("skidpad/firstAlignRadiusError", 0.0);
	this-> param_firstAlignRadiusError = nh->get_parameter("skidpad/firstAlignRadiusError").get_value<double>();

	nh->declare_parameter<double>("skidpad/errorIncrement", 0.0);
	this->param_errorIncrement = nh->get_parameter("skidpad/errorIncrement").get_value<double>();

	nh->declare_parameter<double>("skidpad/centerError", 0.0);
	this->param_centerError= nh->get_parameter("skidpad/centerError").get_value<double>();

	nh->declare_parameter<double>("skidpad/radiusError", 0.0);
	this->param_radiusError = nh->get_parameter("skidpad/radiusError").get_value<double>();

	nh->declare_parameter<double>("skidpad/radiusOffsetL", 0.0);
	this-> param_radiusOffsetL = nh->get_parameter("skidpad/radiusOffsetL").get_value<double>();

	nh->declare_parameter<double>("skidpad/radiusOffsetR", 0.0);
	this->param_radiusOffsetR = nh->get_parameter("skidpad/radiusOffsetR").get_value<double>();

	nh->declare_parameter<int>("skidpad/circlePoints", 0);
	this->param_circlePoints = nh->get_parameter("skidpad/circlePoints").get_value<int>();

	nh->declare_parameter<int>("skidpad/linePoints", 0);
	this->param_linePoints = nh->get_parameter("skidpad/linePoints").get_value<int>();
}

/// @brief callback to get the race status from the subscription
/// @param raceStatus custom message
void SkidpadPlanner::raceStatusCallBack(common_msgs::msg::RaceStatus::SharedPtr raceStatus)
{
	this->raceStatus.current_lap = raceStatus->current_lap;
}

/// @brief callback to get the odometry from the subscription and initialize class parameters
/// @param odometry
void SkidpadPlanner::odometryCallback(nav_msgs::msg::Odometry::SharedPtr odometry)
{
	this->odometry.x = odometry->pose.pose.position.x;
	this->odometry.y = odometry->pose.pose.position.y;
}

/// @brief callback to get cones from subscription
/// @param slamCones cones from odometry  (x,y,z,red,green,blue)
void SkidpadPlanner::slamConesCallback(visualization_msgs::msg::Marker::SharedPtr slamCones)
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

		if (slamCones->points[i].y < 0)
			slamConesR.push_back(point);
	}

	if (slamConesL.size() < 3)
	{
		std::cout<<"Not enough left slamCones"<<std::endl;
		return;
	}
	if (slamConesR.size() < 3)
	{
		std::cout<<"Not enough right slamCones"<<std::endl;
		return;
	}

	this->borders.markers.clear();

    this->generateBorder(slamConesL,
						 true);

	this->generateBorder(slamConesR,
						 false);

	this->bordersPub->publish(this->borders);
	this->firstAlign = true;

	this->generateCenterLine();
}

/// @brief generates a list of equally distanced points on the line
/// @param pointStart beginning of the line
/// @param pointEnd end of the line
/// @return list of points discretized from the line
std::vector<geometry_msgs::msg::Point> SkidpadPlanner::generateDiscretizedLine(const geometry_msgs::msg::Point &pointStart,
										                   const geometry_msgs::msg::Point &pointEnd)
{
	std::vector<geometry_msgs::msg::Point> discretizedPoints;

	double lengthLng = pointEnd.x - pointStart.x;
	double lengthLat = pointEnd.y - pointStart.y;
	double sampleLng = lengthLng / this->param_linePoints;
	double sampleLat = lengthLat / this->param_linePoints;

	for (int i = 0; i < this->param_linePoints; i ++)
	{
		geometry_msgs::msg::Point point;
		point.x = pointStart.x + sampleLng * i;
		point.y = pointStart.y + sampleLat * i;
		point.z = 0.0;

		discretizedPoints.push_back(point);
	}

	return discretizedPoints;
}

/// @brief generates a list of equally distanced points on the given circle
/// @param circle circle (struct)
/// @param borderType left | right [true | false]
/// @return vector of points
std::vector<geometry_msgs::msg::Point> SkidpadPlanner::generateDiscretizedCircle(const SkidpadCircle &circle,
                                                             const bool &borderType)
{
	std::vector<geometry_msgs::msg::Point> discretizedPoints;

	if (borderType)
	{
		for (int i = 0; i < this->param_circlePoints; i ++)
		{
			geometry_msgs::msg::Point point;
			point.x = circle.x + circle.r  * std::cos((((2.0 * M_PI) / this->param_circlePoints) * i) - M_PI_2);
			point.y = circle.y + circle.r  * std::sin((((2.0 * M_PI) / this->param_circlePoints) * i) - M_PI_2);
			point.z = 0.0;

			discretizedPoints.push_back(point);
		}
	}
	else
	{
		for (int i = 0; i < this->param_circlePoints; i ++)
		{
			geometry_msgs::msg::Point point;
			point.x = circle.x + circle.r  * std::cos((((2.0 * M_PI) / this->param_circlePoints) * i) + M_PI_2);
			point.y = circle.y + circle.r  * std::sin((((2.0 * M_PI) / this->param_circlePoints) * i) + M_PI_2);
			point.z = 0.0;

			discretizedPoints.push_back(point);
		}

		std::reverse(discretizedPoints.begin(),
					 discretizedPoints.end());
	}

	return discretizedPoints;
}

/// @brief finds a circle based on a vector of points (all left-side | all right-side) three at a time. It sets the new center to the class variable
/// @param slamCones cones from odometry  (x,y,z,red,green,blue)
/// @param borderType left | right [true | false]
/// @param centerError tolerated error on the new center position relative to the last iteration
/// @param radiusError tolerated error on the new radius relative to the supposed value (from parameter)
void SkidpadPlanner::generateCircleCenter(const std::vector<geometry_msgs::msg::Point> &slamCones,
										  const bool &borderType,
                                          const double &centerError,
                                          const double &radiusError)
{
	SkidpadCircle meanCircle;
	meanCircle.x = 0.0;
	meanCircle.y = 0.0;
	meanCircle.r = 0.0;

	// given 3 points, generate the circle
	for(int i = 0; i < slamCones.size() - 2; i ++)
	{
		geometry_msgs::msg::Point p1 = slamCones[i];

		for(int j = i + 1; j < slamCones.size() - 1; j ++)
		{
			geometry_msgs::msg::Point p2 = slamCones[j];

			for(int k = j + 1; k < slamCones.size(); k ++)
			{
				geometry_msgs::msg::Point p3 = slamCones[k];

				SkidpadCircle circle;

				double d = 2.0 * (p1.x * (p2.y - p3.y) +
								  p2.x * (p3.y - p1.y) +
								  p3.x * (p1.y - p2.y));

				circle.x = ((p1.x * p1.x + p1.y * p1.y) * (p2.y - p3.y) +
							 (p2.x * p2.x + p2.y * p2.y) * (p3.y - p1.y) +
							 (p3.x * p3.x + p3.y * p3.y) * (p1.y - p2.y)) / d;

				circle.y = ((p1.x * p1.x + p1.y * p1.y) * (p3.x - p2.x) +
							 (p2.x * p2.x + p2.y * p2.y) * (p1.x - p3.x) +
							 (p3.x * p3.x + p3.y * p3.y) * (p2.x - p1.x)) / d;

				circle.r = (p1.x - circle.x) * (p1.x - circle.x) +
						   (p1.y - circle.y) * (p1.y - circle.y);

				// check if the center found is in the error
				if (borderType)
				{
					if (circle.x < this->circleL.x - centerError || circle.x > this->circleL.x + centerError)
						continue;

					if (circle.y < this->circleL.y - centerError || circle.y > this->circleL.y + centerError)
						continue;
				}
				else
				{
					if (circle.x < this->circleR.x - centerError || circle.x > this->circleR.x + centerError)
						continue;

					if (circle.y < this->circleR.y - centerError || circle.y > this->circleR.y + centerError)
						continue;
				}

				if ((circle.r > (this->param_innerRadius - radiusError) * (this->param_innerRadius - radiusError) &&
				     circle.r < (this->param_innerRadius + radiusError) * (this->param_innerRadius + radiusError)) ||
				    (circle.r > (this->param_outerRadius - radiusError) * (this->param_outerRadius - radiusError) &&
					 circle.r < (this->param_outerRadius + radiusError) * (this->param_outerRadius + radiusError)))
				{
					meanCircle.x += circle.x;
					meanCircle.y += circle.y;
					meanCircle.r ++; // NOT used as radius, but as counter
				}
			}
		}
	}

	if (meanCircle.r > 0)
	{
		if (borderType)
		{
			this->circleL.x = meanCircle.x / meanCircle.r;
			this->circleL.y = meanCircle.y / meanCircle.r;
		}
		else
		{
			this->circleR.x = meanCircle.x / meanCircle.r;
			this->circleR.y = meanCircle.y / meanCircle.r;
		}
	}
	else // no center was found, new recursive iteration with bigger error
	{
		this->generateCircleCenter(slamCones,
								   borderType,
								   centerError + this->param_errorIncrement,
								   radiusError + this->param_errorIncrement);
	}

}

/// @brief generates the border based on a vector of points (all left-side | all right-side). 1) finds center -> 2) discretizes the circle -> 3) publishes
/// @param slamCones cones from odometry  (x,y,z,red,green,blue)
/// @param borderType left | right [true | false]
void SkidpadPlanner::generateBorder(const std::vector<geometry_msgs::msg::Point> &slamCones,
								    const bool &borderType)
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

    if (!this->firstAlign)
    {
        this->generateCircleCenter(slamCones,
								   borderType,
                                   this->param_firstAlignCenterError,
                                   this->param_firstAlignRadiusError);
    }
	else
	{
		this->generateCircleCenter(slamCones,
								   borderType,
                                   this->param_centerError,
                                   this->param_radiusError);
	}

	std::vector<geometry_msgs::msg::Point> innerCircle;
	std::vector<geometry_msgs::msg::Point> outerCircle;

	if (borderType)
	{
		this->circleL.r = this->param_innerRadius;
		innerCircle = this->generateDiscretizedCircle(this->circleL,
												      true);

		this->circleL.r = this->param_outerRadius;
		outerCircle = this->generateDiscretizedCircle(this->circleL,
											          true);

		border.color.r = 0.0;
		border.color.g = 0.0;
		border.color.b = 1.0;
	}
	else
	{
		this->circleR.r = this->param_innerRadius;
		innerCircle = this->generateDiscretizedCircle(this->circleR,
												      false);

		this->circleR.r = this->param_outerRadius;
		outerCircle = this->generateDiscretizedCircle(this->circleR,
												      false);

		border.color.r = 1.0;
		border.color.g = 1.0;
		border.color.b = 0.0;
	}

    border.points = innerCircle;
	this->borders.markers.push_back(border);

	border.id += 2;
	border.points = outerCircle;
	this->borders.markers.push_back(border);
}

/// @brief generates the centerline calling the other methods and publishes
void SkidpadPlanner::generateCenterLine()
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

	this->circleL.r = this->param_centerRadius + this->param_radiusOffsetL;
	std::vector<geometry_msgs::msg::Point> circlePointsL = generateDiscretizedCircle(this->circleL,
																 true);

	this->circleR.r = this->param_centerRadius + this->param_radiusOffsetR;
	std::vector<geometry_msgs::msg::Point> circlePointsR = generateDiscretizedCircle(this->circleR,
																 false);

	geometry_msgs::msg::Point start;
	start.x = 0.0;
	start.y = 0.0;
	start.z = 0.0;

	// find intersection between two circumferences
	double d = std::sqrt((this->circleL.x - this->circleR.x) * (this->circleL.x - this->circleR.x) +
						 (this->circleL.y - this->circleR.y) * (this->circleL.y - this->circleR.y));

	double a = (d * d) / (d * 2.0);
	double h = std::sqrt((this->param_outerRadius * this->param_outerRadius) - (a * a));

	geometry_msgs::msg::Point base;
	base.x = this->circleR.x + (a * (this->circleL.x - this->circleR.x)) / d;
	base.y = this->circleR.y + (a * (this->circleL.y - this->circleR.y)) / d;

	geometry_msgs::msg::Point middleStart;
	middleStart.x = base.x + (h * (this->circleR.y - this->circleL.y)) / d;
	middleStart.y = base.y - (h * (this->circleR.x - this->circleL.x)) / d;
	middleStart.z = 0.0;

	geometry_msgs::msg::Point middleEnd;
	middleEnd.x = base.x - (h * (this->circleR.y - this->circleL.y)) / d;
	middleEnd.y = base.y + (h * (this->circleR.x - this->circleL.x)) / d;
	middleEnd.z = 0.0;

	double slope = (middleStart.y - middleEnd.y) / (middleStart.x - middleEnd.x);
	double intercept = (middleEnd.x * middleStart.y - middleStart.x * middleEnd.y) / (middleEnd.x - middleStart.x);

	geometry_msgs::msg::Point end;
	end.x = 30.0;
	end.y = slope * end.x + intercept;
	end.z = 0.0;

	std::vector<geometry_msgs::msg::Point> startLine = this->generateDiscretizedLine(start,
																 middleStart);

	std::vector<geometry_msgs::msg::Point> middleStartLine = this->generateDiscretizedLine(middleStart,
																       circlePointsR.front());

	std::vector<geometry_msgs::msg::Point> middleEndLine = this->generateDiscretizedLine(circlePointsL.back(),
																     middleEnd);

	std::vector<geometry_msgs::msg::Point> endLine = this->generateDiscretizedLine(middleEnd,
															   end);

	centerLine.points.insert(centerLine.points.end(),
							 startLine.begin(),
							 startLine.end());

	centerLine.points.insert(centerLine.points.end(),
							 middleStartLine.begin(),
							 middleStartLine.end());

	centerLine.points.insert(centerLine.points.end(),
							 circlePointsR.begin(),
							 circlePointsR.end());

	centerLine.points.insert(centerLine.points.end(),
							 circlePointsR.begin(),
							 circlePointsR.end());

	centerLine.points.insert(centerLine.points.end(),
							 circlePointsL.begin() + std::lround(this->param_circlePoints / 40),
							 circlePointsL.end());

	centerLine.points.insert(centerLine.points.end(),
							 circlePointsL.begin(),
							 circlePointsL.end());

	centerLine.points.insert(centerLine.points.end(),
							 middleEndLine.begin(),
							 middleEndLine.end());

	centerLine.points.insert(centerLine.points.end(),
							 endLine.begin(),
							 endLine.end());

	double minDistance;
	int minIndex;
	int futureIndex = std::min(this->param_circlePoints / 2,
							   this->param_linePoints / 2);
	int endingIndex = std::min(this->lastOdometry + futureIndex,
							   (int) centerLine.points.size() - 1);

	for (int i = this->lastOdometry; i < endingIndex; i ++)
	{
		double distance = (this->odometry.x - centerLine.points[i].x) * (this->odometry.x - centerLine.points[i].x) +
						  (this->odometry.y - centerLine.points[i].y) * (this->odometry.y - centerLine.points[i].y);

		if (i == this->lastOdometry || distance < minDistance)
		{
			minDistance = distance;
			minIndex = i;
		}
	}

	this->lastOdometry = minIndex;

	centerLine.points.erase(centerLine.points.begin(), centerLine.points.begin() + minIndex);

	this->centerLinePub->publish(centerLine);
}