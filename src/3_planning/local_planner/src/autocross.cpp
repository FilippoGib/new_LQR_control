#include "local_planner/autocross.h"
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

	std::vector<Point> points;

	for (int i = 0; i < 3; i ++) // BLUE, YELLOW and CENTER
	{
		this->firstCones.push_back(points);
	}

	for (int i = 0; i < 2; i ++) // BLUE and YELLOW
	{
		this->lastSafeBorder.push_back(points);
	}

	this->loadParameters();
}

/// @brief loads the class parameters from the .yaml file
void AutocrossPlanner::loadParameters()
{
  	nh->declare_parameter<double>("autocross/clusterDistance", 0.0);
  	this->param_clusterDistance = nh->get_parameter("autocross/clusterDistance").get_value<double>();

	nh->declare_parameter<double>("autocross/conesDistance", 0.0);
  	this->param_conesDistance = nh->get_parameter("autocross/conesDistance").get_value<double>();
	RCLCPP_INFO(this->nh->get_logger(), "max CONES DISTANCE: %f meters", std::sqrt(this->param_conesDistance));

	nh->declare_parameter<int>("autocross/firstConesSize", 0);
  	this->param_firstConesSize = nh->get_parameter("autocross/firstConesSize").get_value<int>();

  	nh->declare_parameter<int>("autocross/firstConesStart", 0);
  	this->param_firstConesStart = nh->get_parameter("autocross/firstConesStart").get_value<int>();

	nh->declare_parameter<double>("autocross/diffColorConesDistance", 0);
  	this->param_diffColorConesDistance = nh->get_parameter("autocross/diffColorConesDistance").get_value<double>();

	nh->declare_parameter<int>("autocross/diffColorConesSize", 0);
  	this->param_diffColorConesSize = nh->get_parameter("autocross/diffColorConesSize").get_value<int>();

	nh->declare_parameter<double>("autocross/trackWidth", 0.0);
  	this->param_trackWidth = nh->get_parameter("autocross/trackWidth").get_value<double>();
	RCLCPP_INFO(this->nh->get_logger(), "TRACK WIDTH: %f meters", std::sqrt(this->param_trackWidth));

	nh->declare_parameter<double>("autocross/sharpnessWeight", 0.0);
  	this->param_sharpnessWeight = nh->get_parameter("autocross/sharpnessWeight").get_value<double>();
	RCLCPP_INFO(this->nh->get_logger(), "sharpness weight: %f ", this->param_sharpnessWeight);
}

/// @brief callback to get the race status from the subscription
/// @param raceStatus custom message
void AutocrossPlanner::raceStatusCallBack(mmr_base::msg::RaceStatus::SharedPtr raceStatus)
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

/// @brief callback to get cones from subscription
/// @param slamCones cones from odometry  (x,y,z,red,green,blue)
void AutocrossPlanner::slamConesCallback(visualization_msgs::msg::Marker::SharedPtr slamCones)
{
	if(this->idle)
	{
		return;
	}
	
	std::vector<Point> slamConesB;
	std::vector<Point> slamConesY;

	for (int i = 0; i < slamCones->points.size(); i ++)
	{
		Point point;
		point.x = slamCones->points[i].x;
		point.y = slamCones->points[i].y;

		if (slamCones->colors[i].r < 0.1f && slamCones->colors[i].g < 0.1f && slamCones->colors[i].b > 0.9f)
			slamConesB.push_back(point);

		if(slamCones->colors[i].r > 0.9f && slamCones->colors[i].g > 0.9f && slamCones->colors[i].b < 0.1f)
			slamConesY.push_back(point);
  	}

	if (slamConesB.size() < 3)
	{
		std::cout<<"Not enough blue slamCones"<<std::endl;
		return;
	}
	if (slamConesY.size() < 3)
	{
		std::cout<<"Not enough yellow slamCones"<<std::endl;
		return;
	}

	this->borders.markers.clear();
	this->bordersCompleted.markers.clear();
	std::vector<Point> borderB = this->generateBorder(slamConesB,
													  BLUE);
	#ifdef DEBUG
		RCLCPP_INFO(this->nh->get_logger(), "Blue border size: %d", borderB.size());
	#endif
	std::vector<Point> borderY = this->generateBorder(slamConesY,
													  YELLOW);
	#ifdef DEBUG
		RCLCPP_INFO(this->nh->get_logger(), "Yellow border size: %d", borderY.size());
	#endif

	if(this->currentLap > 1)
	{
		for (int i = 0; i < 5; i ++)
		{
			this->bordersCompletedPub->publish(this->bordersCompleted);
			usleep(100000);
		}
	}

	this->bordersPub->publish(this->borders);

	if(borderB.size() && borderY.size())
	{

		this->generateCenterLine(borderB,
							 borderY);
		#ifdef DEBUG
			RCLCPP_INFO(this->nh->get_logger(), "Centerline generated");
		#endif
	}
}

// Function to calculate the angle between three points
double AutocrossPlanner::calculateAngle(const Point& a, const Point& b, const Point& c) {
    double abx = b.x - a.x;
    double aby = b.y - a.y;
    double cbx = b.x - c.x;
    double cby = b.y - c.y;

    double dotProduct = abx * cbx + aby * cby;
    double magnitudeAB = std::sqrt(abx * abx + aby * aby);
    double magnitudeCB = std::sqrt(cbx * cbx + cby * cby);

    return std::acos(dotProduct / (magnitudeAB * magnitudeCB));
}

double AutocrossPlanner::calculateSharpness(const std::vector<Point>& points, const std::vector<int>& visited) {
    double sharpness = 0.0;
    for (int i = 2; i < visited.size(); ++i) {
        int prevPrevIndex = visited[i - 2];
        int prevIndex = visited[i - 1];
        int currentIndex = visited[i];
        sharpness += calculateAngle(points[prevPrevIndex], points[prevIndex], points[currentIndex]);
    }
    return sharpness;
}

/// @brief recursive function to find the best path
/// @param points unordered points
/// @param path ordered points
/// @param minLength minimum length of the path
void AutocrossPlanner::stp(const std::vector<Point> &points,
						   const std::vector<std::vector<double>> &distances,
						   std::vector<int> &bestPath,
						   std::vector<int> &visited,
						   double currentDistance,
						   double &bestDistance,
						   double &bestSolution,
						   int index,
						   int count,
						   BorderType borderType)
{
	// traveling salesman, saving also the best path
	if (count == points.size())
	{
		double sharpness = 0;

		sharpness = calculateSharpness(points, visited);

		if (currentDistance + sharpness * this->param_sharpnessWeight < bestSolution) //facciamo la valutazione non solo sulla distanza ma anche sugli angoli
		{
			bestSolution = currentDistance + sharpness * this->param_sharpnessWeight;

			for (int i = 0; i < points.size(); i ++)
			{
				bestPath[i] = visited[i];
			}
		}
		return;
	}

	for (int i = 0; i < points.size(); i ++)
	{
		if (isIn(visited, i) == false)
		{
			if (distances[index][i] > this->param_conesDistance)
				continue;

			if (currentDistance + distances[index][i] > bestDistance)
				continue;

			visited[count] = i;
			stp(points, distances, bestPath, visited,
				currentDistance + distances[index][i], bestDistance, bestSolution, i, count + 1, borderType);
			visited[count] = -1;
		}
	}
}

std::vector<Point> AutocrossPlanner::filterPoints(const std::vector<Point> &pointsUnordered,
												  const BorderType borderType)
{
	int minIndex = -1;

	if (this->firstCones[borderType].size() == 0)
	{
		Point start;
		start.x = 0.0;
		start.y = 0.0;

		double minDistance;

		for (int i = 0; i < pointsUnordered.size(); i ++)
		{
			double distance = getQuadraticDistance(pointsUnordered[i], start);
			if (i == 0 || distance < minDistance)
			{
				minIndex = i;
				minDistance = distance;
			}
		}
	}
	else
	{
		double minDistance;

		for (int i = 0; i < pointsUnordered.size(); i ++)
		{
			bool inFirstCones = false;
			for (int j = 0; j < this->firstCones[borderType].size(); j ++)
			{
				if (getQuadraticDistance(pointsUnordered[i], this->firstCones[borderType][j]) < 1.0)
				{
					inFirstCones = true;
					break;
				}
			}

			double distance = getQuadraticDistance(pointsUnordered[i],
									 this->firstCones[borderType].back());
			if ((minIndex == -1 || distance < minDistance) && !inFirstCones)
			{
				minIndex = i;
				minDistance = distance;
			}
		}
	}

	std::vector<Point> safePoints;
	safePoints.push_back(pointsUnordered[minIndex]);

	std::vector<bool> linked(pointsUnordered.size(), false);
	linked[minIndex] = true;

	// every cone in a radius of N meters from the previous is safe (CLUSTERING and FILTERING)
	while (true)
	{
		bool found = false;
		for (int i = 0; i < pointsUnordered.size(); i ++)
		{
			// check if current cone is already in firstCones
			bool inFirstCones = false;
			for (int j = 0; j < this->firstCones[borderType].size(); j ++)
			{
				if (getQuadraticDistance(pointsUnordered[i], this->firstCones[borderType][j]) < 1.0)
				{
					inFirstCones = true;
					break;
				}
			}

			if (inFirstCones)
				continue;
			
			BorderType otherBorderType = (borderType == BLUE) ? YELLOW : BLUE;
			bool isWrongColor = false;
			int lastOtherBorderSize = this->lastSafeBorder[otherBorderType].size();
			if (lastOtherBorderSize >= this->param_diffColorConesSize) {
				for (int k = lastOtherBorderSize-this->param_diffColorConesSize; k < lastOtherBorderSize - 1; k++) {
					if (getPointSegmentDistance(pointsUnordered[i], 
												this->lastSafeBorder[otherBorderType][k], 
												this->lastSafeBorder[otherBorderType][k+1]) 
						< this->param_diffColorConesDistance)
					{
						isWrongColor = true;
						break;
					}
				}
			}

			if (isWrongColor)
				continue;

			// // find the nearest cone on the other last safe border
			// double minDistance;
			// for (int j = 0; j < lastOtherBorderSize; j ++)
			// {
			// 	double distance = getQuadraticDistance(pointsUnordered[i], this->lastSafeBorder[otherBorderType][j]);
			// 	if (j==0 || distance < minDistance)
			// 	{
			// 		minDistance = distance;
			// 	}
			// }

			// if (minDistance > this->param_trackWidth)
			// 	continue;

			if (linked[i] == false)
			{
				// check if the distance from one of the safe points is less than "clusterDistance" meters
				for (int j = 0; j < safePoints.size(); j ++)
				{
					if (getQuadraticDistance(pointsUnordered[i], safePoints[j]) < this->param_clusterDistance)
					{
						safePoints.push_back(pointsUnordered[i]);
						linked[i] = true;
						found = true;
						break;
					}
				}
			}
		}

		if (!found)
			break;
	}

	for (int i = 0; i < this->firstCones[borderType].size(); i ++)
	{
		safePoints.push_back(this->firstCones[borderType][i]);
	}

	return safePoints;
}

/// @brief utility function to order a vector of unordered points based on the curve weight
/// @param pointsUnordered unordered vector of points
/// @return order vector of points
std::vector<Point> AutocrossPlanner::generatePointsOrdered(std::vector<Point> &pointsUnordered,
														   const BorderType borderType)
{
	// update firstCones with new slightly different cones
	for (int i = 0; i < this->firstCones[borderType].size(); i ++)
	{
		for (int j = 0; j < pointsUnordered.size(); j ++)
		{
			if (getQuadraticDistance(this->firstCones[borderType][i], pointsUnordered[j]) < 1.0)
			{
				this->firstCones[borderType][i] = pointsUnordered[j];
				break;
			}
		}
	}

	std::vector<Point> safePoints = this->filterPoints(pointsUnordered, borderType);

	std::vector<std::vector<double>> pointsDistances; // distances matrix

	for (int i = 0; i < safePoints.size(); i ++)
	{
		std::vector<double> row;

		for (int j = 0; j < safePoints.size(); j ++)
		{
			if (i == j)
				row.push_back(0.0);
			else
				row.push_back(getQuadraticDistance(safePoints[i], safePoints[j]));
		}

		pointsDistances.push_back(row);
	}

	std::vector<int> bestPath(safePoints.size(), -1);
	std::vector<int> visited(safePoints.size(), -1);
	int count;
	int index;

	if (this->firstCones[borderType].size() > 0)
	{
		for (int i = 0; i < this->firstCones[borderType].size(); i ++)
		{
			for (int j = 0; j < safePoints.size(); j ++)
			{
				if (getQuadraticDistance(this->firstCones[borderType][i], safePoints[j]) < 1.0)
				{
					bestPath[i] = j;
					visited[i] = j;
					index = j;

					break;
				}
			}
		}
		count = this->firstCones[borderType].size();
	}
	else
	{
		bestPath[0] = 0;
		visited[0] = 0;
		index = 0;
		count = 1;
	}

	double bestDistance = 10e7;
	double bestSolution = 10e7;

	this->stp(safePoints, pointsDistances, bestPath, visited, 0.0, bestDistance, bestSolution, index, count, borderType);

	std::vector<Point> pointsOrdered;

	if (isIn(bestPath, -1))
	{
		return pointsOrdered; // empty vector, stp failed
	}

	for (int i = 0; i < bestPath.size(); i ++)
	{
		pointsOrdered.push_back(safePoints[bestPath[i]]);
	}

	if (pointsOrdered.size() > this->param_firstConesStart)
	{
		this->firstCones[borderType].clear();
		for (int i = 0; i < pointsOrdered.size() - this->param_firstConesSize; i ++)
		{
			this->firstCones[borderType].push_back(pointsOrdered[i]);
		}
	}

	return pointsOrdered;
}

/// @brief generates the borders (one at a time)
/// @param slamCones only blue cones | only yellow cones from slamCones
/// @param borderType blue | yellow [true | false]
/// @return vector of ordered points, the border
std::vector<Point> AutocrossPlanner::generateBorder(const std::vector<Point> &slamCones,
													const BorderType borderType)
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

	if (borderType == 0)
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

	std::vector<Point> borderPoints = slamCones;

	std::vector<Point> borderPointsOrdered = this->generatePointsOrdered(borderPoints, borderType);

	if (borderPointsOrdered.size() < 2)
	{
		borderPointsOrdered = this->lastSafeBorder[borderType];
	} else 
	{
		double totalLength = 0;
		for(int j = 0; j < borderPointsOrdered.size() - 1; j++){
			totalLength += std::sqrt(getQuadraticDistance(borderPointsOrdered[j], borderPointsOrdered[j+1]));
		}
		this->param_conesDistance = std::max(this->param_conesDistance, std::pow((totalLength / borderPointsOrdered.size()) + 2, 2));
		this->param_clusterDistance = this->param_conesDistance;

		this->lastSafeBorder[borderType] = borderPointsOrdered;
	}

	border.points = borderPointsOrdered;
	this->borders.markers.push_back(border);
	
	if(this->currentLap > 1)
	{
		this->bordersCompleted.markers.push_back(border);
	}

	return borderPointsOrdered;
}

/// @brief generate the centerline based only on the borders
/// @param borderB blue border, on the left side of the car, true in boolean
/// @param borderY yellow border, on the right side of the car, false in boolean
void AutocrossPlanner::generateCenterLine(const std::vector<Point> &borderB,
										  const std::vector<Point> &borderY)
{
	visualization_msgs::msg::Marker centerLine;
	centerLine.id = CENTER;
	centerLine.ns = "centerLine";
	centerLine.header.frame_id = "track";
	centerLine.header.stamp = this->nh->get_clock()->now();
	centerLine.type = visualization_msgs::msg::Marker::LINE_STRIP;
	centerLine.action = visualization_msgs::msg::Marker::ADD;
	centerLine.scale.x = 0.1;
	centerLine.scale.y = 0.1;
	centerLine.scale.z = 0.1;
	centerLine.color.a = 1.0;
	centerLine.color.r = 1.0;
	centerLine.color.g = 1.0;
	centerLine.color.b = 1.0;
	centerLine.pose.orientation.w = 1.0;
	centerLine.pose.orientation.x = 0.0;
	centerLine.pose.orientation.y = 0.0;
	centerLine.pose.orientation.z = 0.0;

	std::vector<Point> longerBorder;
	std::vector<Point> shorterBorder;

	if (borderB.size() > borderY.size())
	{
		longerBorder = borderB;
		shorterBorder = borderY;
	}
	else
	{
		longerBorder = borderY;
		shorterBorder = borderB;
	}

	longerBorder = samplePoints(longerBorder, 5);
	shorterBorder = samplePoints(shorterBorder, 5);

	//wrapping to prevent ending the points before the car reach the finish line
	bool endToStart = false;
	double endToStartDistance = getQuadraticDistance(longerBorder.front(), longerBorder.back());

	if (longerBorder.size() > this->param_firstConesStart && endToStartDistance < this->param_clusterDistance)
	{
		endToStart = true;

		longerBorder.insert(longerBorder.end(),
							longerBorder.begin(),
							longerBorder.begin() + this->param_firstConesStart);

		shorterBorder.insert(shorterBorder.end(),
							 shorterBorder.begin(),
							 shorterBorder.begin() + this->param_firstConesStart);
	}

	std::vector<Point> centerPoints;

	// int endingIndex;
	// if (endToStart)
	// 	endingIndex = longerBorder.size() + this->param_firstConesStart;
	// else
	// 	endingIndex = longerBorder.size();

	for (int i = 0; i < longerBorder.size(); i ++)
	{
		double minDistance;
		double minIndex;

		for (int j = 0; j < shorterBorder.size(); j ++)
		{
			double distance = getQuadraticDistance(longerBorder[i], shorterBorder[j]);

			if (j == 0 || distance < minDistance)
			{
				minDistance = distance;
				minIndex = j;
			}
		}

		Point point;
		point.x = (longerBorder[i].x + shorterBorder[minIndex].x) / 2.0;
		point.y = (longerBorder[i].y + shorterBorder[minIndex].y) / 2.0;

		centerPoints.push_back(point);
	}

	double minDistance;
	int minIndex;
	for (int i = 0; i < centerPoints.size(); i ++)
	{
		double distance = getQuadraticDistance(centerPoints[i], this->odometry);

		if (i == 0 || distance < minDistance)
		{
			minDistance = distance;
			minIndex = i;
		}
	}

	// delete every cone before odometry (nearest included)
	if (centerPoints.size() > minIndex + 2)
	{
		centerPoints.erase(centerPoints.begin(), centerPoints.begin() + minIndex + 1);
	}
	else
	{
		centerPoints.erase(centerPoints.begin(), centerPoints.begin() + minIndex);
	}
	centerPoints.insert(centerPoints.begin(), this->odometry);

	// std::vector<Point> centerPointsOrdered = centerPoints;

	centerLine.points = centerPoints;

	this->centerLinePub->publish(centerLine);

	if(this->currentLap > 1)
	{
		this->idle = true;
		// after first lap, the best path is the center line
		// we clear every previous trajectory and we create + publish the completed one (we have all cones)
		centerLine.points.clear();

		if (borderB.size() > borderY.size())
		{
			longerBorder = borderB;
			shorterBorder = borderY;
		}
		else
		{
			longerBorder = borderY;
			shorterBorder = borderB;
		}

		longerBorder = samplePoints(longerBorder, 5);
		shorterBorder = samplePoints(shorterBorder, 5);

		for (int i = 0; i < longerBorder.size(); i ++)
		{
			double minDistance;
			double minIndex;

			for (int j = 0; j < shorterBorder.size(); j ++)
			{
				double distance = getQuadraticDistance(longerBorder[i], shorterBorder[j]);

				if (j == 0 || distance < minDistance)
				{
					minDistance = distance;
					minIndex = j;
				}
			}

			geometry_msgs::msg::Point point;
			point.x = (longerBorder[i].x + shorterBorder[minIndex].x) / 2.0;
			point.y = (longerBorder[i].y + shorterBorder[minIndex].y) / 2.0;
			point.z = euclideanDistance(point.x, longerBorder[i].x, point.y, longerBorder[i].y);

			centerLine.points.push_back(point);
		}

		this->centerLineCompletedPub->publish(centerLine);

		RCLCPP_INFO(this->nh->get_logger(), "AUTOCROSS PLANNER EXECUTION TERMINATED");

		//rclcpp::shutdown(); the node remains "transient local"
	}
}