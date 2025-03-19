#ifndef AUTOCROSS_H
#define AUTOCROSS_H

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "mmr_base/msg/race_status.hpp"
#include "mmr_common_functions/mmr_common_functions.h"
#include <unistd.h>

// #define DEBUG 1

enum BorderType
{
	BLUE,
	YELLOW,
	CENTER
};

class AutocrossPlanner
{
	public:
		AutocrossPlanner(const rclcpp::Node::SharedPtr &nh,
						 const rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr &bordersPub,
						 const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr &centerLinePub,
						 const rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr &bordersCompletedPub,
						 const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr &centerLineCompletedPub);

		void loadParameters();

		void raceStatusCallBack(mmr_base::msg::RaceStatus::SharedPtr raceStatus);

		void odometryCallback(nav_msgs::msg::Odometry::SharedPtr odometry);

		void slamConesCallback(visualization_msgs::msg::Marker::SharedPtr slamCones);

		double calculateAngle(const Point& a, const Point& b, const Point& c);

		double calculateSharpness(const std::vector<Point>& points, const std::vector<int>& visited);

		void stp(const std::vector<Point> &points,
         const std::vector<std::vector<double>> &distances,
         std::vector<int> &bestPath, std::vector<int> &visited,
         double currentDistance, double &bestDistance, double &bestSolution, int index, int count, BorderType borderType);

		std::vector<Point> filterPoints(const std::vector<Point> &points,
										const BorderType borderType);

		std::vector<Point> generatePointsOrdered(std::vector<Point> &pointsUnordered,
												 const BorderType borderType);

		std::vector<Point> generateBorder(const std::vector<Point> &slamCones,
										  const BorderType borderType);

		void generateCenterLine(const std::vector<Point> &borderB,
								const std::vector<Point> &borderY);

	private:
		rclcpp::Node::SharedPtr nh;
		rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr bordersPub;
		rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr centerLinePub;
		rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr bordersCompletedPub;
		rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr centerLineCompletedPub;

		visualization_msgs::msg::MarkerArray borders;
		visualization_msgs::msg::MarkerArray bordersCompleted;
		int currentLap;
		Point odometry;
		bool idle = false; //when true the node is idle and only publishes centerline_completed in a transient local fashion 

		std::vector<std::vector<Point>> firstCones;
		std::vector<std::vector<Point>> lastSafeBorder;

		double param_clusterDistance;
		double param_conesDistance;
		int param_firstConesSize;
		int param_firstConesStart;
		double param_diffColorConesDistance;
		int param_diffColorConesSize;
		double param_trackWidth;
		double param_sharpnessWeight;
};

#endif //AUTOCROSS_H