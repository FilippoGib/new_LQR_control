#ifndef SKIDPAD_H
#define SKIDPAD_H

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "mmr_base/msg/race_status.hpp"
#include "mmr_base/msg/marker.hpp"
#include <unistd.h>

using Point = geometry_msgs::msg::Point;

struct Circle
{
	double x;
	double y;
    double r;
};

class SkidpadPlanner
{
    public:
        SkidpadPlanner(const rclcpp::Node::SharedPtr &nh,
					   const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr &bordersPub,
					   const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr &centerLinePub);

        void loadParameters();

		void raceStatusCallBack(mmr_base::msg::RaceStatus::SharedPtr raceStatus);

		void odometryCallback(nav_msgs::msg::Odometry::SharedPtr odometry);

		void slamConesCallback(mmr_base::msg::Marker::SharedPtr slamCones);

        std::vector<Point> generateDiscretizedLine(const Point &pointStart,
										           const Point &pointEnd);

        std::vector<Point> generateDiscretizedCircle(const Circle &circle,
                                                     const bool &borderType);

        void generateCircleCenter(const std::vector<Point> &slamCones,
                                  const bool &borderType,
                                  const double &centerError,
                                  const double &radiusError);

		void generateBorder(const std::vector<Point> &slamCones,
                            const bool &borderType);

		void generateCenterLine();

    private:
        rclcpp::Node::SharedPtr nh;
		rclcpp::Publisher<mmr_base::msg::MarkerArray>::SharedPtr bordersPub;
		rclcpp::Publisher<mmr_base::msg::Marker>::SharedPtr centerLinePub;

        mmr_base::msg::MarkerArray borders;
        mmr_base::msg::RaceStatus raceStatus;
		Point odometry;

        Circle circleL;
        Circle circleR;

        bool firstAlign;
        int lastOdometry;

        double param_centerX;
        double param_centerY;
        double param_innerRadius;
        double param_centerRadius;
        double param_outerRadius;
        double param_firstAlignCenterError;
        double param_firstAlignRadiusError;
        double param_errorIncrement;
        double param_centerError;
        double param_radiusError;
        double param_radiusOffsetL;
        double param_radiusOffsetR;
        int param_circlePoints;
        int param_linePoints;

};

#endif // SKIDPAD_H