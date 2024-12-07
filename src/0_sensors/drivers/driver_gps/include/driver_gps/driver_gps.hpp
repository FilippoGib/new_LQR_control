#ifndef DRIVER_GPS
#define DRIVER_GPS

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <serial_gps_interface/serial_gps_interface.hpp>
#include <common_msgs/msg/nav_sat_heading.hpp>
#include <GeographicLib/Geodesic.hpp>
#include <std_msgs/msg/float64.hpp>

#define EARTH_RADIUS 6378.137

inline double measure_distance(double lat1, double lon1, double lat2, double lon2)
{
    double dLat = lat2 * M_PI / 180 - lat1 * M_PI / 180;
    double dLon = lon2 * M_PI / 180 - lon1 * M_PI / 180;
    double a = sin(dLat/2) * sin(dLat/2) +
        cos(lat1 * M_PI / 180) * cos(lat2 * M_PI / 180) *
        sin(dLon/2) * sin(dLon/2);
    double c = 2 * atan2(sqrt(a), sqrt(1-a));
    double d = EARTH_RADIUS * c;
    return d * 1000; // meters
}

inline double from_kph_to_mph(double speed_kph)
{
    return speed_kph / 3.6;
}

inline double from_mps_to_kph(double speed_mps)
{
    return speed_mps * 3.6;
}

inline double deg2rad(double deg)
{
    return deg * M_PI / 180;
}

class DriverGPS : public rclcpp::Node
{
    private:
    
    /* Define basic GPS data */
    gps_data_t gps_data;
    string gps_server_addr;
    string gps_server_port;
    double time_diff;
    bool enable_logging = false;

    /* Simple thread to monitor GPS Data */
	std::thread *gpsDataMonitorThread;

    /* Output message */
    sensor_msgs::msg::NavSatFix navsat_data;
    sensor_msgs::msg::NavSatStatus navsat_status;

    /* Actual and prev location. This is used for computing Azimuth */
    double prev_lat = -1.0, act_lat = -1.0, prev_long = -1.0, act_long = -1.0, act_az = 0.0;

    /* GPS location publisher */
    rclcpp::Publisher<common_msgs::msg::NavSatHeading>::SharedPtr gps_position_pub;

    /* GPS speed publisher, default double 64bit (mps) */
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr gps_speed_pub;

    /**
     * Compute Azimuth between two points
    */
    void computeAzimuth()
    {
        // Create a Geodesic object
        GeographicLib::Geodesic geod(GeographicLib::Constants::WGS84_a(), GeographicLib::Constants::WGS84_f());

        // Calculate the azimuth (YAW angle) between the two points
        double s12, azi2;
        geod.Inverse(prev_lat, prev_long, act_lat, act_long, s12, this->act_az, azi2);

        // Print the azimuth
        std::cout << "Azimuth: " << this->act_az << " degrees" << std::endl;
    }

    public:

    /* Default Constructor */
    DriverGPS();

    /* Method to load parameters */
    void loadParameters();

    /* Method to publish GPS data */
    void publishData();

    /* Infinite loop for reading GPS data */
    void monitorGPSData();

    /* Destructor */
    ~DriverGPS();
};

#endif /* DRIVER_GPS */