#include <driver_gps/driver_gps.hpp>

DriverGPS::DriverGPS() : Node("driver_gps_node")
{
    error_t err;

    /* Load Parameters */
    this->loadParameters();

    /* Open GPS Connection */
    err = SerialGpsInterface::open_connection(this->gps_server_addr, this->gps_server_port, this->gps_data);
    if (err)
    {
        this->~DriverGPS();
    }

    /* Create GPS message to publish */
    // this->navsat_status.status = this->gps_data.status;
    this->navsat_status.service = 0xF;
    navsat_data.header.frame_id = "imu_link";
    navsat_data.status = navsat_status;
    navsat_data.latitude = 0;
    navsat_data.longitude = 0;
    navsat_data.altitude = 0;

	/* Define QoS for Best Effort messages transport */
	auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);

    /* Create GPS position publisher */
    this->gps_position_pub = this->create_publisher<common_msgs::msg::NavSatHeading>("/emlid/gps/data", qos);

    /* Create GPS speed publisher */
    this->gps_speed_pub = this->create_publisher<std_msgs::msg::Float64>("/emlid/gps/speed", 10);



    /* Start to monitor GPS data. We're using a thread in order to NOT block the node. */
    this-> gpsDataMonitorThread = new std::thread([this]()
        {
            monitorGPSData();
        } 
    );
}


void DriverGPS::loadParameters()
{
	/* ***** DECLARING PARAMETERS ***** */

	declare_parameter("generic.gps_server_addr", "localhost");
	declare_parameter("generic.gps_server_port", "2947");
	declare_parameter("generic.enable_logging", false);

	/* ******************************** */

	/* ***** READING PARAMETERS ***** */

	get_parameter("generic.gps_server_addr", this->gps_server_addr);
	get_parameter("generic.gps_server_port", this->gps_server_port);
    	get_parameter("generic.enable_logging", this->enable_logging);

	/* ****************************** */
}

void DriverGPS::publishData()
{
    /* check if new data has been received */
    if ((gps_data.fix.latitude != navsat_data.latitude) || 
        (gps_data.fix.longitude != navsat_data.longitude) || 
        (gps_data.fix.altitude != navsat_data.altitude))
    {
        double distance = measure_distance(gps_data.fix.latitude, navsat_data.longitude,
            navsat_data.latitude, gps_data.fix.longitude
        );

        std_msgs::msg::Float64 gps_speed;
        if(this->time_diff == 0)
        {
            /* First data. Set first time recorded */
            this->time_diff = gps_data.fix.time.tv_sec + (gps_data.fix.time.tv_nsec*1e-9);
        } else {
            double delta_time = (gps_data.fix.time.tv_sec + (gps_data.fix.time.tv_nsec*1e-9)) - this->time_diff;
            double speed_mps = (distance*(1/delta_time));
            double speed_kph = speed_mps * 3.6;
            gps_speed.data = speed_mps;
            /* Computer azimuth (Course to North)*/
            this->act_lat = gps_data.fix.latitude;
            this->act_long = gps_data.fix.longitude;
            if (this->prev_lat == -1.0)
            {
                this->prev_lat = gps_data.fix.latitude;
                this->prev_long = gps_data.fix.longitude;
            } else
            {
                this->computeAzimuth();
            }
            this->prev_lat = gps_data.fix.latitude;
            this->prev_long = gps_data.fix.longitude;

            /* Normalize angle */
            if (this->gps_data.fix.track > 180.0)
            {
                this->gps_data.fix.track -= 360;
            } else if(this->gps_data.fix.track < -180.0)
            {
                this->gps_data.fix.track += 360;
            }

            if (this->enable_logging)
            {
                RCLCPP_INFO(this->get_logger(), "You traveled %lf meters in %lf second(s)", distance, delta_time);
                RCLCPP_INFO(this->get_logger(), "Velocity = %lf (m/s) == %lf (km/h)", speed_mps, speed_kph);
                RCLCPP_INFO(this->get_logger(), "Computed Azimuth: %lf degrees.", this->act_az);
                RCLCPP_INFO(this->get_logger(), "GPS LIB AZIMUTH: %lf degrees. Uncertainty: %lf", gps_data.fix.track, gps_data.fix.epd);
                RCLCPP_INFO(this->get_logger(), "DIFF: %lf degrees.", round(gps_data.fix.track-this->act_az));
            }
            this->time_diff = gps_data.fix.time.tv_sec + (gps_data.fix.time.tv_nsec*1e-9);
        }
        if (std::isnan(gps_data.fix.latitude))
        {
            RCLCPP_INFO(this->get_logger(), "GPS DATA NOT AVAILABLE");
            return;
        }
        /* Update new position */
        navsat_data.header.stamp = this->now();
        navsat_data.latitude = gps_data.fix.latitude;
        navsat_data.longitude = gps_data.fix.longitude;
        navsat_data.altitude = gps_data.fix.altitude;
        common_msgs::msg::NavSatHeading navsat_heading_msg;
        navsat_heading_msg.gps_data = navsat_data;
        navsat_heading_msg.heading_rad = deg2rad((gps_data.fix.track + this->act_az) / 2);
        navsat_heading_msg.is_heading_reliable = ((gps_speed.data < 3.0) || (abs(round(gps_data.fix.track-this->act_az)) > 20) || (std::isnan(gps_data.fix.track))) ? false : true;
    
        this->gps_speed_pub->publish(gps_speed);
        this->gps_position_pub->publish(navsat_heading_msg);

    }
}

void DriverGPS::monitorGPSData()
{
    error_t err;

    while(true)
    {
        err = SerialGpsInterface::read_data(this->gps_data);
        if (!err)
        {
            this->publishData();
        }
    }
}

DriverGPS::~DriverGPS()
{
    ;
}
