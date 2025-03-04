#include <gps_imu_fusion/gps_imu_fusion.hpp>

GpsImuFusion::GpsImuFusion() : rclcpp::Node("gps_imu_fusion_node")
{
    /* Load ROS 2 parameters */
    this->loadParameters();

    /* Define QoS for Best Effort messages transport */
	auto qos = rclcpp::QoS(rclcpp::KeepLast(10), rmw_qos_profile_sensor_data);

    /* Create subscribers */
    this->imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
        this->imu_topic, 1, std::bind(&GpsImuFusion::imuDataCallback, this, std::placeholders::_1));

    this->cones_sub = this->create_subscription<visualization_msgs::msg::Marker>(
        this->cones_topic, 1, std::bind(&GpsImuFusion::conesCallback, this, std::placeholders::_1));

    this->gps_sub = this->create_subscription<common_msgs::msg::NavSatHeading>(
        this->gps_topic, qos, std::bind(&GpsImuFusion::gpsDataCallback, this, std::placeholders::_1));

    /* Create EKF Filter */
    this->gndFusion = new InsFilterNonHolonomic();

    /* Load initialia parameters to the filter */
    this->gndFusion->loadParameters(Vector3d(this->gyroscope_noise[0], this->gyroscope_noise[1], this->gyroscope_noise[2]),
                                    Vector3d(this->gyroscope_bias_noise[0], this->gyroscope_bias_noise[1], this->gyroscope_bias_noise[2]),
                                    Vector3d(this->accelerometer_noise[0], this->accelerometer_noise[1], this->accelerometer_noise[2]),
                                    Vector3d(this->accelerometer_bias_noise[0], this->accelerometer_bias_noise[1], this->accelerometer_bias_noise[2]),
                                    this->r_vel,
                                    this->r_pos,
                                    this->gyroscope_bias_decay_factor,
                                    this->accel_bias_decay_factor,
                                    this->zero_velocity_constraint_noise);

    RCLCPP_INFO(this->get_logger(), "Initial constraints: \n");
    this->gndFusion->printFilterConstraints();

    // this->predictFile.open("/tmp/predict_data.txt",std::ios_base::app);

    /* Initialize the transform broadcaster */
    this->tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);


    /* Create Odometry publisher */
    this->odom_pub = this->create_publisher<nav_msgs::msg::Odometry>(this->odom_topic, qos);
}

void GpsImuFusion::loadParameters()
{
    declare_parameter("generic.imu_topic", "/imu/data");
    declare_parameter("generic.gps_topic", "/gps/position");
    declare_parameter("generic.cones_topic", "/perception/cones");
    declare_parameter("generic.output_odom_topic", "/gps_imu_fusion/odom");
    declare_parameter("generic.enable_logging", false);
    declare_parameter("generic.tf_required_topics", 5);

    /* Declare Sensor Frequency parameters */
    declare_parameter("frequency.imu_fs", 100);
    declare_parameter("frequency.gps_fs", 10);

    /* Declare Sensor Noise parameters */
    declare_parameter("noises.gyro_noise", std::vector<double>{0.0, 0.0, 0.0});
    declare_parameter("noises.gyro_bias_noise", std::vector<double>{0.0, 0.0, 0.0});
    declare_parameter("noises.accel_noise", std::vector<double>{0.0, 0.0, 0.0});
    declare_parameter("noises.accel_bias_noise", std::vector<double>{0.0, 0.0, 0.0});
    declare_parameter("noises.zero_velocity_constraint_noise", 1e-3);
    declare_parameter("noises.r_vel", 0.0);
    declare_parameter("noises.r_pos", 0.005);
    declare_parameter("noises.gyroscope_bias_decay_factor", 1.0);
    declare_parameter("noises.accel_bias_decay_factor", 1.0);
    declare_parameter("noises.gps_acc_unc", 0.005);

    get_parameter("generic.imu_topic", this->imu_topic);
    get_parameter("generic.gps_topic", this->gps_topic);
    get_parameter("generic.cones_topic", this->cones_topic);
    get_parameter("generic.output_odom_topic", this->odom_topic);
    get_parameter("generic.enable_logging", this->enable_logging);
    get_parameter("generic.tf_required_topics", this->tf_required_topics);

    /* Get Sensor Frequency parameters */
    get_parameter("frequency.imu_fs", this->imu_fs);
    get_parameter("frequency.gps_fs", this->gps_fs);

    /* Get Sensor Noise parameters */
    get_parameter("noises.gyro_noise", this->gyroscope_noise);
    get_parameter("noises.gyro_bias_noise", this->gyroscope_bias_noise);
    get_parameter("noises.accel_noise", this->accelerometer_noise);
    get_parameter("noises.accel_bias_noise", this->accelerometer_bias_noise);
    get_parameter("noises.zero_velocity_constraint_noise", this->zero_velocity_constraint_noise);
    get_parameter("noises.r_vel", this->r_vel);
    get_parameter("noises.r_pos", this->r_pos);
    get_parameter("noises.gyroscope_bias_decay_factor", this->gyroscope_bias_decay_factor);
    get_parameter("noises.accel_bias_decay_factor", this->accel_bias_decay_factor);
    get_parameter("noises.gps_acc_unc", this->gps_acc_unc);
}

/* ***** CALLBACKS ***** */

void GpsImuFusion::conesCallback(const visualization_msgs::msg::Marker::SharedPtr data)
{
    if (!this->enable_tf)
    {
        RCLCPP_INFO(this->get_logger(), "ENABLE TF!");
        this->enable_tf = true;
    }
    
}


void GpsImuFusion::imuDataCallback(const sensor_msgs::msg::Imu::SharedPtr imu_data)
{

    if (!this->count_imu_topics == 0)
        this->dt = new rclcpp::Duration(this->now() - this->last_imu_sample);
    
    this->last_imu_sample = this->now();

    rclcpp::Time start, end;
    start = this->now();

    /* Get actual orientation */
    double qx = imu_data->orientation.x;
    double qy = imu_data->orientation.y;
    double qz = imu_data->orientation.z;
    double qw = imu_data->orientation.w;

    /* Don't continue if we the filter location is not initialized */
    if (!this->is_filter_location_initialized)
        return;

    /* Set initial orientation */
    if ((!this->is_filter_orientation_initialized) && (this->enable_tf))
    {
        this->gndFusion->setInitState(
            Vector4d(qw, qx, qy, qz), /* Initial Quaternion */
            Vector3d(-5.46688e-06, 5.79212e-07, -3.9609e-06),
            Vector3d::Zero(),
            Vector3d::Zero(),
            Vector3d(-1.98204e-05, -1.34177e-05, -2.47165e-05));
        this->is_filter_orientation_initialized = true;
        this->gndFusion->printCurrentState();
        /* Create first TF and Odometry msg */
        // this->updateTf(Vector3d(0.0, 0.0, 0.0), tf2::Quaternion(qx, qy, qz, qw), imu_data->header.stamp);        

    }

    /* Convert IMU accel_data to eigen vector */
    /** IMU works with ENU reference frame. This EKF works with a NED reference frame. So:
     * Acceleration on X axis of ENU is the acceleration on Y axis for NED
     * Acceleration on Y axis of ENU is the acceleration on X axis for NED
     * Acceleration on -Z axis of ENU is the acceleration on Z axis for NED
    */
    Vector3d accel_data(imu_data->linear_acceleration.y, imu_data->linear_acceleration.x, -imu_data->linear_acceleration.z);

    // Vector3d gyro_data(imu_data->angular_velocity.x, imu_data->angular_velocity.y, -imu_data->angular_velocity.z);
    Vector3d gyro_data(0.0, 0.0, imu_data->angular_velocity.z);


    /* If vehicle is still don't updated filter state due to heavy drift */
    if(!this->is_vehicle_still)
    {
        /* Predict state */

        /* Update dt */
        if (count_imu_topics == 0)
        {
            this->gndFusion->predict(accel_data, gyro_data, (double)1e-2);
        }
        else
        {
            // this->gndFusion->predict(accel_data, gyro_data, (double)this->dt->nanoseconds()*1e-9);
            this->gndFusion->predict(accel_data, gyro_data, (double)1e-2);
            delete dt;
        }
    }
    end = this->now();
    rclcpp::Duration exe_time = end - start;

    if (this->enable_logging)
        RCLCPP_INFO(this->get_logger(), "PREDICT Exe time (ms): %lf", exe_time.nanoseconds() * 1e-6);

    Vector4d act_orientation;
    Vector3d act_position;
    Vector3d act_velocities;

    /* Get vehicle pose */
    this->gndFusion->pose(act_position, act_orientation, act_velocities);

    this->count_imu_topics++;

    /* Discard first GPS packets. This is because the first iterations contain a lot of noise in the orientation. */
    if (this->count_gps_topics > this->tf_required_topics)
    {
        tf2::Quaternion q;

        /* Convert to ROS 2 frame */
        this->globalToLocalFrame(q, act_position, act_orientation);

        /* Publish TF */
        this->updateTf(act_position, q, imu_data->header.stamp);
    }
}

void GpsImuFusion::gpsDataCallback(const common_msgs::msg::NavSatHeading::SharedPtr data)
{
    rclcpp::Time start, end;
    start = this->now();

    /* Retrieve LLA */
    double latitude = data->gps_data.latitude;
    double longitude = data->gps_data.longitude;
    double altitude = data->gps_data.altitude;

    Vector3d lla = Vector3d(latitude, longitude, altitude);
    Vector3d velocities;
    double dLat, dLon, d;

    /* If this is the first GPS message, set first location and heading to North */
    if (this->prev_loc == Vector3d::Zero())
    {
        this->prev_loc = lla;
    }

    /* Measure distance w.r.t previous location */
    this->measure_distance(dLat, dLon, d, lla, prev_loc);
    velocities << dLat, dLon, 0.0;

    d *= 1000.0; /* Distance in meters */   

    /* Update previous location */
    prev_loc = Vector3d(latitude, longitude, altitude);

    if (d < this->gps_acc_unc)
    {
        if (this->enable_logging)
            RCLCPP_INFO(this->get_logger(), "Vehicle is still. Skipping.");
        this->is_vehicle_still = true;

        return;
    } else
    {
        // this->total_dist += d;
        this->act_dist += d;

        count_gps_topics++;

        this->is_vehicle_still = false;
        
        if ((this->count_gps_topics % this->gps_fs) == 0)
        {
            if (this->enable_logging)
                RCLCPP_INFO(this->get_logger(), "You are traveling at %lf m/s == %lf km/h", this->act_dist, (this->act_dist * 3.6));
            this->act_dist = 0;
        }
    }

    /* Check if this is the first Navsat msg */
    if (!is_filter_location_initialized)
    {
        /* In that case, set reference location */
        this->gndFusion->setRefLocation(latitude, longitude, altitude);
        this->is_filter_location_initialized = true;
        // velocities << 3.29344e-08, 0.0, 0;
        return;
    } 

    /* Don't continue if filter initial orientation is not initialized */
    if (!this->is_filter_orientation_initialized)
        return;

    /* Correct */
    this->gndFusion->fusegps(lla, velocities);

    end = this->now();
    rclcpp::Duration exe_time = end - start;

    if (this->enable_logging)
        RCLCPP_INFO(this->get_logger(), "CORRECT Exe time (ms): %lf", exe_time.nanoseconds() * 1e-6);

    /* First TF and odometry msgs */
    if (this->count_gps_topics < this->tf_required_topics)
    {
        /* Update vehicle TF */
        tf2::Quaternion q;

        Vector4d act_orientation;
        Vector3d act_position;
        Vector3d act_velocities;

        /* Get vehicle pose */
        this->gndFusion->pose(act_position, act_orientation, act_velocities);

        /* Convert to ROS 2 frame */
        this->globalToLocalFrame(q, act_position, act_orientation);

        /* Publish TF */
        // this->updateTf(act_position, q, data->header.stamp);
    }
}

/* ***** END CALLBACKS ***** */
void GpsImuFusion::updateTf(Vector3d p, tf2::Quaternion q, rclcpp::Time timestamp)
{
    if (!this->enable_tf)
    {
        RCLCPP_INFO(this->get_logger(), "No cones -> Not publishing TF.");
        return;
    }
    geometry_msgs::msg::TransformStamped t;
    nav_msgs::msg::Odometry odom;

    /* Update header */
    t.header.stamp = timestamp;
    t.header.frame_id = "track";
    t.child_frame_id = "imu_link";

    /* Update TF location */
    t.transform.translation.x = p[1];   /* P[1] is North position in ENU reference frame */
    t.transform.translation.y = p[0];   /* P[0] is East position in ENU reference frame */
    t.transform.translation.z = 0.0f;


    /* Update TF Orientation */
    t.transform.rotation.x = q.getX();
    t.transform.rotation.y = q.getY();
    t.transform.rotation.z = q.getZ();
    t.transform.rotation.w = q.getW();

    /* Upated Odom header */
    odom.header.stamp = timestamp;
    odom.header.frame_id = "track";
    odom.child_frame_id = "imu_link";

    /* Updated Odom pose */
    odom.pose.pose.position.x = p[1];   /* P[1] is North position in ENU reference frame */
    odom.pose.pose.position.y = p[0];   /* P[0] is East position in ENU reference frame */
    odom.pose.pose.position.z = 0.0f;

    /* Update Odom orientation */
    odom.pose.pose.orientation.x = q.getX();
    odom.pose.pose.orientation.y = q.getY();
    odom.pose.pose.orientation.z = q.getZ();
    odom.pose.pose.orientation.w = q.getW();

    /* Publish new TF and Odometry */
    this->tf_broadcaster_->sendTransform(t);
    this->odom_pub->publish(odom);
}