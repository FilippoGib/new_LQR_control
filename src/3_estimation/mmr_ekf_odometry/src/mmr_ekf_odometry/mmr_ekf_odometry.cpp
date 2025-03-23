#include <mmr_ekf_odometry/mmr_ekf_odometry.hpp>

MmrEKFOdometry::MmrEKFOdometry() : rclcpp::Node("mmr_ekf_odometry_node") {
  /* Load node parameters */
  this->loadParameters();

  /* Define QoS for Best Effort messages transport */
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);

  /* Create publisher */
  this->odom_pub = this->create_publisher<nav_msgs::msg::Odometry>(
      this->output_odom_topic, 1);
  this->conesPositionsMarkerPub =
      this->create_publisher<visualization_msgs::msg::Marker>(
          "/slam/cones_positions", 1);

  /* Create subscriptions */
  this->cones_sub = this->create_subscription<visualization_msgs::msg::Marker>(
      this->cones_topic, qos,
      std::bind(&MmrEKFOdometry::conesCallback, this, std::placeholders::_1));

  // this->gps_speed_sub = this->create_subscription<visualization_msgs::msg::GpsSpeed>(
  //     this->gps_speed_topic, qos,
  //     std::bind(&MmrEKFOdometry::gpsSpeedDataCallback, this,
  //     std::placeholders::_1));

  // this->gps_data_sub =
  // this->create_subscription<visualization_msgs::msg::NavSatHeading>(
  //     this->gps_data_topic, qos, std::bind(&MmrEKFOdometry::llaToUTM, this,
  //     std::placeholders::_1));

  // this->imu_odom_sub = this->create_subscription<sensor_msgs::msg::Imu>(
  //     this->imu_topic, qos, std::bind(&MmrEKFOdometry::imuDataCallback, this,
  //     std::placeholders::_1));

  this->fast_lio_odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
      this->input_odom_topic, qos,
      std::bind(&MmrEKFOdometry::fastLioDataCallback, this,
                std::placeholders::_1));

  this->race_status_sub = this->create_subscription<common_msgs::msg::RaceStatus>(
      this->race_status_topic, qos,
      std::bind(&MmrEKFOdometry::raceStatusCallback, this,
                std::placeholders::_1));

  /* Initialize the transform broadcaster */
  this->tf_broadcaster_ =
      std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  /* Initialize vehicle pose */
  this->act_position << 0.0, 0.0, 0.0;
  this->act_orientation.set__w(1.0);
  this->act_orientation.set__x(0.0);
  this->act_orientation.set__y(0.0);
  this->act_orientation.set__z(0.0);

  /* Create EKF SLAM filter object */
  this->ekf_odom = std::make_shared<EKFOdom>(this->proc_noise, this->meas_noise,
                                             this->min_new_cone_distance);

  /* Init mapped cones markers */
  this->initConesMarker(this->conesMarker);
  this->initConesMarker(this->correctedConesMarker);

  /* Init race status */
  this->race_status = common_msgs::msg::RaceStatus();
  this->race_status.current_lap = 0;
}

void MmrEKFOdometry::loadParameters() {
  std::vector<double> tmp_proc_noise(2), tmp_meas_noise(3);

  declare_parameter("generic.cones_topic", "/perception/cones");
  declare_parameter("generic.imu_topic", "/imu/data");
  declare_parameter("generic.input_odom_topic", "/Odometry/fastLioOdom");
  declare_parameter("generic.output_odom_topic", "/Odometry");
  declare_parameter("generic.gps_speed_topic", "/speed/gps");
  declare_parameter("generic.gps_data_topic", "/gps/data");
  declare_parameter("generic.race_status_topic", "/planning/race_status");
  declare_parameter("generic.enable_logging", false);
  declare_parameter("generic.cone_time_seen_th", 2);
  declare_parameter("generic.is_skidpad_mission", false);

  /* Declare Sensor Noise parameters */
  RCLCPP_INFO(this->get_logger(), "test");
  declare_parameter<std::vector<double>>("noises.proc_noise", std::vector<double>{0.0, 0.0});
  declare_parameter<std::vector<double>>("noises.meas_noise", std::vector<double>{0.0, 0.0, 0.0});
  declare_parameter("noises.min_new_cone_distance", 2.0);
  RCLCPP_INFO(this->get_logger(), "test");

  get_parameter("generic.cones_topic", this->cones_topic);
  get_parameter("generic.imu_topic", this->imu_topic);
  get_parameter("generic.input_odom_topic", this->input_odom_topic);
  get_parameter("generic.output_odom_topic", this->output_odom_topic);
  get_parameter("generic.gps_speed_topic", this->gps_speed_topic);
  get_parameter("generic.gps_data_topic", this->gps_data_topic);
  get_parameter("generic.race_status_topic", this->race_status_topic);
  get_parameter("generic.enable_logging", this->enable_logging);
  get_parameter("generic.is_skidpad_mission", this->is_skidpad_mission);

  RCLCPP_INFO(this->get_logger(), "IS_SKIDPAD: %u", this->is_skidpad_mission);

  /* Get Sensor Noise parameters */
  get_parameter("noises.proc_noise", tmp_proc_noise);
  get_parameter("noises.meas_noise", tmp_meas_noise);
  get_parameter("noises.min_new_cone_distance", this->min_new_cone_distance);
  get_parameter("generic.cone_time_seen_th", this->cone_time_seen_th);

  /* Copy noise parameters */
  for (size_t i = 0; i < 3; i++) {
    if (i != 2) {
      this->proc_noise(i) = (float)tmp_proc_noise[i];
    }
    this->meas_noise(i) = (float)tmp_meas_noise[i];
  }
}

void MmrEKFOdometry::initConesMarker(visualization_msgs::msg::Marker &cones) {
  cones.header.frame_id = "track";
  cones.ns = "ConesAbsolutePos";
  cones.id = 0;
  cones.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  cones.action = visualization_msgs::msg::Marker::ADD;
  cones.scale.x = 0.35;
  cones.scale.y = 0.35;
  cones.scale.z = 0.35;
  cones.pose.position.x = 0.0;
  cones.pose.position.y = 0.0;
  cones.pose.position.z = 0.0;
  // Initialize with identity quaternion (no rotation)
  cones.pose.orientation.w = 1.0;
  cones.pose.orientation.x = 0.0;
  cones.pose.orientation.y = 0.0;
  cones.pose.orientation.z = 0.0;
}

/* Callbacks */

void MmrEKFOdometry::conesCallback(
    const visualization_msgs::msg::Marker::SharedPtr cones_data) {
  /* If corrected cons are created -> Skip callback */
  // if(this->corrected_cones_created)
  // {
  //     return;
  // }
  size_t detected_cones = cones_data->points.size();

  Vector3f *z = (Vector3f *)malloc(detected_cones * sizeof(Vector3f));
  for (size_t i = 0; i < detected_cones; i++) {
    z[i](0) = sqrt(pow(cones_data->points[i].x, 2) +
                   pow(cones_data->points[i].y, 2)); // + coneRadius;
    z[i](1) = atan2(cones_data->points[i].y, cones_data->points[i].x);
    z[i](2) = this->getConeColor(cones_data->colors[i]);

    /* Normalize angle */
    z[i](1) = this->ekf_odom->normalizeAngle(z[i](1));

    // RCLCPP_INFO(this->get_logger(), "NODE: INDEX %u CONE X: %lf. CONE Y:
    // %lf", i, cones_data->points[i].x, cones_data->points[i].y);
  }

  rclcpp::Time start, end;
  start = this->now();
  this->ekf_odom->correct(z, detected_cones);
  end = this->now();
  rclcpp::Duration exe_time = end - start;
  if (this->enable_logging)
    RCLCPP_INFO(this->get_logger(), "CORRECT Exe time (ms): %lf",
                exe_time.nanoseconds() * 1e-6);
  // RCLCPP_INFO_STREAM(this->get_logger(), "FILTER STATE: " <<
  // this->ekf_odom->getState());

  free(z);

  /* Get vehicle pose */
  Vector3f xytheta = this->ekf_odom->getState().head(3);

  this->act_position << xytheta(0), xytheta(1), 0.0;

  this->act_yaw = xytheta(2);

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, this->act_yaw);
  this->act_orientation.set__x(q.getX());
  this->act_orientation.set__y(q.getY());
  this->act_orientation.set__z(q.getZ());
  this->act_orientation.set__w(q.getW());

  /* Publish cones */
  if (!this->corrected_cones_created) {
    size_t mapped_cones = this->ekf_odom->getActMappedLandmarks();
    conesMarker.points.reserve(mapped_cones);
    conesMarker.colors.reserve(mapped_cones);

    for (size_t i = 0; i < mapped_cones; i++) {
      /* Get how many times the i-th cone has been seen */
      std::pair<ColorId, uint32_t> cone_info =
          this->ekf_odom->getSignatures()(i).getConeColorAndCount();
      // uint16_t seen_threshold = cone_time_seen_th *
      // (this->race_status.current_lap);

      // RCLCPP_INFO(this->get_logger(), "SEEN TH: %u", seen_threshold);

      /* If less than 5 times, don't map it */
      if (cone_info.second < cone_time_seen_th)
        continue;
      Vector2f cone = this->ekf_odom->getState().segment(3 + (i * 2), 2);
      geometry_msgs::msg::Point p;
      std_msgs::msg::ColorRGBA c;
      this->setConeColor(c, static_cast<uint8_t>(cone_info.first));
      p.x = cone(0);
      p.y = cone(1);
      p.z = 0.0;
      conesMarker.points.push_back(p);
      conesMarker.colors.push_back(c);
    }

    if (this->race_status.current_lap > 1) {
      this->ekf_odom->setFirstLapCompleted(true);
      RCLCPP_INFO(this->get_logger(), "CREATING CORRECTED CONES");
      this->corrected_cones_created = true;
      this->corrected_cones_size = mapped_cones;
      this->correctedConesMarker = conesMarker;
    }
  }

  /* Publish vehicle pose and cones position */
  if (!this->corrected_cones_created) {
    this->pubConesMarkers(conesMarker);
    conesMarker.points.clear();
    conesMarker.colors.clear();
  } else {
    this->pubConesMarkers(correctedConesMarker);
  }

  this->updatePose();
}

// void MmrEKFOdometry::gpsSpeedDataCallback(
//     const visualization_msgs::msg::GpsSpeed::SharedPtr gps_speed_data) {
//   this->ekf_odom->setActVel(gps_speed_data->gps_speed_mps);
//   this->ekf_odom->predict(0.1);
// }

void MmrEKFOdometry::imuDataCallback(
    const sensor_msgs::msg::Imu::SharedPtr imu_data) {
  // this->ekf_odom->setActAngVel(imu_data->angular_velocity.z);
  tf2::Quaternion q;
  q.setX(imu_data->orientation.x);
  q.setY(imu_data->orientation.y);
  q.setZ(imu_data->orientation.z);
  q.setW(imu_data->orientation.w);
  tf2::Matrix3x3 m(q);

  double r, p, y;
  m.getRPY(r, p, y);

  this->act_yaw = y;

  q.setRPY(0.0, 0.0, this->act_yaw);
  this->act_orientation.set__x(q.getX());
  this->act_orientation.set__y(q.getY());
  this->act_orientation.set__z(q.getZ());
  this->act_orientation.set__w(q.getW());
}

void MmrEKFOdometry::fastLioDataCallback(
    const nav_msgs::msg::Odometry::SharedPtr fast_lio_data) {
  // RCLCPP_INFO(this->get_logger(), "FAST LIO CB");
  tf2::Quaternion q;
  q.setX(fast_lio_data->pose.pose.orientation.x);
  q.setY(fast_lio_data->pose.pose.orientation.y);
  q.setZ(fast_lio_data->pose.pose.orientation.z);
  q.setW(fast_lio_data->pose.pose.orientation.w);
  tf2::Matrix3x3 m(q);

  double r, p, y;
  m.getRPY(r, p, y);

  Vector3f pose;
  pose << fast_lio_data->pose.pose.position.x,
      fast_lio_data->pose.pose.position.y, y;
  Vector3f pose_cov;

  /* Retrieve FAST-LIO Pose covariance. The covariance of FAST-LIO is a 6x6. The
     necessary pose covariance is in the main diagonal at indexes 0 (X), 7 (Y),
     35 (Yaw).
  */
  pose_cov(0) = fast_lio_data->pose.covariance.at(0);  /* X covariance */
  pose_cov(1) = fast_lio_data->pose.covariance.at(7);  /* Y covariance */
  pose_cov(2) = fast_lio_data->pose.covariance.at(35); /* Yaw covariance */

  this->ekf_odom->setPose(pose);
  this->ekf_odom->setPoseCovariance(pose_cov);

  /* If this is the second lap, retrieve ONLY vehicle position, and publish
   * already mapped cones. */
  if (this->corrected_cones_created || this->is_skidpad_mission) {
    /* Get vehicle pose */
    this->act_position << pose(0), pose(1), 0.0;
    this->act_yaw = pose(2);

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, this->act_yaw);
    this->act_orientation.set__x(q.getX());
    this->act_orientation.set__y(q.getY());
    this->act_orientation.set__z(q.getZ());
    this->act_orientation.set__w(q.getW());

    if (!this->is_skidpad_mission) {
      this->pubConesMarkers(correctedConesMarker);
    }

    this->updatePose();
  }
}

void MmrEKFOdometry::pubConesMarkers(visualization_msgs::msg::Marker &cones) {
  cones.header.stamp = this->now();
  this->conesPositionsMarkerPub->publish(cones);
}

void MmrEKFOdometry::updatePose() {
  geometry_msgs::msg::TransformStamped t;
  nav_msgs::msg::Odometry _odom;

  /* Update header */
  t.header.stamp = this->now();
  t.header.frame_id = "track";
  t.child_frame_id = "imu_link";

  /* Update TF location */
  t.transform.translation.x = act_position[0];
  t.transform.translation.y = act_position[1];
  t.transform.translation.z = 0.0f;

  /* Update TF Orientation */
  t.transform.rotation = this->act_orientation;

  _odom.header.stamp = this->now();
  _odom.header.frame_id = "track";
  _odom.child_frame_id = "imu_link";

  /* Update Odom position */
  _odom.pose.pose.position.x = act_position[0];
  _odom.pose.pose.position.y = act_position[1];
  _odom.pose.pose.position.z = 0.0f;

  /* Update Odom orientation */
  _odom.pose.pose.orientation = this->act_orientation;

  /* Update Odom covariance (just the main diagonal)*/
  Vector3f pose_cov = this->ekf_odom->getPoseCovariance();

  _odom.pose.covariance.at(0) = pose_cov(0);  /* X Covariance */
  _odom.pose.covariance.at(7) = pose_cov(1);  /* Y Covariance */
  _odom.pose.covariance.at(35) = pose_cov(2); /* Yaw Covariance */

  this->tf_broadcaster_->sendTransform(t);
  this->odom_pub->publish(_odom);
}

void MmrEKFOdometry::raceStatusCallback(
    const common_msgs::msg::RaceStatus::SharedPtr race_status_data) {
  /* Update race status */
  this->race_status = *race_status_data;
}

// void MmrEKFOdometry::llaToUTM(
//     const visualization_msgs::msg::NavSatHeading::SharedPtr gps_data) {
//   double yaw = 0.0;
//   Vector2d act_lat_lon_position;
//   act_lat_lon_position << gps_data->gps_data.latitude,
//       gps_data->gps_data.longitude;
//   if (this->is_first_gps_topic) {
//     if (gps_data->is_heading_reliable) {
//       RCLCPP_INFO(this->get_logger(),
//                   "HEADING RELIABLE. HEADING IS: %lf rad == %lf deg",
//                   gps_data->heading_rad, rad2deg(gps_data->heading_rad));
//       this->obj_gps_odom = new GPSOdometry(act_lat_lon_position);
//       this->is_first_gps_topic = false;
//     }
//   } else {
//     Vector3d gps_pos =
//         this->obj_gps_odom->getGlobalOdometry(act_lat_lon_position);

//     double yaw_diff = atan2(gps_pos(1), gps_pos(0));
//     RCLCPP_INFO(this->get_logger(), "yaw diff is: %lf rad == %lf deg", yaw_diff,
//                 rad2deg(yaw_diff));

//     if (!this->is_yaw_offset_initialized) {

//       this->obj_gps_odom->setYawOffset(yaw_diff);
//       this->is_yaw_offset_initialized = true;
//     }

//     Vector2d local_pos = this->obj_gps_odom->getLocalOdometry(gps_pos);

//     this->act_position << local_pos(0), local_pos(1), 0.0;
//     this->updatePose();
//   }
// }