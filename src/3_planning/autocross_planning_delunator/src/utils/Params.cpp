/**
 * @file Param.cpp
 * @author Oriol Gorriz (origovi2000@gmail.com)
 * @brief Contains the Param class member functions implementation
 * @version 1.0
 * @date 2022-10-31
 * 
 * @copyright Copyright (c) 2022 BCN eMotorsport
 */

#include "utils/Params.hpp"

/* ----------------------------- Private Methods ---------------------------- */

/* ----------------------------- Public Methods ----------------------------- */

Params::Params(rclcpp::Node::SharedPtr const nh) {
  std::string ns = nh->get_namespace();
  // Main
  main.package_path = ament_index_cpp::get_package_share_directory("urinay");
  main.input_cones_topic = nh->declare_parameter<std::string>(ns + "/input_cones_topic", "/AS/C/ccat/cones");
  main.input_pose_topic = nh->declare_parameter<std::string>(ns + "/input_pose_topic", "/AS/C/state");
  main.output_full_center_topic = nh->declare_parameter<std::string>(ns + "/output_full_center_topic", "/AS/P/tracklimits/fullCenter");
  main.output_full_left_topic = nh->declare_parameter<std::string>(ns + "/output_full_left_topic", "/AS/P/tracklimits/fullLeft");
  main.output_full_right_topic = nh->declare_parameter<std::string>(ns + "/output_full_right_topic", "/AS/P/tracklimits/fullRight");
  main.output_partial_center_topic = nh->declare_parameter<std::string>(ns + "/output_partial_center_topic", "/AS/P/tracklimits/partialCenter");
  main.output_partial_left_topic = nh->declare_parameter<std::string>(ns + "/output_partial_left_topic", "/AS/P/tracklimits/partialLeft");
  main.output_partial_right_topic = nh->declare_parameter<std::string>(ns + "/output_partial_right_topic", "/AS/P/tracklimits/partialRight");
  
  main.shutdown_on_loop_closure = nh->declare_parameter<bool>(ns + "/shutdown_on_loop_closure", true);
  main.min_cone_confidence = nh->declare_parameter<float>(ns + "/min_cone_confidence", 0.0);

  // WayComputer
  wayComputer.max_triangle_edge_len = nh->declare_parameter<double>(ns + "/max_triangle_edge_len", 9.0);
  wayComputer.min_triangle_angle = nh->declare_parameter<double>(ns + "/min_triangle_angle", 0.25);
  wayComputer.max_dist_circum_midPoint = nh->declare_parameter<double>(ns + "/max_dist_circum_midPoint", 1.0);
  wayComputer.failsafe_max_way_horizon_size = nh->declare_parameter<int>(ns + "/failsafe_max_way_horizon_size", 6);
  wayComputer.general_failsafe = nh->declare_parameter<bool>(ns + "/general_failsafe", true);
  wayComputer.general_failsafe_safetyFactor = nh->declare_parameter<double>(ns + "/general_failsafe_safetyFactor", 1.4);
  
  // WayComputer::Search
  wayComputer.search.max_way_horizon_size = nh->declare_parameter<int>(ns + "/max_way_horizon_size", 0);
  wayComputer.search.max_search_tree_height = nh->declare_parameter<int>(ns + "/max_search_tree_height", 5);
  wayComputer.search.search_radius = nh->declare_parameter<double>(ns + "/search_radius", 5.0);
  wayComputer.search.max_angle_diff = nh->declare_parameter<double>(ns + "/max_angle_diff", 0.6);
  wayComputer.search.edge_len_diff_factor = nh->declare_parameter<double>(ns + "/edge_len_diff_factor", 0.5);
  wayComputer.search.max_search_options = nh->declare_parameter<int>(ns + "/max_search_options", 2);
  wayComputer.search.max_next_heuristic = nh->declare_parameter<double>(ns + "/max_next_heuristic", 3.0);
  wayComputer.search.heur_dist_ponderation = nh->declare_parameter<float>(ns + "/heur_dist_ponderation", 0.6);
  wayComputer.search.allow_intersection = nh->declare_parameter<bool>(ns + "/allow_intersection", false);
  wayComputer.search.max_treeSearch_time = nh->declare_parameter<float>(ns + "/max_treeSearch_time", 0.05);

  // WayComputer::Way
  wayComputer.way.max_dist_loop_closure = nh->declare_parameter<double>(ns + "/max_dist_loop_closure", 1.0);
  wayComputer.way.max_angle_diff_loop_closure = nh->declare_parameter<double>(ns + "/max_angle_diff_loop_closure", 0.6);
  wayComputer.way.vital_num_midpoints = nh->declare_parameter<int>(ns + "/vital_num_midpoints", 5);

  // Visualization
  visualization.publish_markers = nh->declare_parameter<bool>(ns + "/publish_markers", false);
  visualization.triangulation_topic = nh->declare_parameter<std::string>(ns + "/marker_topics/triangulation", "/AS/P/urinay/markers/triangulation");
  visualization.midpoints_topic = nh->declare_parameter<std::string>(ns + "/marker_topics/midpoints", "/AS/P/urinay/markers/midpoints");
  visualization.way_topic = nh->declare_parameter<std::string>(ns + "/marker_topics/way", "/AS/P/urinay/markers/way");
}