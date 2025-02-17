/**
 * @file Visualization.hpp
 * @author Oriol Gorriz (origovi2000@gmail.com)
 * @brief Contains the Visualization class specification
 * @version 1.0
 * @date 2022-10-31
 * 
 * @copyright Copyright (c) 2022 BCN eMotorsport
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <mmr_base/msg/marker.hpp>
#include <mmr_base/msg/marker_array.hpp>

#include "structures/Way.hpp"
#include "utils/Params.hpp"
#include "utils/definitions.hpp"

/**
 * @brief Class that implements all necessary functions to visualize all
 * the program's results.
 */
class Visualization {
 private:
  /**
   * @brief All Markers publishers.
   */
  rclcpp::Publisher<mmr_base::msg::MarkerArray>::SharedPtr trianglesPub, midpointsPub, wayPub;
  
  /**
   * @brief All parameters related to the Visualization class.
   */
  Params::Visualization params_;

  /**
   * @brief All Markers will be published with this timestamp.
   */
  rclcpp::Time stamp_;

 public:
  /**
   * @brief Construct a new Visualization object.
   */
  Visualization() = default;

  // Singleton pattern
  static Visualization &getInstance();
  Visualization(Visualization const &) = delete;
  void operator=(Visualization const &) = delete;

  /**
   * @brief Method to initialize the Singleton.
   * 
   * @param[in] nh 
   * @param[in] params 
   */
  void init(rclcpp::Node::SharedPtr const nh, const Params::Visualization &params);
  
  /**
   * @brief Sets the \a stamp_ attribute, all Markers will be published with
   * this stamp
   * 
   * @param[in] stamp 
   */
  void setTimestamp(const rclcpp::Time &stamp);

  /**
   * @brief Method to visualize a TriangleSet.
   * 
   * @param[in] triSet 
   */
  void visualize(const TriangleSet &triSet) const;
  
  /**
   * @brief Method to visualize an EdgeSet.
   * 
   * @param[in] edgeSet 
   */
  void visualize(const EdgeSet &edgeSet) const;

  /**
   * @brief Method to visualize a Way.
   * 
   * @param[in] way 
   */
  void visualize(const Way &way) const;
};