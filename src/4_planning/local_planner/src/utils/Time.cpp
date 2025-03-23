/**
 * @file Time.cpp
 * @author Oriol Gorriz (origovi2000@gmail.com)
 * @brief Contains the Time class member functions implementation
 * @version 1.0
 * @date 2022-10-31
 * 
 * @copyright Copyright (c) 2022 BCN eMotorsport
 */

#include "utils/Time.hpp"

std::map<std::string, rclcpp::Time> Time::clocks_;

void Time::tick(const std::string &clockName) {
  std::map<std::string, rclcpp::Time>::iterator it = clocks_.find(clockName);
  if (it != clocks_.end()) {
    RCLCPP_WARN(rclcpp::get_logger(""), "[local_planner] Called tick() two times with same clockName before calling tock()");
    it->second = rclcpp::Time(std::chrono::steady_clock::now().time_since_epoch().count());
  } else {
    clocks_.emplace(clockName, rclcpp::Time(std::chrono::steady_clock::now().time_since_epoch().count()));
  }
}

rclcpp::Duration Time::tock(const std::string &clockName) {
  std::map<std::string, rclcpp::Time>::iterator it = clocks_.find(clockName);
  rclcpp::Duration res(0, 0);
  if (it == clocks_.end()) {
    RCLCPP_ERROR(rclcpp::get_logger(""), "[local_planner] Called tock() before calling tick()");
  } else {
    res = rclcpp::Time(std::chrono::steady_clock::now().time_since_epoch().count()) - it->second;
    RCLCPP_INFO_STREAM(rclcpp::get_logger(""), "[local_planner] " << it->first << " has taken: " << res.seconds() * 1e3 << "ms");
    clocks_.erase(it);
  }
  return res;
}
