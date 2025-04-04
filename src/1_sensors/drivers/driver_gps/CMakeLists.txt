cmake_minimum_required(VERSION 3.5)
project(driver_gps)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

link_libraries(m)
link_libraries(gps)


# Add compile option for GPS library
add_link_options("-lgps")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(common_msgs REQUIRED)
find_package (GeographicLib REQUIRED)
# Find include directories
include_directories(
  include
  ${GeographicLib_INCLUDE_DIRS}
)

# Adding driver_gps class dependency
add_library(driver_gps src/driver_gps/driver_gps.cpp)

# Targeting ROS2 dependencies
ament_target_dependencies(driver_gps
  rclcpp
  sensor_msgs
  common_msgs
)

target_link_libraries(driver_gps
  ${GeographicLib_LIBRARIES}
)

add_executable(driver_gps_node src/driver_gps/driver_gps_node.cpp)

target_link_libraries(driver_gps_node
  driver_gps
)


# Install target
install(TARGETS
  driver_gps_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch directory
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config directory
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
