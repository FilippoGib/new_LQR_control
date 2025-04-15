cmake_minimum_required(VERSION 3.5)
project(mmr_ekf_odometry)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -march=native -O3)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
# find_package(GeographicLib REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(common_msgs REQUIRED)

add_subdirectory(ekf_odom)

include_directories(
  include
  # ${GeographicLib_INCLUDE_DIRS}
)

# Build GPS Odometry lib
# add_library(gps_odometry src/gps_odometry/gps_odometry.cpp)

# target_link_libraries(gps_odometry
#   ${GeographicLib_LIBRARIES}
#   Eigen3::Eigen
# )

# Build GPS test main
# add_executable(gps_test_main src/gps_odometry/test_gps_odometry.cpp)

# target_link_libraries(gps_test_main
#   gps_odometry
# )

# Build IMU Odometry lib
# add_library(imu_odometry src/imu_odometry/imu_odometry.cpp)

# Build Node class
add_library(${PROJECT_NAME} src/mmr_ekf_odometry/${PROJECT_NAME}.cpp)


target_link_libraries(${PROJECT_NAME}
  # gps_odometry
  ekf_odom
)


ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2_ros
  nav_msgs
  visualization_msgs
  common_msgs
)

add_executable(${PROJECT_NAME}_node src/mmr_ekf_odometry/${PROJECT_NAME}_node.cpp)

target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
)

# Install target
# install(TARGETS
#   gps_test_main
#   DESTINATION lib/${PROJECT_NAME}
# )

# Install target
install(TARGETS
  ${PROJECT_NAME}_node
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
