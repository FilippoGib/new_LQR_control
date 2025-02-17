cmake_minimum_required(VERSION 3.5)
project(urinay)

# Specify C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_BUILD_TYPE "Release")

# Find dependencies
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(mmr_base REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

# Create library
add_library(${PROJECT_NAME} SHARED
  src/structures/Point.cpp
  src/structures/Node.cpp
  src/structures/Edge.cpp
  src/structures/Circle.cpp
  src/structures/Triangle.cpp
  src/structures/Trace.cpp
  src/structures/Vector.cpp
  src/structures/Way.cpp
  src/modules/DelaunayTri.cpp
  src/modules/WayComputer.cpp
  src/modules/Visualization.cpp
  src/utils/Time.cpp
  src/utils/Params.cpp
  src/utils/Failsafe.cpp
  src/utils/KDTree.cpp
)

# Add executable
add_executable(${PROJECT_NAME}_exec src/main.cpp)

# Link libraries
target_link_libraries(${PROJECT_NAME}_exec ${PROJECT_NAME})


ament_target_dependencies(
  ${PROJECT_NAME} 
  rclcpp
  std_msgs
  sensor_msgs  
  Eigen3
  mmr_base
  nav_msgs
  geometry_msgs
  tf2_eigen
  tf2_geometry_msgs
)

ament_target_dependencies(
  ${PROJECT_NAME}_exec 
  rclcpp
  std_msgs
  sensor_msgs  
  Eigen3
  mmr_base
  nav_msgs
)

# Install targets and include directories
install(TARGETS
  ${PROJECT_NAME}
  ${PROJECT_NAME}_exec
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY include
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# ament package
ament_package()
