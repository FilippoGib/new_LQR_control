cmake_minimum_required(VERSION 3.5)
project(lqr)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3) 
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(common_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)

include_directories(include)
include_directories(${Boost_INCLUDE_DIRS})

add_library(lqr src/lqr.cpp)

target_include_directories(lqr
  PRIVATE
  ${Boost_INCLUDE_DIRS}
)

ament_target_dependencies(
  lqr 
  rclcpp 
  nav_msgs
  ackermann_msgs
  common_msgs
  visualization_msgs
  tf2
  tf2_ros
  Eigen3
)

add_executable(lqr_node src/lqr_node.cpp)

target_include_directories(lqr_node
  PRIVATE
  ${Boost_INCLUDE_DIRS}
)

target_link_libraries(
  lqr_node
  lqr
)

install(TARGETS
lqr_node
DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY circuits
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
