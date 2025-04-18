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
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include)

add_library(lqr src/lqr.cpp)

ament_target_dependencies(
  lqr 
  rclcpp 
  nav_msgs
  ackermann_msgs
  visualization_msgs
  tf2
  tf2_ros
  Eigen3
)

add_executable(lqr_node src/lqr_node.cpp)

target_link_libraries(
  lqr_node
  lqr
)

install(TARGETS
lqr_node
DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  circuits/vallelunga1_circuit.csv
  circuits/vallelunga1_circuit_undersampled.csv
  circuits/vallelunga_with_curvatures_modified.csv
  circuits/vallelunga_x_y_r_v.csv
  circuits/varano_autocross.csv
  circuits/varano_autocross_x_y_r_v.csv
  circuits/varano_x_y_r_v.csv
  circuits/rettilineo.csv
  circuits/xy_int.csv
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
