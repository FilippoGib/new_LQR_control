cmake_minimum_required(VERSION 3.8)
project(LQR_lateral_control)

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

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

include_directories(
                include
)

add_library(LQR_lateral_control src/LQR_lateral_control.cpp)

ament_target_dependencies(LQR_lateral_control
    rclcpp
    )

add_executable(LQR_lateral_control_node src/LQR_lateral_control_node.cpp)

target_link_libraries(LQR_lateral_control_node LQR_lateral_control)

install(TARGETS
  LQR_lateral_control_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install (DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
