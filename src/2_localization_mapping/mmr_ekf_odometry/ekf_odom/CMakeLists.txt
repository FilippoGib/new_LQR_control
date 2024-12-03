cmake_minimum_required(VERSION 3.5)
project(ekf_odom)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 23)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -march=native -O3)
endif()

# find dependencies
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(OpenMP REQUIRED)

# Find include directories
include_directories(
  include
)

# Adding EKFOdom library
add_library(${PROJECT_NAME} SHARED src/${PROJECT_NAME}.cpp)

target_link_libraries(${PROJECT_NAME}
  Eigen3::Eigen
  OpenMP::OpenMP_CXX
)

# Build testing executable 

# add_executable(main test/main.cpp)

# target_link_libraries(main
#   ${PROJECT_NAME}
# )