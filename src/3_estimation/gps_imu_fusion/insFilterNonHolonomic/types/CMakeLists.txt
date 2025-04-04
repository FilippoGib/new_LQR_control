cmake_minimum_required(VERSION 3.5)
project(ins_filter_non_holonomic_types)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find include directories
include_directories(
  include
)

# Adding Quarternion library
add_library(quaternion_lib types/src/Quaternion.cpp)
# Adding Gyroscope bias library
add_library(gyrobias_lib types/src/GyroscopeBias.cpp)
# Adding NEDPosition library
add_library(nedposition_lib types/src/NEDPosition.cpp)
# Adding NEDVelocities library
add_library(nedvelocities_lib types/src/NEDVelocities.cpp)
# Adding Accelerometer bias library
add_library(accelbias_lib types/src/AccelerometerBias.cpp)
# Adding InsFilterNonHolonomicState library
add_library(ins_filter_non_holonomic_state types/src/InsFilterNonHolonomicState.cpp)