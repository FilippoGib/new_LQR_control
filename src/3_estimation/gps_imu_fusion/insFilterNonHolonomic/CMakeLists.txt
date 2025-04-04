cmake_minimum_required(VERSION 3.5)
project(ins_filter_non_holonomic)

set(Upstream_VERSION 3.5)

# Include InsFilterNonHolonomicState dependency
include(${CMAKE_CURRENT_SOURCE_DIR}/types/CMakeLists.txt)

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

# find dependencies
find_package(Eigen3 REQUIRED NO_MODULE)

# Find include directories
include_directories(
  include
  "types/include"
)

# Adding InsFilterNonHolonomic library
add_library(ins_filter_non_holonomic SHARED src/InsFilterNonHolonomic.cpp)

target_link_libraries(ins_filter_non_holonomic
  Eigen3::Eigen
  ins_filter_non_holonomic_state
)

# install(TARGETS
#   ins_filter_non_holonomic
#   DESTINATION "/usr/lib"
# )