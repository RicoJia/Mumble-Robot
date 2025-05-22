cmake_minimum_required(VERSION 3.5)
project(mumble_onboard LANGUAGES CXX)

# Ensure we use the same vectorization flags as prebuilt PCL & g2o
add_compile_options(-march=native -msse4.2 -mavx2 -mfpmath=sse)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

option(ENABLE_GPROF "Build with gprof profiling support" OFF)
if(ENABLE_GPROF)
  message(STATUS "==> gprof profiling ENABLED")
  add_compile_options(-pg)
  add_link_options(-pg)
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE
      Release
      CACHE STRING "Build type (Debug, Release, RelWithDebInfo)" FORCE)
endif()

if(CMAKE_BUILD_TYPE MATCHES "Debug")
  message(STATUS "Building in Debug mode with gprof and gdb support")
  set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -pg -ggdb -O0")
  set(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} -pg")
elseif(CMAKE_BUILD_TYPE MATCHES "Release")
  message(STATUS "Building in Release mode with optimizations")
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(PCL REQUIRED)

include_directories(include)

# Pull in Halo subproject
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/halo)

# Main executable
add_executable(
  mumbo_halo_slam
  src/mumbo_halo_slam.cpp)
target_link_libraries(
  mumbo_halo_slam halo)
ament_target_dependencies(mumbo_halo_slam rclcpp)

file(GLOB MUMBLE_SCRIPTS "${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.py")

# Install Python scripts
install(PROGRAMS ${MUMBLE_SCRIPTS} DESTINATION lib/${PROJECT_NAME})

# Install C++ binary
install(TARGETS mumbo_halo_slam DESTINATION lib/${PROJECT_NAME})

ament_package()
