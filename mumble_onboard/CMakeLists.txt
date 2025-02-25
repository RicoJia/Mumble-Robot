cmake_minimum_required(VERSION 3.5)
project(mumble_onboard)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Ensure that a build type is set
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE
      Release
      CACHE STRING "Build type (Debug, Release, RelWithDebInfo)" FORCE)
endif()
# Define common flags for different build types
if(CMAKE_BUILD_TYPE MATCHES "Debug")
  message(STATUS "Building in Debug mode with gprof and gdb support")
  set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -pg -ggdb -O0")
  set(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} -pg")
elseif(CMAKE_BUILD_TYPE MATCHES "Release")
  message(STATUS "Building in Release mode with optimizations")
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native -O3")
  if(WITH_SSE)
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -msse4.2")
  endif()
  if(WITH_AVX)
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -mavx2")
  endif()
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(PCL REQUIRED)

# Include Cpp "include" directory
include_directories(include)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/halo)
# Create Cpp executable
add_executable(
  mumbo_halo_slam
  src/mumbo_halo_slam.cpp)
# target_include_directories(mumbo_halo_slam PRIVATE )
target_link_libraries(
  mumbo_halo_slam halo)
ament_target_dependencies(mumbo_halo_slam rclcpp)
# Install Cpp executables
install(TARGETS mumbo_halo_slam DESTINATION lib/${PROJECT_NAME})
# Install Python modules
ament_python_install_package(${PROJECT_NAME})
# # Install Python executables install(PROGRAMS scripts/py_node.py DESTINATION
# lib/${PROJECT_NAME} )
ament_package()
