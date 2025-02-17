cmake_minimum_required(VERSION 3.5)
project(mumble_onboard)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
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
