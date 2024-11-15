cmake_minimum_required(VERSION 3.5)
project(ros2_dream_base_description)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(urdf REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(gazebo_ros REQUIRED)


# xacro is NOT a package

# Include directories
include_directories(
  include
  ${GAZEBO_INCLUDE_DIRS}
)

# Install launch files, meshes, urdf, worlds, etc.
install(DIRECTORY launch meshes urdf worlds config
  DESTINATION share/${PROJECT_NAME}/
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
# install(PROGRAMS
#   ros2_dream_base_description/my_python_node.py
#   DESTINATION lib/${PROJECT_NAME}
# )

add_library(ros2_dream_gazebo_diff_drive_plugin SHARED src/ros2_dream_gazebo_diff_drive_plugin.cpp)
ament_target_dependencies(ros2_dream_gazebo_diff_drive_plugin rclcpp std_msgs geometry_msgs nav_msgs tf2 tf2_ros gazebo_ros)
target_link_libraries(ros2_dream_gazebo_diff_drive_plugin ${GAZEBO_LIBRARIES})
install(TARGETS ros2_dream_gazebo_diff_drive_plugin
  DESTINATION lib/${PROJECT_NAME})

ament_package()

