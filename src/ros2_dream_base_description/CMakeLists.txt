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
# xacro is NOT a package

# Include directories
include_directories(
  include
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

# If you have C++ executables, add them here
# add_executable(my_cpp_node src/my_cpp_node.cpp)
# ament_target_dependencies(my_cpp_node rclcpp gazebo_ros ...)
# install(TARGETS my_cpp_node
#   DESTINATION lib/${PROJECT_NAME}
# )

ament_package()

