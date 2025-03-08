cmake_minimum_required(VERSION 3.8)
project(mumble_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME} "srv/MotorCommand.srv")

install(
  DIRECTORY mumble_interfaces/
  DESTINATION local/lib/python3.10/dist-packages/mumble_interfaces
  FILES_MATCHING
  PATTERN "*.py")


ament_export_dependencies(rosidl_default_runtime)
ament_package()
