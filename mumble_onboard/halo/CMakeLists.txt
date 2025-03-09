cmake_minimum_required(VERSION 3.10)

# (Optional) fix the PCL_ROOT variable policy warning
cmake_policy(SET CMP0074 NEW)

project(halo LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(rosbag2_transport REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PCL REQUIRED)

# Ensure we have a release type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE
      Release
      CACHE STRING \"Build type\" FORCE)
endif()

# Add library
add_library(
  halo STATIC
  src/halo.cpp)

target_include_directories(halo PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
                                       ${PCL_INCLUDE_DIRS})

# Plain signature
target_link_libraries(
  halo ${PCL_LIBRARIES})

# Use ament macro to link ROS 2 libraries
ament_target_dependencies(halo rclcpp rosbag2_cpp rosbag2_storage
                          rosbag2_transport sensor_msgs)

# Optionally build tests
option(BUILD_HALO_TESTS "Build halo unit tests" ON)
if(BUILD_HALO_TESTS)
  enable_testing()
  find_package(GTest REQUIRED)
  include_directories(${GTEST_INCLUDE_DIRS})

  add_executable(test_knn tests/test_knn.cpp)
  target_link_libraries(test_knn halo GTest::GTest GTest::Main pthread)
  add_test(NAME test_knn COMMAND test_knn)

  # Linear Fitting Test
  add_executable(test_linear_fitting tests/test_linear_fitting.cpp)
  target_link_libraries(test_linear_fitting halo GTest::GTest GTest::Main
                        pthread)
  add_test(NAME test_linear_fitting COMMAND test_linear_fitting)

  add_executable(test_2dslam tests/test_2dslam.cpp)
  target_link_libraries(test_2dslam halo GTest::GTest GTest::Main pthread)
  add_test(NAME test_2dslam COMMAND test_2dslam)
endif()

ament_package()
