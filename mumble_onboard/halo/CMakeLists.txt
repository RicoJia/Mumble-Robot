cmake_minimum_required(VERSION 3.10)

# (Optional) fix the PCL_ROOT variable policy warning
cmake_policy(SET CMP0074 NEW)

project(halo LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS
    "${CMAKE_CXX_FLAGS} -march=native -funroll-loops -ftree-vectorize"
)# for SIMD optimizations
# Propagate Debug flags
if(CMAKE_BUILD_TYPE MATCHES "Debug")
  message(STATUS "Halo: Building in Debug mode with gdb symbols")
  # append -g and -O0 so both the static lib and test executables carry debug
  # info
  set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -pg -ggdb -O0")
  set(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} -pg")
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(rosbag2_transport REQUIRED)
find_package(sensor_msgs REQUIRED)
list(INSERT CMAKE_PREFIX_PATH 0 "/opt/pcl-1.14.0")
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED) # ← must come before
                                       # ament_target_dependencies
find_package(glog REQUIRED)
find_package(OpenCV REQUIRED)
find_package(g2o REQUIRED)
find_package(fmt REQUIRED)
find_package(gflags REQUIRED)

message(STATUS "Found PCL ${PCL_VERSION}") # pcl 1.12.1

# Ensure we have a release type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE
      Release
      CACHE STRING \"Build type\" FORCE)
endif()
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -g ${CMAKE_CXX_FLAGS}")

find_program(CCACHE_PROGRAM ccache)
if(CCACHE_PROGRAM)
  set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE "${CCACHE_PROGRAM}")
  set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK ccache)
  message(STATUS "CCACHE found, using ccache")
else()
  message(STATUS "CCACHE not found, not using ccache")
endif()

# Add library
add_library(
  halo STATIC
  src/halo.cpp)

target_compile_features(halo PUBLIC cxx_std_20)

target_include_directories(
  halo PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include ${PCL_INCLUDE_DIRS}
              ${OpenCV_INCLUDE_DIRS} ${G2O_INCLUDE_DIRS})

set(G2O_LIBS
    g2o_core
    g2o_stuff
    g2o_solver_dense
    g2o_solver_cholmod
    g2o_types_slam2d
    g2o_types_slam3d
    g2o_solver_eigen
    g2o_types_sba
    g2o_csparse_extension
    fmt::fmt)

# Plain signature
target_link_libraries(
  halo ${PCL_LIBRARIES} ${OpenCV_LIBS} ${G2O_LIBS} ${DL_LIB} # Add this line to
                                                             # link libdl
)

# Use ament macro to link ROS 2 libraries
ament_target_dependencies(
  halo
  rclcpp
  rosbag2_cpp
  rosbag2_storage
  rosbag2_transport
  sensor_msgs
  pcl_conversions)

add_executable(
  export_scan_2_txt
  src/export_scan_2_txt.cpp)
target_link_libraries(
  export_scan_2_txt halo pthread ${G2O_LIBS})

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

  # ############################################################################
  # 2D SLAM Tests
  # ############################################################################

  function(add_slam_gtest name)
    add_executable(${name} tests/${name}.cpp)
    target_link_libraries(
      ${name}
      halo
      GTest::GTest
      GTest::Main
      pthread
      ${G2O_LIBS}
      gflags
      glog)
    add_test(NAME ${name} COMMAND ${name} --gtest_catch_exceptions=0)
  endfunction()

  add_slam_gtest(test_math)
  add_slam_gtest(test_2d_slam_visualization)
  add_slam_gtest(test_2d_slam_degredation)
  add_slam_gtest(test_2d_slam_mapping)
  add_slam_gtest(test_2d_slam_icp_methods)
  add_slam_gtest(test_2d_slam_submaps)
  add_slam_gtest(test_2d_slam_likelihood_fields)
  add_slam_gtest(test_2d_slam_occ_map)

  # ############################################################################
  # 3D SLAM Tests
  # ############################################################################
  add_slam_gtest(test_3d_icp)
  add_slam_gtest(test_direct_3d_ndt_lo)
  add_slam_gtest(test_datastructures)
  add_slam_gtest(test_3d_loamlike_feature_extraction)
  add_slam_gtest(test_yaml_loaded_config)

endif()

ament_package()
