cmake_minimum_required(VERSION 3.10)
project(halo LANGUAGES CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE
      Release
      CACHE STRING "Build type (Debug, Release, RelWithDebInfo)" FORCE)
endif()
# Use the common flags from the workspace-level CMakeLists.txt
add_definitions(${CMAKE_CXX_FLAGS})

# Add the library as interface since it's header only
add_library(
  halo STATIC
  src/halo.cpp)

target_include_directories(halo PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include/
                                       ${PCL_INCLUDE_DIRS})
target_link_libraries(
  halo
  PUBLIC ${PCL_LIBRARIES} ${PCL_DEFINITIONS} # if needed
)

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
endif()
