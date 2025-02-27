# HALO (Hybrid Autonomous Localization & Observation)

This is a ROS2-independent static library for Lidar-Inertial-Odometry SLAM.

## Features

- Data structures are compile-time memory-efficient with 2D and 3D data. This is done by template meta programming.
- A KD Tree that's ~3 faster than PCL KD tree 
    - [In this test](./tests/test_knn.cpp), PCL spends ~0.156s to find 5 neighbors on ~20k `XYZI` points. `halo::KDTree` spends ~0.051s.

## Usage

- You don't need to statically link any external depedentcies. All you need is: 

```cmake
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/halo)
target_link_libraries(my_executable
    halo 
)
```