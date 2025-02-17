# HALO (Hybrid Autonomous Localization & Observation)

This is a ROS2-independent static library for Lidar-Inertial-Odometry SLAM.

## Usage

- You don't need to statically link any external depedentcies. All you need is: 

```cmake
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/halo)
target_link_libraries(my_executable
    halo 
)
```