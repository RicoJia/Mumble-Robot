# HALO (Hybrid Autonomous Localization & Observation)

This is a ROS2-independent static library for Lidar-Inertial-Odometry SLAM. It supports both 2D and 3D LiDAR data through templating, RAII with unique_ptr. It also uses vectorization, SIMD, and thread pooling (C++ 20). Its lower level library is Eigen (no PCL).

## Features

`halo` provides high-performance spatial search structures optimized for 2D and 3D data using template metaprogramming for compile-time memory efficiency. It includes:

- KD Tree: ~3× faster than PCL's KD Tree, achieving 100% precision and recall.
- OctoTree / QuadTree: Similar speed to PCL’s KD Tree but optimized for structured data.
- NanoFLANN Wrapper: Lightweight KD Tree with 4ms query time.
- Grid Search (2D & 3D): Fast approximate search (4ms at 0.5m resolution) with high recall and precision.

Below is a [summary of their performances](./tests/test_knn.cpp)

| Method                | Query Time (ms) | Recall (%) | Precision (%) | Notes                  |
|----------------------|---------------|------------|-------------|------------------------|
| **halo::KDTree**     | **5.1**       | 100        | 100         | 3× faster than PCL     |
| **PCL KD Tree**      | 156           | 100        | 100         | Baseline               |
| **halo::OctoTree (and QuadTree for 2D)** | 194         | 100        | 100         | Similar to PCL KD Tree |
| **NanoFLANN KD Tree** | 4             | 100          | 100           | Fast wrapper           |
| **3D Grid Search**    | 4             | 95.6       | 98.1        | Resolution = 0.5m      |

## Usage

- You don't need to statically link any external depedentcies. All you need is: 

```cmake
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/halo)
target_link_libraries(my_executable
    halo 
)
```