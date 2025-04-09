# HALO (Hybrid Autonomous Localization & Observation)

This is a ROS2 static library for Lidar-Inertial-Odometry SLAM. It supports both 2D and 3D LiDAR data through templating, RAII with unique_ptr. It also uses vectorization, SIMD, and thread pooling (C++ 20). Its lower level library is Eigen (no PCL).

## Features

`halo` provides simple 2D and 3D LiDAR-Odometry SLAM Frameworks with high-performance spatial search structures optimized using template metaprogramming. It includes infrastructure for:

High level: 
- 2D LiDAR-only pose graph SLAM
- 2D LiDAR-IMU SLAM

Lower level:

- K-Nearest Neighbor Search
- Scan Matching using ICP
- Submap Generation
- Loop Detection
- Global Map Generation From Submaps

## 2D SLAM Frameworks

<div style="text-align: center;">
    <p align="center">
       <figure>
            <img src="https://i.postimg.cc/43kNnkJg/2025-04-02-09-47-58.png" height="300" alt=""/>
       </figure>
    </p>
</div>

### K-Nearest Neighbor Search

- KD Tree: ~3× faster than PCL's KD Tree, achieving 100% precision and recall.
- OctoTree / QuadTree: Similar speed to PCL’s KD Tree but optimized for structured data.
- NanoFLANN Wrapper: Lightweight KD Tree with 4ms query time.
- Grid Search (2D & 3D): Fast approximate search (4ms at 0.5m resolution) with high recall and precision.

Below is a [summary of their performances](./tests/test_knn.cpp)

| Method                | Query Time (ms) | Recall (%) | Precision (%) | Notes                  |
|----------------------|---------------|------------|-------------|------------------------|
| **halo::KDTree**     | **5.1**  (58 on rpi4b)     | 100        | 100         | 3× faster than PCL     |
| **PCL KD Tree**      | 156   (131 on rpi4b)        | 100        | 100         | Baseline               |
| **halo::OctoTree (and QuadTree for 2D)** | 194   (293 on rpi4b)      | 100        | 100         | Similar to PCL KD Tree |
| **NanoFLANN KD Tree** | 4  (22 on rpi4b)           | 100          | 100           | Fast wrapper           |
| **3D Grid Search**    | 4  (33 on rpi4b)           | 95.6       | 98.1        | Resolution = 0.5m      |

### Scan Matching

### Submap Generation

## Usage

- You don't need to statically link any external depedentcies. All you need is: 

```cmake
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/halo)
target_link_libraries(my_executable
    halo 
)
```

## 3D SLAM Frameworks

### Scan Matching

- [Point-Point ICP](./include/halo/lo3d/icp_3d_methods.hpp)

<p align="center">
  <img src="https://i.postimg.cc/nhF3MYCN/pcl-source.png" height="200"/>
  <img src="https://i.postimg.cc/65LH915d/pcl-target.png" height="200"/>
  <img src="https://i.postimg.cc/MK8PJ8x6/pcl-Merged.png" height="200"/>
    <figcaption style="text-align: center; margin-top: 8px;">
        From Left to Right: Source, Target, Merged Point Clouds
    </figcaption>
</p>


## Conventions:

- angles are wrapped to `[-pi, pi]`. Please adjust the lidar `angle_min` and `angle_max` accordingly

## Generic Tools
- `visualize_pcd.py`: visualize a 3D point cloud in pcd 