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

## 3D SLAM Components & Frameworks

### Lidar Odometers

- NDT Odometer & Incremental NDT Odometer

    <div style="text-align: center;">
    <p align="center">
        <figure>
            <img src="https://i.postimg.cc/8cT7R7LN/ndt-sputnik.gif" height="300" alt=""/>
        </figure>
    </p>
    </div>

- LOAM-Like Odometer

    <div style="text-align: center;">
    <p align="center">
        <figure>
            <img src="./media/ndt-sputnik.gif" height="300" alt=""/>
        </figure>
    </p>
    </div>

TODO: remove me



### Scan Matching

We are using [2 datasets from EPFL](https://lgg.epfl.ch/statues.php) as benchmarks: Kneeling Lady and Aquarius. Our task is to match the source to the target point cloud. Below are comparisons between [Point-Point ICP, Point-Line ICP, Point-Plane ICP, NDT, and PCL ICP](./include/halo/lo3d/icp_3d_methods.hpp)

<p align="center">
  <img src="https://i.postimg.cc/1z8gGxbw/kneeling-lady-unaligned.png" height="200"/>
  <img src="https://i.postimg.cc/mg21r1wd/kneeling-lady-pt-pt.png" height="200"/>
  <img src="https://i.postimg.cc/ydSZHqPs/kneeling-lady-pt-line-png.png" height="200"/>
  <img src="https://i.postimg.cc/VLFb1Wjr/kneeling-lady-pt-plane.png" height="200"/>
  <img src="https://i.postimg.cc/V6mZsw9p/kneeling-lady-ndt.png" height="200"/>
    <figcaption style="text-align: center; margin-top: 8px;">
    From Left to Right: Original Unaligned, Pt-Pt ICP, Pt-Line ICP, NDT of The Kneeling Lady Dataset
    </figcaption>
</p>


<p align="center">
  <img src="https://i.postimg.cc/wjPhLrCn/aquarius-unaligned.png" height="200"/>
  <img src="https://i.postimg.cc/B6kj9Wv5/aquarius-pt-pt.png" height="200"/>
  <img src="https://i.postimg.cc/cHrvb1d4/aquarius-pt-line.png" height="200"/>
  <img src="https://i.postimg.cc/jSCHWTFz/aquarius-pt-plane-merged.png" height="200"/>
  <img src="https://i.postimg.cc/k5xhdGSW/aquarius-NDT.png" height="200"/>
    <figcaption style="text-align: center; margin-top: 8px;">
        From Left to Right: Original Unaligned, Pt-Pt ICP, Pt-Line ICP, NDT of The Aquarius Dataset
    </figcaption>
</p>

We can tell that there are no visual matching anomalies between the algorithms. The Profiling results are:

| Dataset          | Method     | Time (ms) | Iterations | Error   |
|----------------|------------|-----------|------------|---------|
| Kneeling Lady  | pt-pt      | 87       | 12         | 0.079   |
| Kneeling Lady  | pt-line    | 80       | 7          | 0.00002   |
| Kneeling Lady  | pt-plane   | 55       | 3          | 0.00002   |
| Kneeling Lady  | NDT        | 17       | 3          | 0.029   |
| Kneeling Lady  | PCL-ICP        | 764 | N/A          | 0.04   |
| Kneeling Lady  | PCL-NDT        | 119       | N/A          | 0.16   |
| Aquarius       | pt-pt      | 252       | 23         | 0.001   |
| Aquarius       | pt-line    | 138       | 10         | 0.0005   |
| Aquarius       | pt-plane   | 122       | 4          | 0.0005   |
| Aquarius       | NDT        | 52        | 8          | 0.27   |
| Aquarius       | PCL-ICP        | 2453 | 8          | 0.07   |
| Aquarius  | PCL-NDT        | 213       | N/A          | 0.118   |

### Direct NDT Lidar-Odometry

This implementation is a proof of concept with SIMD and multi-threaded optimzations on sub-components. However, in this implementation, front end scan matching and back end optimization are done in the same thread.

## Conventions

- angles are wrapped to `[-pi, pi]`. Please adjust the lidar `angle_min` and `angle_max` accordingly

## Generic Tools
- `visualize_pcd.py`: visualize a 3D point cloud in pcd 
- `yaml_loaded_config.hpp`: handy configuration object that can be set by an yaml file without manual loading every single field

## Acknowledgements

[Without Dr.Gao Xiang's great work in LiDAR SLAM](https://github.com/gaoxiang12), this repo could not happen. Thank you Dr. Gao!