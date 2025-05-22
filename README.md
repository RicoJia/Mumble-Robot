# Mumble-Robot
This is a ROS2 rover that is able to work with 2D and 3D SLAM methods

## How to Launch
1. `mkdir -p Mumble-Robot && cd Mumble-Robot`
1. `git clone git@github.com:RicoJia/Mumble-Robot.git`
1. `mv Mumble-Robot src`
1. `colcon build`
1. `./launch_mumble.sh`

## Project Overview

[Draw.io architecture can be seen here](https://drive.google.com/file/d/1xhYCnX8CW-aLHqrS937fNR_KCQ_xAEB0/view?usp=sharing)

### Docker System

The docker compose orchestration architecture:

<div style="text-align: center;">
<p align="center">
    <figure>
        <img src="https://github.com/user-attachments/assets/380d3e80-478a-41f9-8de0-0a1f552c23fd" height="300" alt=""/>
    </figure>
</p>
</div>

## Tools

- `save_bag_scans_to_pcd.py`: save point cloud2 messages into individual pcd files on a specified path
- `pcd_consecutive_viewer.py`: visualize `.pcd` files in a directory using arrow keys. It also automatically saves the camera perspective from last time in a `.json` file.
- `run_ndt_test.sh`: run the entire 3d SLAM pipeline, and visualize.

