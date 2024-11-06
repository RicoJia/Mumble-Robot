
## Project Road Map

### Take The Gao-Xiang class. (4 weeks?)

### Finish the 3D RGBD SLAM Implementation (2 weeks)

- Continue on with the Implementation.
- Try writing the whole pipeline using the analytic functions
- Create a docker image on top of the robot's image
- Test the algorithms in there


### Create A 2D Lidar Robot (1-2 weeks)

Objective: Set Up the physical robot with a 2D Lidar, docker, and ROS 2 simulation and nav stack

- Create a docker image that the robot will work with

```
docker: docker run --gpus all -it --rm --name rico_test rwthika/ros2-ml:latest
```
    - Check out the rwthika website and work on those

- Create a simulation of the robot
    - Isaac sim?  https://catalog.ngc.nvidia.com/orgs/nvidia/containers/isaac-sim
    - Go cheap on this. Can steal the URDF from another project. 
    - Being able to Visualize the robot on Rviz

- Test with ROS 2 navstack. 
    - SLAM node
    - Global Planner
    - Local Planner


### Cartographer Implementation (4 weeks)

### Test with 3D Lidar Implementation (4 weeks)

