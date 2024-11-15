
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
docker run --gpus all -it --rm --name rico_test rwthika/ros2-ml:latest
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

## Part 1 - Get The Rover Up And Running

- manual https://www.waveshare.com/wiki/WAVE_ROVER

When its internal power is depleted, use the provided 12.6V 2A power supply to charge it. It supports charging while in use. Upon booting up, the OLED screen on the robot displays the following information:

- The first line indicates that the WiFi is in AP mode, and the WiFi hotspot is named "UGV".
- The second line indicates that the STA mode is turned off. When the WiFi is in STA mode, the router will assign an IP address, which will be displayed.
- The third line displays the MAC address of this device, which is unique and used for ESP-NOW communication.
- The fourth line indicates the voltage of the product's power supply.

### Wifi Modes

- STA Mode (Station Mode) is when a WiFi acts as a client that connects to an access point (AP). Address could be obtained by DHCP.
- AP Mode: when a Wi-Fi device acts as a hub that for wireless devices (stations) to connect to it.
    - It can broadcast a wifi-network (SSID) that other devices
- After powering on, use your smartphone or computer to connect to the robot's WiFi network named "UGV" with the password "12345678". Once connected to the WiFi, open the Google Chrome browser and type "192.168.4.1" in the address bar to access the web-based user interface. From there, you can use the functionalities of the web interface to control the robot. You can also send JSON commands to the robot from this page.


