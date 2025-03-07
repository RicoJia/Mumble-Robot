#!/usr/bin/env python3

"""
ROS2 Bag Recorder for IMU and LiDAR messages. 
Features: 
- Can rotate bag file when its size hits xGB
- Uses file-level zstd compression with a high compression level.
"""

import os
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, LaserScan
import rclpy.serialization

# Import rosbag2_py interfaces
from rosbag2_py import SequentialWriter, StorageOptions, ConverterOptions
from mumble_interfaces.mumble_logging import get_logger

logger = get_logger("mumble_bag_recorder")
# TODO
# Define the maximum bag file size (4GB)
MAX_BAG_SIZE = 4 * 1024 * 1024 * 1024  # 4GB in bytes

class BagRecorderNode(Node):
    def __init__(self):
        super().__init__('mumble_bag_recorder')

        self.create_subscription(Imu, "/imu_data", self.imu_callback, 10)
        self.create_subscription(LaserScan, "/scan", self.lidar_callback, 10)

    def imu_callback(self, msg: Imu):
        #TODO Remember to remove
        print(f'Rico: {Imu}')

    def lidar_callback(self, msg: LaserScan):
        #TODO Remember to remove
        print(f'Rico: {msg}')

def main(args=None):
    rclpy.init(args=args)
    bag_recorder = BagRecorderNode()
    logger.info("Hello, mumble_bag_recorder is here!")
    try: 
        rclpy.spin(bag_recorder)
    except KeyboardInterrupt:
        bag_recorder.get_logger().info("Keyboard interrupt received, shutting down.")
    finally:
        bag_recorder.get_logger().info("Shutting down bag recorder node.")
        bag_recorder.destroy_node()
        rclpy.shutdown()

if __name__=="__main__":
    main()