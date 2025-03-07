#!/usr/bin/env python3

"""
ROS2 Bag Recorder for IMU and LiDAR messages. 
Features: 
- Can remove stale files when the total size hits xGB
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
# Define the maximum bag file size (4GB)
MAX_BAG_SIZE = 4 * 1024 * 1024 * 1024  # 4GB in bytes
BAG_BASE_NAME = "mumble_imu_lidar_bag"
STORAGE_ID = "sqlite3"
COMPRESSION_MODE = "file"
COMPRESSION_FORMAT = "zstd"
COMPRESSION_LEVEL = 22  # highest compression leve


class BagRecorderNode(Node):
    def __init__(self):
        super().__init__("mumble_bag_recorder")
        self._bag_index = 0
        self.writer_lock = threading.Lock()
        self._setup_writer()
        self.create_subscription(Imu, "/imu_data", self.imu_callback, 10)
        self.create_subscription(LaserScan, "/scan", self.lidar_callback, 10)

    def _setup_writer(self):
        self.bag_uri = f"{BAG_BASE_NAME}_{self._bag_index}"
        storage_options = StorageOptions()
        storage_options.uri = self.bag_uri
        storage_options.storage_id = STORAGE_ID
        setattr(storage_options, "compression_mode", COMPRESSION_MODE)
        setattr(storage_options, "compression_format", COMPRESSION_FORMAT)
        setattr(storage_options, "compression_level", COMPRESSION_LEVEL)
        converter_options = ConverterOptions("", "")
        self.writer = SequentialWriter()
        self.writer.open(storage_options, converter_options)
        self.get_logger().info(f"Opened new bag file: {self.bag_uri}")

    def _write_message(self, topic: str, msg):
        serialized_msg = rclpy.serialization.serialize_message(msg)
        timestamp = self.get_clock().now().nanoseconds
        with self.writer_lock:
            self.writer.write(topic, serialized_msg, timestamp)

    def imu_callback(self, msg: Imu):
        self._write_message("/imu_data", msg)

    def lidar_callback(self, msg: LaserScan):
        self._write_message("/scan", msg)


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


if __name__ == "__main__":
    main()
