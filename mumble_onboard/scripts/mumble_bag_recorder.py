#!/usr/bin/env python3

"""
ROS2 Bag Recorder for IMU and LiDAR messages. 
Features: 
- Can remove stale files when the total size hits xGB
- Uses file-level zstd compression with a high compression level.
"""

import os
import threading
import time

import ament_index_python.packages
import rclpy
import rclpy.serialization
from rclpy.node import Node

# Import rosbag2_py interfaces
from rosbag2_py import ConverterOptions, SequentialWriter, StorageOptions
from rosbag2_py._storage import TopicMetadata
from sensor_msgs.msg import Imu, LaserScan

from mumble_interfaces.mumble_logging import get_logger

logger = get_logger(f"mumble_bag_recorder")
package_name = "mumble_onboard"
package_path = ament_index_python.packages.get_package_share_directory(package_name)
bags_dir = os.path.join(package_path, f"bags_{time.time()}")
os.makedirs(bags_dir, exist_ok=True)
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
        self.writer_lock = threading.Lock()
        self._setup_writer()
        self.create_subscription(Imu, "/imu_data", self.imu_callback, 10)
        self.create_subscription(LaserScan, "/scan", self.lidar_callback, 10)

    def _setup_writer(self):
        self.bag_uri = os.path.join(bags_dir, f"{BAG_BASE_NAME}")
        storage_options = StorageOptions(uri=self.bag_uri, storage_id=STORAGE_ID)
        converter_options = ConverterOptions("", "")
        self.writer = SequentialWriter()
        self.writer.open(storage_options, converter_options)

        metadata_ls = [
            TopicMetadata(
                name="/imu_data",
                type="sensor_msgs/msg/Imu",
                serialization_format="cdr",  # Default serialization format in ROS 2
            ),
            TopicMetadata(
                name="/scan",
                type="sensor_msgs/msg/LaserScan",
                serialization_format="cdr",
            ),
        ]
        for m in metadata_ls:
            self.writer.create_topic(m)
        logger.info(f"Opened new bag directory: {self.bag_uri}")

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
