# !/usr/bin/env python3
"""
Run this node: ros2_sudo run mumble_physical_runtime serial_interface
ALL topic subscribers need sudo as well
This file includes: IMU and Motor interfaces
"""

import threading
import time
from functools import partial

import numpy as np
import rclpy
from adafruit_rplidar import RPLidar

# from mumble_interfaces.srv import MotorCommand
from geometry_msgs.msg import Twist
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, LaserScan

from mumble_interfaces.mumble_logging import get_logger
from mumble_physical_runtime.rpi_lidar_a1_mumble import (
    TOTAL_NUM_ANGLES,
    find_lidar_usb_device,
)
from mumble_physical_runtime.waveshare_control import BaseController, test_imu

READ_PERIOD = 1.0 / 30.0  # NO MORE THAN THIS
imu_msg = Imu()
MOTOR_COMMAND_END_TIME = time.perf_counter()
MOTOR_STOP_CHECK_PERIOD = 1.0 / 30.0  # NO MORE THAN THIS
MOTOR_COMMAND_VALID_TIME = 15 * MOTOR_STOP_CHECK_PERIOD
TEN_SECONDS = 10.0
WHEEL_BASE = 0.13

logger = get_logger("serial_interface")

OMEGA_SCALAR = 35.0  # Open Loop Control Constant


def cmd_vel_cb(base, twist: Twist):
    v = twist.linear.x
    omega = twist.angular.z * OMEGA_SCALAR
    left_speed = v - WHEEL_BASE * omega / 2.0
    right_speed = v + WHEEL_BASE * omega / 2.0
    # TODO Remember to remove
    print(f"Rico: TODO: {left_speed, right_speed}")
    global MOTOR_COMMAND_END_TIME
    MOTOR_COMMAND_END_TIME = time.perf_counter() + MOTOR_COMMAND_VALID_TIME
    base.base_speed_ctrl(left_speed, right_speed)


def motor_stop_check_cb(base):
    global MOTOR_COMMAND_END_TIME
    if MOTOR_COMMAND_END_TIME < time.perf_counter():
        logger.info(f"Stop,")
        base.base_speed_ctrl(0.0, 0.0)
        MOTOR_COMMAND_END_TIME = time.perf_counter() + TEN_SECONDS


def publish_imu_data(base, imu_pub, node):
    data = base.raw_imu_info()
    try:
        imu_msg.header.stamp = node.get_clock().now().to_msg()
        imu_msg.header.frame_id = "imu_link"
        # Assign angular velocity values
        imu_msg.angular_velocity.x = float(data["gx"])
        imu_msg.angular_velocity.y = float(data["gy"])
        imu_msg.angular_velocity.z = float(data["gz"])
        # Assign linear acceleration values
        imu_msg.linear_acceleration.x = float(data["ax"])
        imu_msg.linear_acceleration.y = float(data["ay"])
        imu_msg.linear_acceleration.z = float(data["az"])
        imu_pub.publish(imu_msg)
    except Exception as e:
        logger.warn(f"IMU exception occured: {e}")


def lidar_thread(lidar, scan_pub, node):
    while rclpy.ok():
        scan_data = np.zeros(TOTAL_NUM_ANGLES)
        try:
            for scan in lidar.iter_scans():
                for _, angle, distance in scan:
                    scan_data[min(359, int(angle))] = distance / 1000.0
                scan_msg = LaserScan()
                scan_msg.header.stamp = node.get_clock().now().to_msg()
                scan_msg.header.frame_id = "laser_frame"
                scan_msg.angle_min = 0.0
                scan_msg.angle_max = 2 * np.pi
                scan_msg.angle_increment = 2 * np.pi / TOTAL_NUM_ANGLES
                scan_msg.range_min = 0.1
                scan_msg.range_max = 12.0
                scan_msg.ranges = scan_data.tolist()
                scan_pub.publish(scan_msg)
        except Exception as e:
            logger.warn(f"LiDAR Exception: {e}")
    lidar.stop()
    lidar.disconnect()
    logger.info(f"Lidar stopped")


def main(args=None):
    rclpy.init(args=args)
    node = Node("serial_interface")
    base = BaseController("/dev/ttyS0", 115200, mock=False)
    qos_profile = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=10)
    imu_pub = node.create_publisher(Imu, "imu_data", qos_profile)
    callback_group = ReentrantCallbackGroup()
    logger.info(f"Hello! I am the serial_interface")
    timer = node.create_timer(
        READ_PERIOD, partial(publish_imu_data, base, imu_pub, node)
    )
    node.create_subscription(
        Twist, "/cmd_vel", partial(cmd_vel_cb, base), 10, callback_group=callback_group
    )
    node.create_timer(MOTOR_STOP_CHECK_PERIOD, partial(motor_stop_check_cb, base))

    device_address = find_lidar_usb_device()
    # If USB is not found, we will see an exception here.
    lidar = RPLidar(None, device_address)
    scan_pub = node.create_publisher(LaserScan, "scan", qos_profile)
    lidar_thread_instance = threading.Thread(
        target=lidar_thread, args=(lidar, scan_pub, node)
    )
    lidar_thread_instance.start()

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()
    lidar_thread_instance.join()
