#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from mumble_physical_runtime.waveshare_control import BaseController
from sensor_msgs.msg import Imu
from functools import partial

READ_PERIOD = 1.0 / 40  # 40Hz
imu_msg = Imu()


def publish_imu_data(base, imu_pub, node):
    # resp = base.raw_imu_info()
    imu_msg.header.stamp = node.get_clock().now().to_msg()
    imu_msg.header.frame_id = "imu_link"
    # imu_msg.orientation.x = resp['orientation']['x']
    # imu_msg.orientation.y = resp['orientation']['y']
    # imu_msg.orientation.z = resp['orientation']['z']
    # imu_msg.orientation.w = resp['orientation']['w']
    # imu_msg.angular_velocity.x = resp['angular_velocity']['x']
    # imu_msg.angular_velocity.y = resp['angular_velocity']['y']
    # imu_msg.angular_velocity.z = resp['angular_velocity']['z']
    # imu_msg.linear_acceleration.x = resp['linear_acceleration']['x']
    # imu_msg.linear_acceleration.y = resp['linear_acceleration']['y']
    # imu_msg.linear_acceleration.z = resp['linear_acceleration']['z']
    imu_pub.publish(imu_msg)
    # TODO Remember to remove
    print(f"Rico: {imu_msg}")


def main(args=None):
    rclpy.init(args=args)
    # namespace?
    node = Node("imu_broadcaster")
    base = BaseController("/dev/serial0", 115200, mock=True)
    imu_pub = node.create_publisher(Imu, "imu_data", 10)
    # TODO Remember to remove
    print(f"Rico: Hello! I am the IMU broadcaster")
    timer = node.create_timer(
        READ_PERIOD, partial(publish_imu_data, base, imu_pub, node)
    )
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
