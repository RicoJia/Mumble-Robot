#!/usr/bin/env python3
# Run this node: ros2_sudo run mumble_physical_runtime imu_broadcaster

import rclpy
from rclpy.node import Node
from mumble_physical_runtime.waveshare_control import BaseController, test_imu
from mumble_interfaces.srv import MotorCommand
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Imu
from functools import partial

READ_PERIOD = 1.0 / 40  # 40Hz
imu_msg = Imu()

def publish_imu_data(base, imu_pub, node):
    data = base.raw_imu_info()
    # TODO Remember to remove
    imu_msg.header.stamp = node.get_clock().now().to_msg()
    imu_msg.header.frame_id = "imu_link"
    # Assign angular velocity values
    imu_msg.angular_velocity.x = data["gx"]
    imu_msg.angular_velocity.y = data["gy"]
    imu_msg.angular_velocity.z = data["gz"]
    # Assign linear acceleration values
    imu_msg.linear_acceleration.x = data["ax"]
    imu_msg.linear_acceleration.y = data["ay"]
    imu_msg.linear_acceleration.z = data["az"]
    imu_pub.publish(imu_msg)
    # TODO Remember to remove
    print(f"Rico: {imu_msg}")


def main(args=None):
    rclpy.init(args=args)
    # namespace?
    node = Node("imu_broadcaster")
    # base = BaseController("/dev/ttyAMA0", 115200, mock=False)
    imu_pub = node.create_publisher(Imu, "imu_data", 10)
    print(f"Rico: Hello! I am the IMU broadcaster")
    # timer = node.create_timer(
    #     READ_PERIOD, partial(publish_imu_data, base, imu_pub, node)
    # )

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()
