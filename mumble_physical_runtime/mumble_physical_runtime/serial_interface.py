#!/usr/bin/env python3
# Run this node: ros2_sudo run mumble_physical_runtime serial_interface
# This file includes: IMU and Motor interfaces

import rclpy
from rclpy.node import Node
from mumble_physical_runtime.waveshare_control import BaseController, test_imu
from mumble_interfaces.srv import MotorCommand
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import Imu
from functools import partial
import time

READ_PERIOD = 1.0 / 40.0  # 40Hz
imu_msg = Imu()
MOTOR_COMMAND_END_TIME = time.perf_counter()
MOTOR_STOP_CHECK_PERIOD = 1.0/60.0  #0.1s
TEN_SECONDS = 10.0

def motor_command(request, response):
    print(request)
    global MOTOR_COMMAND_END_TIME
    MOTOR_COMMAND_END_TIME = time.perf_counter() + request.duration_s
    return response
    

def motor_stop_check_cb():
    global MOTOR_COMMAND_END_TIME
    if MOTOR_COMMAND_END_TIME < time.perf_counter():
        #TODO Remember to remove
        print(f'Rico: issuing stop cmd,  {MOTOR_COMMAND_END_TIME, time.perf_counter()}')
        MOTOR_COMMAND_END_TIME = time.perf_counter() + TEN_SECONDS
        

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
    node = Node("serial_interface")
    # base = BaseController("/dev/ttyAMA0", 115200, mock=False)
    imu_pub = node.create_publisher(Imu, "imu_data", 10)
    callback_group = ReentrantCallbackGroup()
    print(f"Hello! I am the serial_interface")
    # timer = node.create_timer(
    #     READ_PERIOD, partial(publish_imu_data, base, imu_pub, node)
    # )

    node.create_service(MotorCommand, "motor_command", motor_command, callback_group=callback_group)
    node.create_timer(MOTOR_STOP_CHECK_PERIOD, motor_stop_check_cb)

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()
