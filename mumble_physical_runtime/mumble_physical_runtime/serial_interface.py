#!/usr/bin/env python3
# Run this node: ros2_sudo run mumble_physical_runtime serial_interface
# ALL topic subscribers need sudo as well
# This file includes: IMU and Motor interfaces

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from mumble_physical_runtime.waveshare_control import BaseController, test_imu
from mumble_interfaces.srv import MotorCommand
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import Imu
from functools import partial
import time

READ_PERIOD = 1.0 / 30.0  # NO MORE THAN THIS
imu_msg = Imu()
MOTOR_COMMAND_END_TIME = time.perf_counter()
MOTOR_STOP_CHECK_PERIOD = 1.0/30.0  #NO MORE THAN THIS
TEN_SECONDS = 10.0

def motor_command(base, request, response):
    global MOTOR_COMMAND_END_TIME
    MOTOR_COMMAND_END_TIME = time.perf_counter() + request.duration_s
    base.base_speed_ctrl(request.left_speed, request.right_speed)
    return response
    

def motor_stop_check_cb(base):
    global MOTOR_COMMAND_END_TIME
    if MOTOR_COMMAND_END_TIME < time.perf_counter():
        print(f'Rico: issuing stop cmd,  {MOTOR_COMMAND_END_TIME, time.perf_counter()}')
        base.base_speed_ctrl(0.0, 0.0)
        MOTOR_COMMAND_END_TIME = time.perf_counter() + TEN_SECONDS
        

def publish_imu_data(base, imu_pub, node):
    data = base.raw_imu_info()
    try:
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
    except Exception as e:
        print(f'Exception occured: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = Node("serial_interface")
    base = BaseController("/dev/ttyS0", 115200, mock=False)
    qos_profile = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=10)
    imu_pub = node.create_publisher(Imu, "imu_data", qos_profile)
    callback_group = ReentrantCallbackGroup()
    print(f"Hello! I am the serial_interface")
    timer = node.create_timer(
        READ_PERIOD, partial(publish_imu_data, base, imu_pub, node)
    )

    node.create_service(MotorCommand, "motor_command", 
                        partial(motor_command, base), callback_group=callback_group)
    node.create_timer(MOTOR_STOP_CHECK_PERIOD, partial(motor_stop_check_cb, base))

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()