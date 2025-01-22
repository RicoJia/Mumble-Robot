#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

def main(args=None):
    rclpy.init(args=args)
    # namespace?
    node = Node("imu_broadcaster")
    #TODO Remember to remove
    print(f'Rico: Hello! I am the IMU broadcaster')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()