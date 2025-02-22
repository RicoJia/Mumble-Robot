import threading
import time

import rclpy
from rclpy.node import Node

from mumble_interfaces.srv import MotorCommand

RATE = 40
EXECUTION_DURATION = (
    8.0 / RATE
)  # makes the exeuction rate slightly longer to account for any small timing delays. 0.2s


# the service can respond to Commands ~45hz
def call_motor_service_periodically(node, client, rate_hz=1.0):
    """Calls the motor_command service at a fixed rate."""
    rate = node.create_rate(rate_hz)  # Set rate (Hz)

    i = 0.0
    while rclpy.ok():
        i += 1.0
        request = MotorCommand.Request()
        request.left_speed = 0.5
        request.right_speed = 0.5
        request.duration_s = EXECUTION_DURATION

        future = client.call_async(request)
        node.get_logger().info("Sent motor command request.")
        # If in a class, we can pass make the class a child class of node, and pass it in.
        print(future.result())
        rate.sleep()


def main():
    rclpy.init()
    node = Node("motor_command_caller")
    client = node.create_client(MotorCommand, "motor_command")
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info("Waiting for service...")

    try:
        call_motor_service_periodically(node, client, rate_hz=RATE)  # Call at 2 Hz
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
