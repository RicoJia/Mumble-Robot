#!/usr/bin/env python3

import sys
import threading

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sshkeyboard import listen_keyboard

# Velocity magnitude limits
MIN_VEL = np.array([0.0, 0.0])  # [linear, angular]
MAX_VEL = np.array([1.0, 3.14])  # [linear, angular]

# Increment steps
LIN_INCREMENT = np.array([0.02, 0.0])
ANG_INCREMENT = np.array([0.0, 0.1])

# Keys that adjust velocity magnitude
CHAR_KEYS = {"w", "s", "a", "d"}
# Keys that set velocity direction signs
ARROW_KEYS = {"up", "down", "left", "right"}


class KeyboardTeleopNode(Node):
    def __init__(self):
        super().__init__("keyboard_teleop")
        # Publisher
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # Current velocity magnitude [lin, ang]
        self.velocity = np.array([0.0, 0.0])

        # Current direction sign [lin_sign, ang_sign]
        #  +1 = forward / left turn, -1 = backward / right turn, 0 = no movement
        self.sign = np.array([0, 0])

        # Track pressed arrow keys
        self.pressed_arrows = set()

        self.get_logger().info("Keyboard Teleop Node started using sshkeyboard.")

        # Start the sshkeyboard listener in a background thread
        self.keyboard_thread = threading.Thread(
            target=self.start_keyboard_listener, daemon=True
        )
        self.keyboard_thread.start()

    def start_keyboard_listener(self):
        """
        Launches sshkeyboard's listen_keyboard in a blocking way.
        We'll run this in a background thread so rclpy.spin() can continue.
        """
        listen_keyboard(
            on_press=self.on_key_press,
            on_release=None,
            delay_second_char=0.03,
            delay_other_chars=0.01,
        )

    def on_key_press(self, key: str):
        """
        Called whenever a key is pressed.
        - w/s => increment/decrement linear velocity magnitude
        - a/d => increment/decrement angular velocity magnitude
        - up/down/left/right => set sign for velocity
        """
        # 1) Update velocity magnitudes for WASD
        if key in ("w", "a", "s", "d"):
            if key == "w":
                self.velocity = np.minimum(self.velocity + LIN_INCREMENT, MAX_VEL)
            elif key == "s":
                self.velocity = np.maximum(self.velocity - LIN_INCREMENT, MIN_VEL)
            elif key == "a":
                self.velocity = np.minimum(self.velocity + ANG_INCREMENT, MAX_VEL)
            elif key == "d":
                self.velocity = np.maximum(self.velocity - ANG_INCREMENT, MIN_VEL)
            self.get_logger().info(f"velocity update: {self.velocity}")

        # 2) Update direction sign for arrow keys
        if key in ARROW_KEYS:
            self.pressed_arrows.add(key)
            self.recalc_sign()
            self.publish_twist()

        # If ESC is pressed, optionally shut down or ignore
        if key == "esc":
            self.get_logger().info("ESC pressed: shutting down.")
            rclpy.shutdown()
        self.pressed_arrows.discard(key)

    def on_key_release(self, key: str):
        """
        Called whenever a key is released.
        - We do NOT change velocity magnitude on release
        - But we remove arrow keys from the set => sign might revert to 0
        """
        pass

    def recalc_sign(self):
        """
        Recompute self.sign = [lin_sign, ang_sign] based on pressed_arrows.
        - Up => linear sign +1, Down => -1, none => 0
        - Left => angular sign +1, Right => -1, none => 0
        """
        lin_sign = 0
        ang_sign = 0

        # If both up and down are pressed, they cancel out => 0
        if "up" in self.pressed_arrows:
            lin_sign = +1
        elif "down" in self.pressed_arrows:
            lin_sign = -1

        # If both left and right are pressed, they cancel out => 0
        if "left" in self.pressed_arrows:
            ang_sign = +1
        elif "right" in self.pressed_arrows:
            ang_sign = -1

        self.sign = np.array([lin_sign, ang_sign], dtype=np.float32)

    def publish_twist(self):
        """
        Publish the final Twist message:
          linear.x = sign[0] * velocity[0]
          angular.z = sign[1] * velocity[1]
        """
        lin_val = float(self.sign[0] * self.velocity[0])
        ang_val = float(self.sign[1] * self.velocity[1])

        msg = Twist()
        msg.linear.x = lin_val
        msg.angular.z = ang_val

        self.cmd_vel_pub.publish(msg)
        self.get_logger().info(
            f"Published Twist: linear.x={msg.linear.x:.2f}, angular.z={msg.angular.z:.2f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Restore terminal settings in case sshkeyboard changed them
        import os

        os.system("stty sane")

        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
