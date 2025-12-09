#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray, Int64MultiArray

import sys
import tty
import termios
import os
import io


class TeleopKeyboard(Node):
    def __init__(self):
        super().__init__('teleop_keyboard')

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.motor_cmd_pub = self.create_publisher(Int32MultiArray, 'motor_cmd', 10)

        # Subscribers
        self.encoder_sub = self.create_subscription(
            Int64MultiArray, 'encoder_ticks', self.encoder_callback, 10)

        # Velocity control values
        self.linear_speed = 0.0
        self.angular_speed = 0.0
        self.speed_step = 0.1
        self.turn_step = 0.5
        self.max_linear = 1.0
        self.max_angular = 2.0

        # PWM control values
        self.control_mode = "vel"
        self.pwm_step = 50
        self.left_pwm = 0
        self.right_pwm = 0

        self.last_encoder = [0, 0]

        self._input_stream = None
        self._input_fd = None
        self._setup_input_stream()

        self.print_instructions()
        self.get_logger().info("Teleop Keyboard Started")

    # ----------------------------------------------------------------------
    # PRINT INSTRUCTIONS
    # ----------------------------------------------------------------------
    def print_instructions(self):
        print("\n" + "=" * 60)
        print("        ROS2 ARDUINO BRIDGE - KEYBOARD TELEOP")
        print("=" * 60)
        print("\nVelocity Mode (cmd_vel):")
        print("   W/S : Increase / decrease linear speed")
        print("   A/D : Increase / decrease angular speed")
        print("   X or SPACE : Stop")
        print("\nPWM Mode (motor_cmd):")
        print("   I/K : Left motor + / -")
        print("   O/L : Right motor + / -")
        print("   M : Stop PWM motors")
        print("\nGeneral:")
        print("   T : Toggle modes")
        print("   R : Reset all")
        print("   Q : Quit")
        print("=" * 60)
        print(f"\nCurrent mode: {self.control_mode.upper()}\n")

    # ----------------------------------------------------------------------
    def encoder_callback(self, msg):
        self.last_encoder = msg.data

        if self.control_mode == "vel":
            print(f"\rEnc L={msg.data[0]:6d}  R={msg.data[1]:6d}  "
                  f"Lin={self.linear_speed:+.2f}  Ang={self.angular_speed:+.2f}   ",
                  end='', flush=True)
        else:
            print(f"\rEnc L={msg.data[0]:6d}  R={msg.data[1]:6d}  "
                  f"PWM L={self.left_pwm:+4d} R={self.right_pwm:+4d}   ",
                  end='', flush=True)

    # ----------------------------------------------------------------------
    def get_key(self):
        if self._input_stream is None:
            return None

        fd = self._input_fd
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            ch = self._input_stream.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
        return ch.lower()

    # ----------------------------------------------------------------------
    def run(self):
        try:
            while rclpy.ok():
                key = self.get_key()
                if key is None:
                    self.get_logger().error("No TTY available for keyboard.")
                    break

                if self.control_mode == "vel":
                    self.handle_velocity_mode(key)
                else:
                    self.handle_pwm_mode(key)

                # Common keys
                if key == 't':
                    self.control_mode = "pwm" if self.control_mode == "vel" else "vel"
                    print(f"\nSwitched to {self.control_mode.upper()} mode\n")

                elif key == 'q':
                    print("\nQuitting teleop...\n")
                    break

                rclpy.spin_once(self, timeout_sec=0.01)

        except KeyboardInterrupt:
            pass

    # ----------------------------------------------------------------------
    def handle_velocity_mode(self, key):
        if key == 'w':
            self.linear_speed = min(self.linear_speed + self.speed_step, self.max_linear)
        elif key == 's':
            self.linear_speed = max(self.linear_speed - self.speed_step, -self.max_linear)
        elif key == 'a':
            self.angular_speed = min(self.angular_speed + self.turn_step, self.max_angular)
        elif key == 'd':
            self.angular_speed = max(self.angular_speed - self.turn_step, -self.max_angular)
        elif key in ['x', ' ']:
            self.linear_speed = 0.0
            self.angular_speed = 0.0
        elif key == 'r':
            self.linear_speed = 0.0
            self.angular_speed = 0.0

        if key in ['w', 's', 'a', 'd', 'x', ' ', 'r']:
            self.publish_cmd_vel()

    # ----------------------------------------------------------------------
    def handle_pwm_mode(self, key):
        if key == 'i':
            self.left_pwm = min(self.left_pwm + self.pwm_step, 255)
        elif key == 'k':
            self.left_pwm = max(self.left_pwm - self.pwm_step, -255)
        elif key == 'o':
            self.right_pwm = min(self.right_pwm + self.pwm_step, 255)
        elif key == 'l':
            self.right_pwm = max(self.right_pwm - self.pwm_step, -255)
        elif key in ['m', ' ']:
            self.left_pwm = 0
            self.right_pwm = 0
        elif key == 'r':
            self.left_pwm = 0
            self.right_pwm = 0

        if key in ['i', 'k', 'o', 'l', 'm', ' ', 'r']:
            self.publish_motor_cmd()

    # ----------------------------------------------------------------------
    def publish_cmd_vel(self):
        msg = Twist()
        msg.linear.x = self.linear_speed
        msg.angular.z = self.angular_speed
        self.cmd_vel_pub.publish(msg)

    def publish_motor_cmd(self):
        msg = Int32MultiArray()
        msg.data = [self.left_pwm, self.right_pwm]
        self.motor_cmd_pub.publish(msg)

    # ----------------------------------------------------------------------
    def _setup_input_stream(self):
        """Works on laptop and Pi (SSH/terminal)."""
        try:
            fd = sys.stdin.fileno()
            if os.isatty(fd):
                self._input_stream = sys.stdin
                self._input_fd = fd
                return
        except Exception:
            pass

        try:
            tty_stream = open('/dev/tty')
            fd = tty_stream.fileno()
            if os.isatty(fd):
                self.get_logger().info("Using /dev/tty for keyboard input")
                self._input_stream = tty_stream
                self._input_fd = fd
                return
        except Exception:
            pass

        self._input_stream = None
        self._input_fd = None


# ----------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = TeleopKeyboard()

    try:
        node.run()
    finally:
        # Send stop
        stop = Twist()
        node.cmd_vel_pub.publish(stop)

        if node._input_stream not in (None, sys.stdin):
            node._input_stream.close()

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
