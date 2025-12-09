#!/usr/bin/env python3
"""
Teleop Keyboard Node
=====================

Simple keyboard control for mobile base.

Controls:
    w/s: Forward/Backward
    a/d: Turn left/right
    q/e: Forward-left/Forward-right
    z/c: Backward-left/Backward-right
    x or SPACE: Stop

    +/-: Increase/decrease linear speed
    [/]: Increase/decrease angular speed

    ESC or Ctrl+C: Quit

Parameters:
    linear_speed: Initial linear speed in m/s (default: 0.3)
    angular_speed: Initial angular speed in rad/s (default: 0.5)
    linear_step: Speed increment for linear velocity (default: 0.05)
    angular_step: Speed increment for angular velocity (default: 0.1)

Usage:
    ros2 run base_controller teleop_keyboard.py
    ros2 run base_controller teleop_keyboard.py --ros-args -p linear_speed:=0.5
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

import sys
import termios
import tty
import select


# Key bindings
MOVE_BINDINGS = {
    'w': (1.0, 0.0),    # Forward
    's': (-1.0, 0.0),   # Backward
    'a': (0.0, 1.0),    # Turn left
    'd': (0.0, -1.0),   # Turn right
    'q': (1.0, 1.0),    # Forward-left
    'e': (1.0, -1.0),   # Forward-right
    'z': (-1.0, 1.0),   # Backward-left
    'c': (-1.0, -1.0),  # Backward-right
}

STOP_KEYS = ['x', ' ']
QUIT_KEYS = ['\x1b', '\x03']  # ESC, Ctrl+C


INSTRUCTIONS = """
==================================================
      MOBILE BASE TELEOP KEYBOARD
==================================================

Moving around:
        w
   q    s    e
        a    d
   z    x    c

w/s : forward/backward
a/d : turn left/right
q/e : forward + turn
z/c : backward + turn
x/SPACE : stop

Speed control:
+/- : increase/decrease linear speed
[/] : increase/decrease angular speed

ESC or Ctrl+C to quit
==================================================
"""


class TeleopKeyboard(Node):
    def __init__(self):
        super().__init__('teleop_keyboard')

        # Declare parameters
        self.declare_parameter('linear_speed', 0.3)
        self.declare_parameter('angular_speed', 0.5)
        self.declare_parameter('linear_step', 0.05)
        self.declare_parameter('angular_step', 0.1)
        self.declare_parameter('max_linear_speed', 1.0)
        self.declare_parameter('max_angular_speed', 2.0)

        # Get parameters
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.linear_step = self.get_parameter('linear_step').value
        self.angular_step = self.get_parameter('angular_step').value
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value

        # Publisher
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Current velocity
        self.current_linear = 0.0
        self.current_angular = 0.0

        # Terminal settings
        self.settings = termios.tcgetattr(sys.stdin)

    def get_key(self, timeout=0.1):
        """Get a single keypress."""
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def print_status(self):
        """Print current speed settings."""
        print(f'\rSpeed: linear={self.linear_speed:.2f} m/s, angular={self.angular_speed:.2f} rad/s    ', end='')
        sys.stdout.flush()

    def publish_velocity(self, linear_x, angular_z):
        """Publish velocity command."""
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self.cmd_vel_pub.publish(twist)
        self.current_linear = linear_x
        self.current_angular = angular_z

    def run(self):
        """Main loop."""
        print(INSTRUCTIONS)
        self.print_status()

        try:
            while rclpy.ok():
                key = self.get_key()

                if key in MOVE_BINDINGS:
                    x, th = MOVE_BINDINGS[key]
                    linear_x = x * self.linear_speed
                    angular_z = th * self.angular_speed
                    self.publish_velocity(linear_x, angular_z)

                elif key in STOP_KEYS:
                    self.publish_velocity(0.0, 0.0)
                    print('\r*** STOP ***                                        ')
                    self.print_status()

                elif key == '+' or key == '=':
                    self.linear_speed = min(
                        self.linear_speed + self.linear_step,
                        self.max_linear_speed
                    )
                    print(f'\rLinear speed: {self.linear_speed:.2f} m/s                    ')
                    self.print_status()

                elif key == '-':
                    self.linear_speed = max(
                        self.linear_speed - self.linear_step,
                        0.0
                    )
                    print(f'\rLinear speed: {self.linear_speed:.2f} m/s                    ')
                    self.print_status()

                elif key == ']':
                    self.angular_speed = min(
                        self.angular_speed + self.angular_step,
                        self.max_angular_speed
                    )
                    print(f'\rAngular speed: {self.angular_speed:.2f} rad/s                  ')
                    self.print_status()

                elif key == '[':
                    self.angular_speed = max(
                        self.angular_speed - self.angular_step,
                        0.0
                    )
                    print(f'\rAngular speed: {self.angular_speed:.2f} rad/s                  ')
                    self.print_status()

                elif key in QUIT_KEYS:
                    break

                # No key pressed - if we were moving, keep the command alive
                # by not sending anything (bridge handles timeout)

        except Exception as e:
            self.get_logger().error(f'Error: {e}')

        finally:
            # Send stop command on exit
            self.publish_velocity(0.0, 0.0)
            print('\n\nExiting teleop...')

    def destroy_node(self):
        """Cleanup."""
        # Restore terminal settings
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TeleopKeyboard()

    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
