#!/usr/bin/env python3
"""
Integrated Teleop Control
==========================
Keyboard control for both mobile base and robotic arm.

Controls:
  Mobile Base:
    w/s: Forward/Backward
    a/d: Turn left/right
    space: Stop

  Arm Joints:
    1-6: Select joint
    i/k: Increase/decrease joint angle
    h: Home position
    g: Open/close gripper
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
import sys
import termios
import tty
import select


class IntegratedTeleop(Node):
    def __init__(self):
        super().__init__('integrated_teleop')

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.joint_cmd_pub = self.create_publisher(JointState, '/arm/joint_commands', 10)

        # State
        self.linear_speed = 0.3
        self.angular_speed = 0.5
        self.joint_step = 0.1
        self.selected_joint = 0
        self.joint_positions = [0.0] * 6
        self.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4',
                            'gripper_base_joint', 'left_gear_joint']

        self.get_logger().info('Integrated Teleop Ready')
        self.print_instructions()

    def print_instructions(self):
        print("\n=== Integrated Teleop Control ===")
        print("Mobile Base: w/s=fwd/back, a/d=turn, space=stop")
        print("Arm: 1-6=select joint, i/k=move, h=home, g=gripper")
        print(f"Selected joint: {self.selected_joint + 1} ({self.joint_names[self.selected_joint]})")
        print("q=quit\n")

    def get_key(self):
        """Get a single keypress"""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
            if rlist:
                key = sys.stdin.read(1)
            else:
                key = ''
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return key

    def run(self):
        """Main loop"""
        try:
            while rclpy.ok():
                key = self.get_key()

                if key == 'q':
                    break

                # Mobile base control
                twist = Twist()
                if key == 'w':
                    twist.linear.x = self.linear_speed
                elif key == 's':
                    twist.linear.x = -self.linear_speed
                elif key == 'a':
                    twist.angular.z = self.angular_speed
                elif key == 'd':
                    twist.angular.z = -self.angular_speed
                elif key == ' ':
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0

                if key in ['w', 's', 'a', 'd', ' ']:
                    self.cmd_vel_pub.publish(twist)

                # Joint selection
                if key in ['1', '2', '3', '4', '5', '6']:
                    self.selected_joint = int(key) - 1
                    print(f"Selected: {self.joint_names[self.selected_joint]}")

                # Joint control
                if key == 'i':
                    self.joint_positions[self.selected_joint] += self.joint_step
                    self.publish_joint_cmd()
                elif key == 'k':
                    self.joint_positions[self.selected_joint] -= self.joint_step
                    self.publish_joint_cmd()

                # Home position
                if key == 'h':
                    self.joint_positions = [0.0] * 6
                    self.publish_joint_cmd()
                    print("Home position")

                # Gripper toggle
                if key == 'g':
                    if self.joint_positions[5] < 0.4:
                        self.joint_positions[5] = 0.7
                        print("Gripper: OPEN")
                    else:
                        self.joint_positions[5] = 0.0
                        print("Gripper: CLOSED")
                    self.publish_joint_cmd()

        except Exception as e:
            self.get_logger().error(f'Error: {e}')
        finally:
            # Stop robot
            self.cmd_vel_pub.publish(Twist())

    def publish_joint_cmd(self):
        """Publish joint command"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = self.joint_positions
        self.joint_cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = IntegratedTeleop()

    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
