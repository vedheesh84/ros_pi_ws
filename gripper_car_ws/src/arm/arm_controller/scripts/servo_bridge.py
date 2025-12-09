#!/usr/bin/env python3
"""
Servo Bridge for Robotic Arm
============================
ROS2 node for servo-based robotic arm control via Arduino/PCA9685.

Topics:
  Subscribe:
    - /arm/joint_commands (sensor_msgs/JointState): Joint position commands
    - /arduino_cmd (std_msgs/String): Direct servo commands

  Publish:
    - /joint_states (sensor_msgs/JointState): Current joint positions
    - /arduino_ack (std_msgs/String): Command acknowledgments
    - /arm/status (std_msgs/String): Status messages

Parameters:
  - serial_port: Arduino serial port (default: /dev/ttyACM0)
  - baud_rate: Serial baud rate (default: 115200)
  - publish_rate: Joint state publish rate in Hz (default: 50.0)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import serial
import threading
import math
import time
import numpy as np


class ServoBridge(Node):
    """Servo bridge for robotic arm control"""

    def __init__(self):
        super().__init__('servo_bridge')

        # Declare parameters
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('wait_for_ack', True)  # wait for Arduino acknowledgment
        self.declare_parameter('ack_timeout', 2.0)  # timeout waiting for ack

        # Get parameters
        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baud_rate').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.wait_for_ack = self.get_parameter('wait_for_ack').value
        self.ack_timeout = self.get_parameter('ack_timeout').value

        # Acknowledgment state
        self.pending_ack = False
        self.ack_received = threading.Event()

        # Joint configuration (6 actuated joints + 1 mimic joint)
        self.joint_names = [
            'joint_1', 'joint_2', 'joint_3',
            'joint_4', 'gripper_base_joint', 'left_gear_joint'
        ]
        self.joint_positions = [0.0] * 6

        # Joint limits from URDF (in radians) - MUST match arm.urdf.xacro
        self.joint_limits = {
            'joint_1': (-3.1416, 3.1416),             # Base rotation: -pi to pi
            'joint_2': (-1.5708, 1.5708),             # Shoulder: -pi/2 to pi/2
            'joint_3': (-1.5708, 1.5708),             # Elbow: -pi/2 to pi/2
            'joint_4': (-1.5708, 1.5708),             # Wrist: -pi/2 to pi/2
            'gripper_base_joint': (-3.1416, 3.1416),  # Gripper rotation: -pi to pi
            'left_gear_joint': (0.0, 1.0),            # Gripper open/close: 0 to 1.0 rad
        }

        # Mimic joints for gripper - all joints that mimic left_gear_joint
        # These need to be published for MoveIt to work correctly
        self.mimic_joints = {
            'right_gear_joint': -1.0,      # multiplier = -1
            'left_finger_joint': 1.0,      # multiplier = 1
            'right_finger_joint': 1.0,     # multiplier = 1
            'left_joint': -1.0,            # multiplier = -1
            'right_joint': 1.0,            # multiplier = 1
        }
        self.mimic_positions = {name: 0.0 for name in self.mimic_joints}

        # QoS profile
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Publishers
        self.pub_joint_state = self.create_publisher(JointState, '/joint_states', qos)
        self.pub_ack = self.create_publisher(String, '/arduino_ack', 10)
        self.pub_status = self.create_publisher(String, '/arm/status', 10)

        # Subscribers
        self.sub_joint_cmd = self.create_subscription(
            JointState, '/arm/joint_commands', self.joint_cmd_callback, qos)
        self.sub_arduino_cmd = self.create_subscription(
            String, '/arduino_cmd', self.arduino_cmd_callback, 10)

        # Serial setup
        self.serial = None
        self.serial_lock = threading.Lock()
        self.connected = False

        try:
            self.serial = serial.Serial(port, baud, timeout=1)
            time.sleep(2)
            self.connected = True
            self.get_logger().info(f'Connected to Arduino at {port}')
        except Exception as e:
            self.get_logger().error(f'Serial open failed: {e}')

        # Serial reader thread
        self.running = True
        self.reader_thread = threading.Thread(target=self.serial_loop, daemon=True)
        self.reader_thread.start()

        # Joint state publish timer
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_joint_states)

        self.get_logger().info('Servo Bridge initialized')

    def joint_cmd_callback(self, msg):
        """Handle joint position commands from trajectory_bridge"""
        # Wait for previous acknowledgment if enabled (ensures Arduino completed last command)
        if self.wait_for_ack and self.pending_ack:
            self.get_logger().debug('Waiting for previous command acknowledgment...')
            if not self.ack_received.wait(timeout=self.ack_timeout):
                self.get_logger().warn('Ack timeout, proceeding anyway')
            self.ack_received.clear()
            self.pending_ack = False

        # Start with last known servo angles for all 6 joints
        # This ensures we always send SET_ALL_SERVOS even if MoveIt only sends 5 joints
        servo_angles = [self.joint_to_servo_angle(i, self.joint_positions[i]) for i in range(6)]
        joints_updated = 0

        for i, joint_name in enumerate(msg.name):
            if joint_name in self.joint_names:
                idx = self.joint_names.index(joint_name)
                position_rad = msg.position[i]
                angle_deg = self.joint_to_servo_angle(idx, position_rad)
                servo_angles[idx] = angle_deg
                # Update internal position tracking
                self.joint_positions[idx] = position_rad
                joints_updated += 1

        if joints_updated > 0:
            # Always use SET_ALL_SERVOS - fill missing joints with last known positions
            angles_str = ",".join(str(a) for a in servo_angles)
            command = f"SET_ALL_SERVOS,{angles_str}"
            self.send_command(command)
            self.get_logger().info(f'Sent SET_ALL_SERVOS: {angles_str}')

            if self.wait_for_ack:
                self.pending_ack = True

    def arduino_cmd_callback(self, msg):
        """Send raw commands to Arduino"""
        self.send_command(msg.data.strip())

    def send_command(self, command):
        """Send command to Arduino"""
        if not self.connected or not self.serial:
            return False

        try:
            with self.serial_lock:
                self.serial.write((command + '\n').encode())
                self.serial.flush()
            return True
        except Exception as e:
            self.get_logger().warn(f'Serial write error: {e}')
            return False

    def serial_loop(self):
        """Background serial reader"""
        while self.running and self.serial:
            try:
                with self.serial_lock:
                    if self.serial.in_waiting > 0:
                        line = self.serial.readline().decode().strip()
                        if line:
                            self.process_message(line)
            except Exception as e:
                self.get_logger().error(f'Serial error: {e}')
            time.sleep(0.01)

    def process_message(self, message):
        """Process Arduino messages"""
        # SERVO_POS,<6 values> - joint position feedback
        if message.startswith('SERVO_POS,'):
            parts = message.split(',')
            if len(parts) == 7:
                servo_angles = [float(x) for x in parts[1:7]]
                self.update_joint_positions(servo_angles)

        # OK,SERVO,... or OK,SET_ALL_SERVOS - command acknowledgment
        elif message.startswith('OK,') or message.startswith('ERROR,'):
            # Signal acknowledgment received
            self.ack_received.set()
            ack = String()
            ack.data = message
            self.pub_ack.publish(ack)
            self.get_logger().debug(f'Received ack: {message}')

        # HEARTBEAT <millis> - Arduino heartbeat
        elif message.startswith('HEARTBEAT'):
            self.get_logger().debug(f'Arduino heartbeat: {message}')

        # ARM READY - Arduino startup message
        elif message.startswith('ARM READY'):
            self.get_logger().info('Arduino ARM READY received')
            status = String()
            status.data = 'ARM_READY'
            self.pub_status.publish(status)

        elif message.startswith('STATUS'):
            status = String()
            status.data = message
            self.pub_status.publish(status)

    def update_joint_positions(self, servo_angles):
        """Convert servo angles to joint positions"""
        for i, deg in enumerate(servo_angles):
            self.joint_positions[i] = self.servo_to_joint_angle(i, deg)

    def joint_to_servo_angle(self, joint_idx, position_rad):
        """
        Convert joint position (rad) to servo angle (0-180 deg).

        Mapping strategy:
        - Servo 90° = joint 0 rad (center position)
        - Servo range is [0, 180] mapped to joint's full range
        """
        joint_name = self.joint_names[joint_idx]
        lower, upper = self.joint_limits[joint_name]

        # Clamp position to joint limits
        position_rad = np.clip(position_rad, lower, upper)

        if joint_idx == 5:  # Gripper gear: [0, pi/4] → [90, 135] (center to open)
            # Servo 90° = closed (0 rad), Servo 135° = open (pi/4 rad)
            angle = 90.0 + (position_rad * 180.0 / math.pi)
        else:
            # General mapping: joint range → [0, 180]
            # Center of joint range → servo 90°
            joint_range = upper - lower
            center = (upper + lower) / 2.0
            # Map: center → 90°, lower → 0°, upper → 180°
            angle = 90.0 + ((position_rad - center) / (joint_range / 2.0)) * 90.0

        return int(np.clip(angle, 0, 180))

    def servo_to_joint_angle(self, joint_idx, angle_deg):
        """
        Convert servo angle (0-180 deg) to joint position (rad).

        Mapping strategy:
        - Servo 90° = joint 0 rad (center position) for most joints
        - For gripper, servo 90° = closed, servo 135° = open
        """
        joint_name = self.joint_names[joint_idx]
        lower, upper = self.joint_limits[joint_name]

        if joint_idx == 5:  # Gripper gear: [90, 135] → [0, pi/4]
            # Servo 90° = closed (0 rad), Servo 135° = open (pi/4 rad)
            position_rad = (angle_deg - 90.0) * (math.pi / 180.0)
        else:
            # General mapping: [0, 180] → joint range
            # Servo 90° → center of joint range
            joint_range = upper - lower
            center = (upper + lower) / 2.0
            # Map: 90° → center, 0° → lower, 180° → upper
            position_rad = center + ((angle_deg - 90.0) / 90.0) * (joint_range / 2.0)

        # Clamp to joint limits
        position_rad = np.clip(position_rad, lower, upper)

        return position_rad

    def publish_joint_states(self):
        """Publish current joint states including all mimic joints for gripper."""
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()

        # Get left_gear_joint position to compute all mimic joint positions
        left_gear_pos = self.joint_positions[5]  # left_gear_joint

        # Calculate all mimic joint positions
        for name, multiplier in self.mimic_joints.items():
            self.mimic_positions[name] = left_gear_pos * multiplier

        # Combine actuated joints + all mimic joints
        all_names = self.joint_names + list(self.mimic_joints.keys())
        all_positions = self.joint_positions + list(self.mimic_positions.values())

        joint_state.name = all_names
        joint_state.position = all_positions
        joint_state.velocity = [0.0] * len(all_names)
        joint_state.effort = [0.0] * len(all_names)

        self.pub_joint_state.publish(joint_state)

    def destroy_node(self):
        """Cleanup"""
        self.running = False
        if self.serial:
            self.serial.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ServoBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
