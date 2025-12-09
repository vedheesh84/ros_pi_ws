#!/usr/bin/env python3
"""
Base Hardware Bridge
====================
ROS2 node for 4-wheel differential drive control via Arduino.

Features:
- cmd_vel to motor velocity conversion
- Encoder-based odometry with TF broadcasting
- Joint state publishing for wheel visualization

Topics:
  Subscribe:
    - /cmd_vel (geometry_msgs/Twist): Velocity commands

  Publish:
    - /odom (nav_msgs/Odometry): Odometry with TF
    - /joint_states (sensor_msgs/JointState): Wheel positions
    - /base/status (std_msgs/String): Status messages

Parameters:
  - serial_port: Arduino serial port (default: /dev/ttyACM0)
  - baud_rate: Serial baud rate (default: 115200)
  - encoder_ticks_per_rev: Encoder resolution (default: 360)
  - wheel_radius: Wheel radius in meters (default: 0.075)
  - wheel_base: Distance between left/right wheels (default: 0.67)
  - publish_rate: State publish rate in Hz (default: 50.0)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster
import serial
import threading
import math
import time


class BaseHardwareBridge(Node):
    """Hardware bridge for 4-wheel differential drive mobile base"""

    def __init__(self):
        super().__init__('base_hardware_bridge')

        # Declare parameters
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('encoder_ticks_per_rev', 360)
        self.declare_parameter('wheel_radius', 0.075)  # meters
        self.declare_parameter('wheel_base', 0.67)     # meters
        self.declare_parameter('publish_rate', 50.0)   # Hz
        self.declare_parameter('cmd_timeout', 0.5)     # seconds
        self.declare_parameter('base_frame', 'body_link')
        self.declare_parameter('odom_frame', 'odom')

        # Get parameters
        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.ticks_per_rev = self.get_parameter('encoder_ticks_per_rev').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_base = self.get_parameter('wheel_base').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.cmd_timeout = self.get_parameter('cmd_timeout').value
        self.base_frame = self.get_parameter('base_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value

        # Calculate constants
        self.meters_per_tick = (2.0 * math.pi * self.wheel_radius) / self.ticks_per_rev

        # Serial connection
        self.serial_conn = None
        self.serial_lock = threading.Lock()
        self.connected = False

        # Odometry state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_encoder_counts = [0, 0, 0, 0]  # M1, M2, M3, M4
        self.last_odom_time = self.get_clock().now()

        # Wheel joint names (for visualization)
        self.wheel_joint_names = ['front_right_joint', 'front_left_joint',
                                   'back_right_joint', 'back_left_joint']
        self.wheel_joint_positions = [0.0] * 4

        # Command tracking
        self.last_cmd_time = time.time()

        # QoS profile
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, qos_profile)

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/odom', qos_profile)
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', qos_profile)
        self.status_pub = self.create_publisher(String, '/base/status', qos_profile)

        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Timers
        self.publish_timer = self.create_timer(
            1.0 / self.publish_rate, self.publish_state_callback)

        # Connect to Arduino
        self.connect_to_arduino()

        # Start serial reader thread
        self.reader_thread = threading.Thread(target=self.serial_reader_loop, daemon=True)
        self.reader_thread.start()

        self.get_logger().info('Base Hardware Bridge initialized')
        self.get_logger().info(f'Serial: {self.serial_port} @ {self.baud_rate}')
        self.get_logger().info(f'Wheel: radius={self.wheel_radius}m, base={self.wheel_base}m')

    def connect_to_arduino(self):
        """Establish serial connection to Arduino"""
        try:
            with self.serial_lock:
                if self.serial_conn and self.serial_conn.is_open:
                    self.serial_conn.close()

                self.serial_conn = serial.Serial(
                    port=self.serial_port,
                    baudrate=self.baud_rate,
                    timeout=0.1,
                    write_timeout=0.1
                )
                time.sleep(2.0)  # Arduino reset delay

                self.serial_conn.reset_input_buffer()
                self.serial_conn.reset_output_buffer()

                self.send_command("STOP")
                self.connected = True
                self.get_logger().info('Connected to Arduino')
                self.publish_status('Connected')

        except serial.SerialException as e:
            self.connected = False
            self.get_logger().error(f'Connection failed: {e}')
            self.publish_status(f'Connection failed: {e}')

    def send_command(self, command):
        """Send command to Arduino"""
        if not self.connected or not self.serial_conn:
            return False

        try:
            with self.serial_lock:
                cmd_str = f"{command}\n"
                self.serial_conn.write(cmd_str.encode('utf-8'))
                self.serial_conn.flush()
            return True
        except serial.SerialException as e:
            self.get_logger().warn(f'Serial write error: {e}')
            self.connected = False
            return False

    def serial_reader_loop(self):
        """Background thread to read Arduino responses"""
        while rclpy.ok():
            if not self.connected:
                time.sleep(2.0)
                self.connect_to_arduino()
                continue

            try:
                with self.serial_lock:
                    if self.serial_conn and self.serial_conn.in_waiting > 0:
                        line = self.serial_conn.readline().decode('utf-8').strip()
                        if line:
                            self.process_message(line)

            except serial.SerialException as e:
                self.get_logger().error(f'Serial read error: {e}')
                self.connected = False
            except UnicodeDecodeError:
                pass

            time.sleep(0.01)

    def process_message(self, message):
        """Process incoming message from Arduino"""
        parts = message.split(',')
        msg_type = parts[0]

        try:
            if msg_type == 'ODOM' and len(parts) >= 5:
                enc_counts = [int(parts[i]) for i in range(1, 5)]
                self.update_odometry(enc_counts)

            elif msg_type == 'STATUS':
                self.get_logger().info(f'Arduino: {message}')

            elif msg_type == 'ERROR':
                self.get_logger().warn(f'Arduino error: {message}')

        except (ValueError, IndexError) as e:
            self.get_logger().debug(f'Parse error: {message} - {e}')

    def cmd_vel_callback(self, msg):
        """Handle /cmd_vel commands"""
        linear_x = msg.linear.x
        angular_z = msg.angular.z

        command = f"VEL,{linear_x:.3f},{angular_z:.3f}"
        if self.send_command(command):
            self.last_cmd_time = time.time()

    def update_odometry(self, encoder_counts):
        """Update odometry from encoder counts"""
        current_time = self.get_clock().now()
        dt = (current_time - self.last_odom_time).nanoseconds / 1e9

        if dt <= 0:
            return

        # Calculate wheel distances (average left and right sides)
        left_ticks = (encoder_counts[0] + encoder_counts[2]) / 2.0 - \
                     (self.last_encoder_counts[0] + self.last_encoder_counts[2]) / 2.0
        right_ticks = (encoder_counts[1] + encoder_counts[3]) / 2.0 - \
                      (self.last_encoder_counts[1] + self.last_encoder_counts[3]) / 2.0

        left_dist = left_ticks * self.meters_per_tick
        right_dist = right_ticks * self.meters_per_tick

        # Differential drive kinematics
        distance = (left_dist + right_dist) / 2.0
        delta_theta = (right_dist - left_dist) / self.wheel_base

        # Update pose
        if abs(delta_theta) < 1e-6:
            delta_x = distance * math.cos(self.theta)
            delta_y = distance * math.sin(self.theta)
        else:
            radius = distance / delta_theta
            delta_x = radius * (math.sin(self.theta + delta_theta) - math.sin(self.theta))
            delta_y = -radius * (math.cos(self.theta + delta_theta) - math.cos(self.theta))

        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        # Velocities
        vx = distance / dt
        vth = delta_theta / dt

        # Update wheel positions
        for i in range(4):
            self.wheel_joint_positions[i] += (encoder_counts[i] - self.last_encoder_counts[i]) * \
                                             (2.0 * math.pi / self.ticks_per_rev)

        self.last_encoder_counts = encoder_counts.copy()
        self.last_odom_time = current_time

        self.publish_odometry(current_time, vx, vth)

    def publish_odometry(self, timestamp, vx, vth):
        """Publish odometry and TF"""
        q = self.euler_to_quaternion(0, 0, self.theta)

        # TF: odom -> base_frame
        t = TransformStamped()
        t.header.stamp = timestamp.to_msg()
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = q
        self.tf_broadcaster.sendTransform(t)

        # Odometry message
        odom = Odometry()
        odom.header.stamp = timestamp.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = q
        odom.twist.twist.linear.x = vx
        odom.twist.twist.angular.z = vth
        self.odom_pub.publish(odom)

    def publish_state_callback(self):
        """Publish joint states"""
        if not self.connected:
            return

        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = self.wheel_joint_names
        joint_state.position = self.wheel_joint_positions
        joint_state.velocity = [0.0] * 4
        joint_state.effort = [0.0] * 4
        self.joint_state_pub.publish(joint_state)

    def publish_status(self, status_msg):
        """Publish status message"""
        msg = String()
        msg.data = status_msg
        self.status_pub.publish(msg)

    def euler_to_quaternion(self, roll, pitch, yaw):
        """Convert Euler to quaternion"""
        cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
        cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
        cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)

        q = Quaternion()
        q.w = cr * cp * cy + sr * sp * sy
        q.x = sr * cp * cy - cr * sp * sy
        q.y = cr * sp * cy + sr * cp * sy
        q.z = cr * cp * sy - sr * sp * cy
        return q

    def destroy_node(self):
        """Cleanup"""
        if self.connected:
            self.send_command("STOP")
            time.sleep(0.1)
        with self.serial_lock:
            if self.serial_conn and self.serial_conn.is_open:
                self.serial_conn.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BaseHardwareBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
