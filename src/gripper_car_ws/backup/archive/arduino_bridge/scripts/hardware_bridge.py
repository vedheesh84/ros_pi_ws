#!/usr/bin/env python3
"""
UNIFIED MOBILE MANIPULATOR HARDWARE BRIDGE
==========================================
ROS2 node that interfaces with the unified Arduino controller

FUNCTIONALITY:
- Mobile base velocity control (cmd_vel -> Arduino)
- Encoder-based odometry publishing (Arduino -> odom + TF)
- Robotic arm servo control (joint commands -> Arduino)
- Joint state publishing for arm (Arduino -> joint_states)
- System status monitoring

TOPICS:
Subscribe:
  - /cmd_vel (geometry_msgs/Twist): Mobile base velocity commands
  - /arm/joint_commands (sensor_msgs/JointState): Arm position commands

Publish:
  - /odom (nav_msgs/Odometry): Mobile base odometry with TF
  - /joint_states (sensor_msgs/JointState): Combined base + arm joint states
  - /hardware/status (std_msgs/String): System status messages

PARAMETERS:
  - serial_port: Arduino serial port (default: /dev/ttyACM0)
  - baud_rate: Serial baud rate (default: 115200)
  - encoder_ticks_per_rev: Encoder resolution (default: 360)
  - wheel_radius: Wheel radius in meters (default: 0.075)
  - wheel_base: Distance between left/right wheels (default: 0.67)
  - publish_rate: State publish rate in Hz (default: 50.0)

AUTHOR: Mobile Manipulator Integration
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
import numpy as np

class UnifiedHardwareBridge(Node):
    """
    Unified hardware bridge for mobile manipulator
    Handles both mobile base and robotic arm through single Arduino
    """

    def __init__(self):
        super().__init__('unified_hardware_bridge')

        # Declare parameters
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('encoder_ticks_per_rev', 360)
        self.declare_parameter('wheel_radius', 0.075)  # meters
        self.declare_parameter('wheel_base', 0.67)     # meters
        self.declare_parameter('publish_rate', 50.0)   # Hz
        self.declare_parameter('cmd_timeout', 0.5)     # seconds
        self.declare_parameter('reconnect_delay', 2.0) # seconds

        # Get parameters
        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.ticks_per_rev = self.get_parameter('encoder_ticks_per_rev').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_base = self.get_parameter('wheel_base').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.cmd_timeout = self.get_parameter('cmd_timeout').value
        self.reconnect_delay = self.get_parameter('reconnect_delay').value

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

        # Joint state (arm)
        self.arm_joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4',
                                 'gripper_base_joint', 'left_gear_joint']
        self.arm_joint_positions = [0.0] * 6

        # Wheel joint names (for visualization)
        self.wheel_joint_names = ['front_right_joint', 'front_left_joint',
                                   'back_right_joint', 'back_left_joint']
        self.wheel_joint_positions = [0.0] * 4

        # Command tracking
        self.last_cmd_time = time.time()

        # QoS profile for real-time communication
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, qos_profile)

        self.joint_cmd_sub = self.create_subscription(
            JointState, '/arm/joint_commands', self.joint_cmd_callback, qos_profile)

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/odom', qos_profile)
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', qos_profile)
        self.status_pub = self.create_publisher(String, '/hardware/status', qos_profile)

        # TF broadcaster for odometry
        self.tf_broadcaster = TransformBroadcaster(self)

        # Timers
        self.publish_timer = self.create_timer(
            1.0 / self.publish_rate, self.publish_state_callback)

        # Connect to Arduino
        self.connect_to_arduino()

        # Start serial reader thread
        self.reader_thread = threading.Thread(target=self.serial_reader_loop, daemon=True)
        self.reader_thread.start()

        self.get_logger().info('Unified Hardware Bridge initialized')
        self.get_logger().info(f'Serial port: {self.serial_port} @ {self.baud_rate} baud')
        self.get_logger().info(f'Wheel radius: {self.wheel_radius}m, base: {self.wheel_base}m')

    # ========================================================================
    # SERIAL COMMUNICATION
    # ========================================================================

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

                # Clear buffer
                self.serial_conn.reset_input_buffer()
                self.serial_conn.reset_output_buffer()

                # Send stop command for safety
                self.send_command("STOP")

                self.connected = True
                self.get_logger().info('✓ Connected to Arduino')
                self.publish_status('Connected to Arduino')

        except serial.SerialException as e:
            self.connected = False
            self.get_logger().error(f'✗ Failed to connect: {e}')
            self.publish_status(f'Connection failed: {e}')

    def send_command(self, command):
        """Send command to Arduino with error handling"""
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
        reconnect_attempts = 0

        while rclpy.ok():
            if not self.connected:
                # Attempt reconnection
                time.sleep(self.reconnect_delay)
                reconnect_attempts += 1
                self.get_logger().warn(f'Reconnection attempt {reconnect_attempts}...')
                self.connect_to_arduino()
                continue

            try:
                with self.serial_lock:
                    if self.serial_conn and self.serial_conn.in_waiting > 0:
                        line = self.serial_conn.readline().decode('utf-8').strip()
                        if line:
                            self.process_arduino_message(line)
                            reconnect_attempts = 0

            except serial.SerialException as e:
                self.get_logger().error(f'Serial read error: {e}')
                self.connected = False

            except UnicodeDecodeError:
                pass  # Ignore corrupt data

            time.sleep(0.01)  # 100 Hz read loop

    def process_arduino_message(self, message):
        """Process incoming message from Arduino"""
        if not message:
            return

        parts = message.split(',')
        msg_type = parts[0]

        try:
            # Odometry data: ODOM,enc1,enc2,enc3,enc4,timestamp
            if msg_type == 'ODOM' and len(parts) == 6:
                enc_counts = [int(parts[i]) for i in range(1, 5)]
                timestamp_ms = int(parts[5])
                self.update_odometry(enc_counts)

            # Servo positions: SERVO_POS,pos0,pos1,...,pos5
            elif msg_type == 'SERVO_POS' and len(parts) == 7:
                for i in range(6):
                    angle_deg = int(parts[i + 1])
                    # Convert angle to radians and map to joint range
                    self.arm_joint_positions[i] = self.angle_to_joint_position(i, angle_deg)

            # Status messages
            elif msg_type in ['STATUS', 'INFO', 'OK']:
                self.get_logger().info(f'Arduino: {message}')

            # Errors
            elif msg_type == 'ERROR':
                self.get_logger().warn(f'Arduino error: {message}')

            # Emergency stop
            elif msg_type == 'EMERGENCY_STOP':
                self.get_logger().error('⚠ EMERGENCY STOP ACTIVATED!')
                self.publish_status('EMERGENCY_STOP')

            # Heartbeat
            elif msg_type == 'HEARTBEAT':
                pass  # Silent heartbeat

        except (ValueError, IndexError) as e:
            self.get_logger().debug(f'Parse error: {message} - {e}')

    # ========================================================================
    # MOBILE BASE CONTROL
    # ========================================================================

    def cmd_vel_callback(self, msg):
        """Handle /cmd_vel commands for mobile base"""
        linear_x = msg.linear.x
        angular_z = msg.angular.z

        # Send velocity command to Arduino
        command = f"VEL,{linear_x:.3f},{angular_z:.3f}"
        success = self.send_command(command)

        if success:
            self.last_cmd_time = time.time()
        else:
            self.get_logger().warn('Failed to send velocity command')

    def update_odometry(self, encoder_counts):
        """
        Update odometry from encoder counts
        Uses differential drive kinematics
        """
        current_time = self.get_clock().now()
        dt = (current_time - self.last_odom_time).nanoseconds / 1e9

        if dt <= 0:
            return

        # Calculate distance traveled by each wheel
        # Average left side (M1 + M3) and right side (M2 + M4)
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
            # Straight line motion
            delta_x = distance * math.cos(self.theta)
            delta_y = distance * math.sin(self.theta)
        else:
            # Arc motion
            radius = distance / delta_theta
            delta_x = radius * (math.sin(self.theta + delta_theta) - math.sin(self.theta))
            delta_y = -radius * (math.cos(self.theta + delta_theta) - math.cos(self.theta))

        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta

        # Normalize theta to [-pi, pi]
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        # Calculate velocities
        vx = distance / dt
        vth = delta_theta / dt

        # Update wheel positions for visualization
        for i in range(4):
            self.wheel_joint_positions[i] += (encoder_counts[i] - self.last_encoder_counts[i]) * \
                                             (2.0 * math.pi / self.ticks_per_rev)

        # Store for next iteration
        self.last_encoder_counts = encoder_counts.copy()
        self.last_odom_time = current_time

        # Publish odometry
        self.publish_odometry(current_time, vx, vth)

    def publish_odometry(self, timestamp, vx, vth):
        """Publish odometry message and TF transform"""
        # Create quaternion from yaw
        q = self.euler_to_quaternion(0, 0, self.theta)

        # Publish TF: odom -> body_link
        t = TransformStamped()
        t.header.stamp = timestamp.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'body_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = q

        self.tf_broadcaster.sendTransform(t)

        # Publish Odometry message
        odom = Odometry()
        odom.header.stamp = timestamp.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'body_link'

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = q

        odom.twist.twist.linear.x = vx
        odom.twist.twist.angular.z = vth

        self.odom_pub.publish(odom)

    # ========================================================================
    # ROBOTIC ARM CONTROL
    # ========================================================================

    def joint_cmd_callback(self, msg):
        """Handle /arm/joint_commands for robotic arm"""
        # Map joint names to servo indices
        for i, joint_name in enumerate(msg.name):
            if joint_name in self.arm_joint_names:
                idx = self.arm_joint_names.index(joint_name)
                position_rad = msg.position[i]

                # Convert joint position to servo angle
                angle_deg = self.joint_position_to_angle(idx, position_rad)

                # Send servo command
                command = f"SERVO,{idx},{angle_deg}"
                self.send_command(command)

    def joint_position_to_angle(self, joint_idx, position_rad):
        """
        Convert joint position (radians) to servo angle (0-180 degrees)
        Mapping depends on joint type and physical configuration
        """
        # Default: map [-pi, pi] to [0, 180]
        # joint_1-4: Revolute joints
        # gripper_base_joint: Gripper rotation
        # left_gear_joint: Gripper open/close

        if joint_idx < 4:  # Arm joints
            # Map [-pi, pi] to [0, 180]
            angle = (position_rad + math.pi) * (180.0 / (2.0 * math.pi))
        elif joint_idx == 4:  # Gripper base
            angle = (position_rad + math.pi / 2) * (180.0 / math.pi)
        else:  # Gripper gear (0-180 already)
            angle = position_rad * (180.0 / math.pi)

        return int(np.clip(angle, 0, 180))

    def angle_to_joint_position(self, joint_idx, angle_deg):
        """
        Convert servo angle (0-180 degrees) to joint position (radians)
        Inverse of joint_position_to_angle
        """
        if joint_idx < 4:  # Arm joints
            position_rad = (angle_deg * (2.0 * math.pi / 180.0)) - math.pi
        elif joint_idx == 4:  # Gripper base
            position_rad = (angle_deg * (math.pi / 180.0)) - math.pi / 2
        else:  # Gripper gear
            position_rad = angle_deg * (math.pi / 180.0)

        return position_rad

    # ========================================================================
    # STATE PUBLISHING
    # ========================================================================

    def publish_state_callback(self):
        """Publish joint states at fixed rate"""
        if not self.connected:
            return

        # Create joint state message for ALL joints (base + arm)
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()

        # Add wheel joints
        joint_state.name.extend(self.wheel_joint_names)
        joint_state.position.extend(self.wheel_joint_positions)
        joint_state.velocity.extend([0.0] * 4)
        joint_state.effort.extend([0.0] * 4)

        # Add arm joints
        joint_state.name.extend(self.arm_joint_names)
        joint_state.position.extend(self.arm_joint_positions)
        joint_state.velocity.extend([0.0] * 6)
        joint_state.effort.extend([0.0] * 6)

        self.joint_state_pub.publish(joint_state)

    def publish_status(self, status_msg):
        """Publish status message"""
        msg = String()
        msg.data = status_msg
        self.status_pub.publish(msg)

    # ========================================================================
    # UTILITY FUNCTIONS
    # ========================================================================

    def euler_to_quaternion(self, roll, pitch, yaw):
        """Convert Euler angles to quaternion"""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q = Quaternion()
        q.w = cr * cp * cy + sr * sp * sy
        q.x = sr * cp * cy - cr * sp * sy
        q.y = cr * sp * cy + sr * cp * sy
        q.z = cr * cp * sy - sr * sp * cy

        return q

    def destroy_node(self):
        """Cleanup on shutdown"""
        self.get_logger().info('Shutting down hardware bridge...')

        # Stop motors
        if self.connected:
            self.send_command("STOP")
            time.sleep(0.1)

        # Close serial connection
        with self.serial_lock:
            if self.serial_conn and self.serial_conn.is_open:
                self.serial_conn.close()

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UnifiedHardwareBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
