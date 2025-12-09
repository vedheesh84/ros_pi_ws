#!/usr/bin/env python3
"""
Odometry Publisher Node
========================

Calculates odometry from encoder ticks and publishes odom + TF.
This node subscribes to encoder ticks from the serial bridge and computes
robot pose and velocity using differential drive kinematics.

Topics:
    Subscribe:
        /encoder_ticks (std_msgs/Int64MultiArray): Raw encoder counts [FL, FR, BL, BR]

    Publish:
        /odom (nav_msgs/Odometry): Odometry with pose and twist
        /joint_states (sensor_msgs/JointState): Wheel joint positions

    Broadcast TF:
        odom -> body_link

Parameters:
    encoder_ticks_per_rev: Encoder resolution (default: 360)
    wheel_radius: Wheel radius in meters (default: 0.075)
    wheel_base: Distance between left/right wheels (default: 0.35)
    base_frame: Robot base frame (default: body_link)
    odom_frame: Odometry frame (default: odom)
    publish_tf: Whether to publish TF (default: true)
    publish_rate: Publishing rate in Hz (default: 50.0)

Usage:
    ros2 run base_controller odom_publisher_node.py
"""

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Int64MultiArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster

import math


def quaternion_from_euler(roll, pitch, yaw):
    """Convert Euler angles to quaternion."""
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


class OdomPublisherNode(Node):
    def __init__(self):
        super().__init__('odom_publisher')

        # Declare parameters
        self.declare_parameter('encoder_ticks_per_rev', 360)
        self.declare_parameter('wheel_radius', 0.075)
        self.declare_parameter('wheel_base', 0.35)
        self.declare_parameter('base_frame', 'body_link')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('publish_rate', 50.0)

        # Wheel joint names
        self.declare_parameter('wheel_joints', [
            'front_left_joint',
            'front_right_joint',
            'back_left_joint',
            'back_right_joint'
        ])

        # Covariance values
        self.declare_parameter('pose_covariance_diagonal', [0.01, 0.01, 0.01, 0.01, 0.01, 0.03])
        self.declare_parameter('twist_covariance_diagonal', [0.01, 0.01, 0.01, 0.01, 0.01, 0.03])

        # Get parameters
        self.encoder_ticks_per_rev = self.get_parameter('encoder_ticks_per_rev').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_base = self.get_parameter('wheel_base').value
        self.base_frame = self.get_parameter('base_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.publish_tf = self.get_parameter('publish_tf').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.wheel_joints = self.get_parameter('wheel_joints').value
        self.pose_cov_diag = self.get_parameter('pose_covariance_diagonal').value
        self.twist_cov_diag = self.get_parameter('twist_covariance_diagonal').value

        # Calculate meters per encoder tick
        wheel_circumference = 2.0 * math.pi * self.wheel_radius
        self.meters_per_tick = wheel_circumference / self.encoder_ticks_per_rev
        self.radians_per_tick = (2.0 * math.pi) / self.encoder_ticks_per_rev

        # Robot pose state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Velocity state
        self.vx = 0.0
        self.vy = 0.0
        self.vth = 0.0

        # Encoder state
        self.last_encoder_counts = None
        self.last_time = None

        # Wheel positions (for joint_states)
        self.wheel_positions = [0.0, 0.0, 0.0, 0.0]

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)

        # TF broadcaster
        if self.publish_tf:
            self.tf_broadcaster = TransformBroadcaster(self)

        # Subscriber
        self.encoder_sub = self.create_subscription(
            Int64MultiArray,
            '/encoder_ticks',
            self.encoder_callback,
            10
        )

        self.get_logger().info('Odometry Publisher Node started')
        self.get_logger().info(f'  Wheel radius: {self.wheel_radius}m')
        self.get_logger().info(f'  Wheel base: {self.wheel_base}m')
        self.get_logger().info(f'  Encoder ticks/rev: {self.encoder_ticks_per_rev}')
        self.get_logger().info(f'  Meters per tick: {self.meters_per_tick:.6f}m')

    def encoder_callback(self, msg):
        """Process encoder ticks and compute odometry."""
        current_time = self.get_clock().now()
        current_counts = list(msg.data)  # [FL, FR, BL, BR]

        if len(current_counts) != 4:
            self.get_logger().warn(f'Invalid encoder data length: {len(current_counts)}')
            return

        # First message - just store and return
        if self.last_encoder_counts is None:
            self.last_encoder_counts = current_counts
            self.last_time = current_time
            return

        # Calculate time delta
        dt_ns = (current_time - self.last_time).nanoseconds
        dt = dt_ns / 1e9  # Convert to seconds

        if dt <= 0:
            return

        # Calculate encoder deltas
        delta_counts = [
            current_counts[i] - self.last_encoder_counts[i]
            for i in range(4)
        ]

        # Average left and right sides (4-wheel differential drive)
        # FL + BL = left side, FR + BR = right side
        left_delta = (delta_counts[0] + delta_counts[2]) / 2.0
        right_delta = (delta_counts[1] + delta_counts[3]) / 2.0

        # Convert to distance traveled
        left_dist = left_delta * self.meters_per_tick
        right_dist = right_delta * self.meters_per_tick

        # Differential drive kinematics
        distance = (left_dist + right_dist) / 2.0
        delta_theta = (right_dist - left_dist) / self.wheel_base

        # Update pose using exact integration for arc motion
        if abs(delta_theta) < 1e-6:
            # Straight line motion
            self.x += distance * math.cos(self.theta)
            self.y += distance * math.sin(self.theta)
        else:
            # Arc motion - use exact solution
            radius = distance / delta_theta
            self.x += radius * (math.sin(self.theta + delta_theta) - math.sin(self.theta))
            self.y += -radius * (math.cos(self.theta + delta_theta) - math.cos(self.theta))

        self.theta += delta_theta

        # Normalize theta to [-pi, pi]
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        # Calculate velocities
        self.vx = distance / dt
        self.vy = 0.0  # No lateral velocity for differential drive
        self.vth = delta_theta / dt

        # Update wheel positions (radians)
        for i in range(4):
            self.wheel_positions[i] += delta_counts[i] * self.radians_per_tick

        # Store for next iteration
        self.last_encoder_counts = current_counts
        self.last_time = current_time

        # Publish odometry and joint states
        self.publish_odometry(current_time)
        self.publish_joint_states(current_time)

    def publish_odometry(self, stamp):
        """Publish odometry message and TF."""
        # Create quaternion from yaw
        q = quaternion_from_euler(0.0, 0.0, self.theta)

        # Create odometry message
        odom = Odometry()
        odom.header.stamp = stamp.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        # Set pose
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = q

        # Set pose covariance (6x6 matrix as 36 element array)
        odom.pose.covariance = [0.0] * 36
        odom.pose.covariance[0] = self.pose_cov_diag[0]   # x
        odom.pose.covariance[7] = self.pose_cov_diag[1]   # y
        odom.pose.covariance[14] = self.pose_cov_diag[2]  # z
        odom.pose.covariance[21] = self.pose_cov_diag[3]  # roll
        odom.pose.covariance[28] = self.pose_cov_diag[4]  # pitch
        odom.pose.covariance[35] = self.pose_cov_diag[5]  # yaw

        # Set twist (in body frame)
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = self.vth

        # Set twist covariance
        odom.twist.covariance = [0.0] * 36
        odom.twist.covariance[0] = self.twist_cov_diag[0]
        odom.twist.covariance[7] = self.twist_cov_diag[1]
        odom.twist.covariance[14] = self.twist_cov_diag[2]
        odom.twist.covariance[21] = self.twist_cov_diag[3]
        odom.twist.covariance[28] = self.twist_cov_diag[4]
        odom.twist.covariance[35] = self.twist_cov_diag[5]

        # Publish odometry
        self.odom_pub.publish(odom)

        # Publish TF
        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = stamp.to_msg()
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            t.transform.rotation = q

            self.tf_broadcaster.sendTransform(t)

    def publish_joint_states(self, stamp):
        """Publish wheel joint states."""
        js = JointState()
        js.header.stamp = stamp.to_msg()
        js.name = self.wheel_joints
        js.position = self.wheel_positions
        js.velocity = []  # Could add wheel velocities if needed
        js.effort = []

        self.joint_state_pub.publish(js)


def main(args=None):
    rclpy.init(args=args)
    node = OdomPublisherNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
