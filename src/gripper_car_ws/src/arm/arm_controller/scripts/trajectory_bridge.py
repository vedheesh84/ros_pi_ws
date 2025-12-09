#!/usr/bin/env python3
"""
MoveIt Trajectory Bridge
========================
Bridges MoveIt trajectory execution to servo commands.

This node provides FollowJointTrajectory action servers for both
arm and gripper groups, forwarding commands to servo_bridge.

Action Servers:
    - /arm_controller/follow_joint_trajectory
    - /gripper_controller/follow_joint_trajectory

Publish:
    - /arm/joint_commands (sensor_msgs/JointState)
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from control_msgs.action import FollowJointTrajectory, GripperCommand
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
import time


class TrajectoryBridge(Node):
    """Bridge MoveIt trajectories to servo commands for arm and gripper"""

    def __init__(self):
        super().__init__('trajectory_bridge')

        # Parameters for rate limiting
        self.declare_parameter('min_command_interval', 1.0)  # seconds between commands
        self.declare_parameter('skip_intermediate_points', True)  # skip to final point

        self.min_command_interval = self.get_parameter('min_command_interval').value
        self.skip_intermediate = self.get_parameter('skip_intermediate_points').value

        self.callback_group = ReentrantCallbackGroup()

        # Joint command publisher (shared by both controllers)
        self.joint_cmd_pub = self.create_publisher(
            JointState, '/arm/joint_commands', 10)

        # Action server for arm controller
        self._arm_action_server = ActionServer(
            self,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory',
            self.execute_trajectory,
            callback_group=self.callback_group
        )

        # Action server for gripper controller (MoveIt uses GripperCommand)
        self._gripper_action_server = ActionServer(
            self,
            GripperCommand,
            '/gripper_controller/gripper_cmd',
            self.execute_gripper,
            callback_group=self.callback_group
        )

        self.get_logger().info(
            f'Trajectory Bridge initialized for arm and gripper '
            f'(min_interval={self.min_command_interval}s)'
        )

    async def execute_trajectory(self, goal_handle):
        """Execute a trajectory with rate limiting for Arduino compatibility"""
        self.get_logger().info('Executing trajectory...')

        trajectory = goal_handle.request.trajectory
        feedback_msg = FollowJointTrajectory.Feedback()
        result = FollowJointTrajectory.Result()

        try:
            num_points = len(trajectory.points)

            if num_points == 0:
                goal_handle.succeed()
                result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                return result

            # If skip_intermediate is enabled, only send key points
            if self.skip_intermediate and num_points > 2:
                # Send only first and last points (or sample at intervals)
                points_to_send = [0, num_points - 1]
                self.get_logger().info(
                    f'Skipping intermediate points: {num_points} -> {len(points_to_send)} points')
            else:
                points_to_send = list(range(num_points))

            last_send_time = 0.0

            for idx, i in enumerate(points_to_send):
                point = trajectory.points[i]

                # Rate limiting: ensure minimum interval between commands
                current_time = time.time()
                time_since_last = current_time - last_send_time
                if time_since_last < self.min_command_interval:
                    wait_time = self.min_command_interval - time_since_last
                    self.get_logger().debug(f'Rate limiting: waiting {wait_time:.2f}s')
                    time.sleep(wait_time)

                # Send joint commands
                joint_cmd = JointState()
                joint_cmd.header.stamp = self.get_clock().now().to_msg()
                joint_cmd.name = list(trajectory.joint_names)
                joint_cmd.position = list(point.positions)
                self.joint_cmd_pub.publish(joint_cmd)
                last_send_time = time.time()

                self.get_logger().info(
                    f'Sent point {idx+1}/{len(points_to_send)} '
                    f'(original: {i+1}/{num_points})')

                # Update feedback
                feedback_msg.joint_names = list(trajectory.joint_names)
                feedback_msg.actual.positions = list(point.positions)
                feedback_msg.desired.positions = list(point.positions)
                goal_handle.publish_feedback(feedback_msg)

            # Wait for Arduino to complete the final movement
            self.get_logger().info(
                f'Waiting {self.min_command_interval}s for Arduino to complete...')
            time.sleep(self.min_command_interval)

            goal_handle.succeed()
            result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
            self.get_logger().info('Trajectory execution completed')

        except Exception as e:
            self.get_logger().error(f'Trajectory execution failed: {e}')
            goal_handle.abort()
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL

        return result

    async def execute_gripper(self, goal_handle):
        """Handle GripperCommand action goals from MoveIt and forward to servo bridge."""
        self.get_logger().info('Executing gripper command...')

        result = GripperCommand.Result()

        try:
            # GripperCommand goal contains a `command` with `position` and optional `max_effort`
            cmd = goal_handle.request.command
            position = float(cmd.position)

            # Publish as a JointState containing only the gripper joint name
            joint_cmd = JointState()
            joint_cmd.header.stamp = self.get_clock().now().to_msg()
            joint_cmd.name = ['left_gear_joint']
            joint_cmd.position = [position]
            self.joint_cmd_pub.publish(joint_cmd)

            # Wait a short interval for Arduino to react
            time.sleep(self.min_command_interval)

            # Succeed the action and return the achieved position
            goal_handle.succeed()
            try:
                result.position = position
            except Exception:
                # Some result message variants may have different fields; ignore
                pass
            return result

        except Exception as e:
            self.get_logger().error(f'Gripper command failed: {e}')
            goal_handle.abort()
            # Return empty result on abort
            return result


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
