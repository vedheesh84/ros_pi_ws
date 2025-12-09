#!/usr/bin/env python3
"""
Arm Interface Module
====================
MoveIt2 wrapper for controlling the robotic arm in pick-and-place operations.

This module provides a high-level interface for arm movements, abstracting
the MoveIt2 planning and execution API. It supports:
- Moving to named poses (defined in MoveIt SRDF)
- Moving to custom joint positions (defined in arm_poses.yaml)
- Gripper open/close operations
- Velocity and acceleration scaling

Usage:
    from arm_interface import ArmInterface

    # Initialize (requires MoveIt to be running)
    arm = ArmInterface(node, config_path='/path/to/arm_poses.yaml')

    # Move to named pose
    success = arm.move_to_named_pose('home')

    # Move to custom pose from config
    success = arm.move_to_config_pose('scan_pose')

    # Gripper control
    arm.open_gripper()
    arm.close_gripper()

Dependencies:
    - moveit_py or moveit_commander
    - MoveIt2 running with 'arm' and 'gripper' planning groups

Note: This interface uses Python bindings for MoveIt2. Ensure the
move_group node is running before initializing.
"""

import yaml
import threading
from typing import Dict, List, Optional
from dataclasses import dataclass, field
from enum import Enum

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration


class ArmState(Enum):
    """Current state of the arm interface."""
    IDLE = "idle"
    MOVING = "moving"
    ERROR = "error"


@dataclass
class ArmPose:
    """
    Represents a joint configuration for the arm.

    Attributes:
        joint_positions: Dictionary mapping joint names to positions (radians)
        name: Pose name for identification
    """
    joint_positions: Dict[str, float]
    name: str = ""

    def to_list(self, joint_order: List[str]) -> List[float]:
        """
        Convert to ordered list of positions.

        Args:
            joint_order: List of joint names in desired order

        Returns:
            List of joint positions in the specified order
        """
        return [self.joint_positions.get(name, 0.0) for name in joint_order]


@dataclass
class ArmConfig:
    """
    Configuration for the arm interface.

    Attributes:
        arm_group: MoveIt planning group name for arm
        gripper_group: MoveIt planning group name for gripper
        arm_joints: Ordered list of arm joint names
        gripper_joints: Ordered list of gripper joint names
        velocity_scaling: Velocity scaling factor (0.0-1.0)
        acceleration_scaling: Acceleration scaling factor (0.0-1.0)
        planning_time: Maximum planning time in seconds
        motion_timeout: Timeout for motion execution in seconds
    """
    arm_group: str = "arm"
    gripper_group: str = "gripper"
    arm_joints: List[str] = field(default_factory=lambda: [
        'joint_1', 'joint_2', 'joint_3', 'joint_4', 'gripper_base_joint'
    ])
    gripper_joints: List[str] = field(default_factory=lambda: ['left_gear_joint'])
    velocity_scaling: float = 0.5
    acceleration_scaling: float = 0.5
    planning_time: float = 5.0
    motion_timeout: float = 30.0


class ArmInterface:
    """
    High-level interface for robotic arm control using trajectory actions.

    This class wraps the FollowJointTrajectory action interface to provide
    easy-to-use methods for pick-and-place operations. It manages arm and
    gripper movements independently.

    Note: This implementation uses direct trajectory actions rather than
    MoveIt for simplicity. For collision-aware planning, use MoveIt Python
    bindings instead.
    """

    def __init__(
        self,
        node: Node,
        poses_config_path: Optional[str] = None,
        arm_config: Optional[ArmConfig] = None
    ):
        """
        Initialize the arm interface.

        Args:
            node: ROS2 node for creating clients and subscriptions
            poses_config_path: Path to arm_poses.yaml configuration
            arm_config: Arm configuration (uses defaults if None)
        """
        self.node = node
        self.logger = node.get_logger()
        self.config = arm_config or ArmConfig()

        # State tracking
        self.state = ArmState.IDLE
        self._state_lock = threading.Lock()

        # Custom poses from config file
        self.custom_poses: Dict[str, ArmPose] = {}
        if poses_config_path:
            self._load_poses_config(poses_config_path)

        # Named poses (typically defined in MoveIt SRDF)
        # These are common preset positions
        self.named_poses = {
            'home': ArmPose({
                'joint_1': 0.0, 'joint_2': 0.0, 'joint_3': 0.0,
                'joint_4': 0.0, 'gripper_base_joint': 0.0
            }, 'home'),
            'stowed': ArmPose({
                'joint_1': 0.0, 'joint_2': -0.5, 'joint_3': 1.0,
                'joint_4': 0.5, 'gripper_base_joint': 0.0
            }, 'stowed'),
            'ready': ArmPose({
                'joint_1': 0.0, 'joint_2': 0.3, 'joint_3': -0.3,
                'joint_4': 0.0, 'gripper_base_joint': 0.0
            }, 'ready'),
        }

        # Gripper positions
        self.gripper_open_position = 0.04   # Fully open
        self.gripper_closed_position = 0.0  # Fully closed

        # Current joint states
        self._current_joint_state: Optional[JointState] = None
        self._joint_state_lock = threading.Lock()

        # Subscribe to joint states
        self.joint_state_sub = node.create_subscription(
            JointState,
            '/joint_states',
            self._joint_state_callback,
            10
        )

        # Action client for arm trajectory control
        self.arm_trajectory_client = ActionClient(
            node,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory'
        )

        # Publisher for gripper commands (direct joint control)
        self.gripper_cmd_pub = node.create_publisher(
            JointState,
            '/arm/joint_commands',
            10
        )

        self.logger.info(
            f"Arm interface initialized:\n"
            f"  Arm joints: {self.config.arm_joints}\n"
            f"  Gripper joints: {self.config.gripper_joints}\n"
            f"  Custom poses loaded: {list(self.custom_poses.keys())}"
        )

    def _load_poses_config(self, config_path: str) -> None:
        """
        Load custom arm poses from YAML configuration.

        Args:
            config_path: Path to arm_poses.yaml file
        """
        try:
            with open(config_path, 'r') as f:
                config = yaml.safe_load(f)

            if 'arm_poses' not in config:
                self.logger.warning("No 'arm_poses' key in config file")
                return

            for name, joints in config['arm_poses'].items():
                self.custom_poses[name] = ArmPose(
                    joint_positions={k: float(v) for k, v in joints.items()},
                    name=name
                )

            self.logger.info(f"Loaded {len(self.custom_poses)} custom poses")

        except Exception as e:
            self.logger.error(f"Failed to load poses config: {e}")

    def _joint_state_callback(self, msg: JointState) -> None:
        """
        Callback for joint state updates.

        Args:
            msg: Current joint state message
        """
        with self._joint_state_lock:
            self._current_joint_state = msg

    def get_current_joint_positions(self) -> Optional[Dict[str, float]]:
        """
        Get current joint positions.

        Returns:
            Dictionary of joint names to positions, or None if unavailable
        """
        with self._joint_state_lock:
            if self._current_joint_state is None:
                return None

            return dict(zip(
                self._current_joint_state.name,
                self._current_joint_state.position
            ))

    def wait_for_action_servers(self, timeout: float = 10.0) -> bool:
        """
        Wait for action servers to become available.

        Args:
            timeout: Maximum time to wait in seconds

        Returns:
            True if servers available, False on timeout
        """
        self.logger.info("Waiting for arm trajectory action servers...")

        arm_ready = self.arm_trajectory_client.wait_for_server(timeout_sec=timeout)

        if arm_ready:
            self.logger.info("Action servers ready")
            return True
        else:
            self.logger.error("Action servers not available")
            return False

    def move_to_joint_positions(
        self,
        joint_positions: Dict[str, float],
        duration: float = 2.0
    ) -> bool:
        """
        Move arm to specified joint positions.

        Args:
            joint_positions: Dictionary of joint names to target positions
            duration: Time to complete motion in seconds

        Returns:
            True if motion succeeded, False otherwise
        """
        with self._state_lock:
            if self.state == ArmState.MOVING:
                self.logger.warning("Arm already in motion")
                return False
            self.state = ArmState.MOVING

        try:
            # Build trajectory message
            trajectory = JointTrajectory()
            trajectory.joint_names = self.config.arm_joints

            point = JointTrajectoryPoint()
            point.positions = [
                joint_positions.get(name, 0.0)
                for name in self.config.arm_joints
            ]
            point.time_from_start = Duration(
                sec=int(duration),
                nanosec=int((duration % 1) * 1e9)
            )
            trajectory.points = [point]

            # Create goal
            goal = FollowJointTrajectory.Goal()
            goal.trajectory = trajectory

            # Send goal and wait
            future = self.arm_trajectory_client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)

            if future.result() is None:
                self.logger.error("Failed to send trajectory goal")
                return False

            goal_handle = future.result()
            if not goal_handle.accepted:
                self.logger.error("Trajectory goal rejected")
                return False

            # Wait for result
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(
                self.node, result_future,
                timeout_sec=self.config.motion_timeout
            )

            if result_future.result() is None:
                self.logger.error("Trajectory execution failed")
                return False

            self.logger.info("Arm motion completed")
            return True

        except Exception as e:
            self.logger.error(f"Motion failed: {e}")
            with self._state_lock:
                self.state = ArmState.ERROR
            return False

        finally:
            with self._state_lock:
                if self.state == ArmState.MOVING:
                    self.state = ArmState.IDLE

    def move_to_named_pose(self, pose_name: str, duration: float = 2.0) -> bool:
        """
        Move arm to a named pose.

        Args:
            pose_name: Name of preset pose (home, stowed, ready)
            duration: Time to complete motion

        Returns:
            True if motion succeeded, False otherwise
        """
        if pose_name not in self.named_poses:
            self.logger.error(f"Unknown named pose: {pose_name}")
            return False

        pose = self.named_poses[pose_name]
        self.logger.info(f"Moving to named pose: {pose_name}")
        return self.move_to_joint_positions(pose.joint_positions, duration)

    def move_to_config_pose(self, pose_name: str, duration: float = 2.0) -> bool:
        """
        Move arm to a custom pose from configuration.

        Args:
            pose_name: Name of pose defined in arm_poses.yaml
            duration: Time to complete motion

        Returns:
            True if motion succeeded, False otherwise
        """
        if pose_name not in self.custom_poses:
            self.logger.error(f"Unknown config pose: {pose_name}")
            return False

        pose = self.custom_poses[pose_name]
        self.logger.info(f"Moving to config pose: {pose_name}")
        return self.move_to_joint_positions(pose.joint_positions, duration)

    def _move_gripper(self, position: float, duration: float = 1.0) -> bool:
        """
        Move gripper to specified position using direct joint command.

        Args:
            position: Target gripper position (radians)
            duration: Time to wait after sending command

        Returns:
            True if command sent successfully, False otherwise
        """
        try:
            # Create joint state message for gripper
            joint_cmd = JointState()
            joint_cmd.header.stamp = self.node.get_clock().now().to_msg()
            joint_cmd.name = self.config.gripper_joints
            joint_cmd.position = [position]

            # Publish gripper command
            self.gripper_cmd_pub.publish(joint_cmd)

            # Wait for gripper motion to complete
            import time
            time.sleep(duration)

            self.logger.info(f"Gripper moved to position: {position:.3f}")
            return True

        except Exception as e:
            self.logger.error(f"Gripper motion failed: {e}")
            return False

    def open_gripper(self, duration: float = 1.0) -> bool:
        """
        Open the gripper.

        Args:
            duration: Time to complete motion

        Returns:
            True if motion succeeded, False otherwise
        """
        self.logger.info("Opening gripper")
        return self._move_gripper(self.gripper_open_position, duration)

    def close_gripper(self, duration: float = 1.0) -> bool:
        """
        Close the gripper.

        Args:
            duration: Time to complete motion

        Returns:
            True if motion succeeded, False otherwise
        """
        self.logger.info("Closing gripper")
        return self._move_gripper(self.gripper_closed_position, duration)

    def stow(self) -> bool:
        """
        Stow the arm in a safe position.

        Returns:
            True if motion succeeded, False otherwise
        """
        return self.move_to_named_pose('stowed')

    def is_moving(self) -> bool:
        """Check if arm is currently in motion."""
        with self._state_lock:
            return self.state == ArmState.MOVING

    def get_state(self) -> ArmState:
        """Get current arm state."""
        with self._state_lock:
            return self.state


# =============================================================================
# Standalone testing
# =============================================================================
if __name__ == '__main__':
    import sys

    rclpy.init()

    class TestNode(Node):
        def __init__(self):
            super().__init__('arm_interface_test')

    node = TestNode()

    # Get config path from args
    config_path = sys.argv[1] if len(sys.argv) > 1 else None

    arm = ArmInterface(node, poses_config_path=config_path)

    print("Waiting for action servers...")
    if arm.wait_for_action_servers():
        print("Moving to home position...")
        arm.move_to_named_pose('home')
    else:
        print("Action servers not available")

    node.destroy_node()
    rclpy.shutdown()
