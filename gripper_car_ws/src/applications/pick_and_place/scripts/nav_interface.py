#!/usr/bin/env python3
"""
Navigation Interface Module
============================
Nav2 wrapper for autonomous navigation in pick-and-place operations.

This module provides a high-level interface for robot navigation using
the Nav2 stack. It wraps the NavigateToPose action to provide simple
methods for sending navigation goals.

Usage:
    from nav_interface import NavInterface
    from location_manager import LocationCoordinates

    # Initialize (requires Nav2 to be running)
    nav = NavInterface(node)

    # Navigate to coordinates
    location = LocationCoordinates(x=2.0, y=1.0, theta=1.57, name='station_A')
    success = nav.navigate_to_location(location)

    # Or navigate to raw coordinates
    success = nav.navigate_to_pose(x=2.0, y=1.0, theta=1.57)

    # Check navigation status
    if nav.is_navigating():
        nav.cancel_navigation()

Dependencies:
    - nav2_msgs: Nav2 action messages
    - geometry_msgs: Pose messages
    - tf_transformations or transforms3d: Quaternion conversion

Note: Nav2 navigation stack must be running before using this interface.
"""

import math
import threading
from typing import Optional, Callable
from dataclasses import dataclass
from enum import Enum

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Quaternion
from action_msgs.msg import GoalStatus

# Import location manager types
try:
    from location_manager import LocationCoordinates
except ImportError:
    # Define locally if module not in path
    @dataclass
    class LocationCoordinates:
        x: float
        y: float
        theta: float
        name: str = ""


class NavState(Enum):
    """Current state of navigation."""
    IDLE = "idle"
    NAVIGATING = "navigating"
    SUCCEEDED = "succeeded"
    FAILED = "failed"
    CANCELED = "canceled"


@dataclass
class NavConfig:
    """
    Configuration for navigation interface.

    Attributes:
        action_server: Nav2 action server name
        goal_frame: Frame ID for navigation goals
        robot_base_frame: Robot base frame ID
        goal_tolerance_xy: Position tolerance in meters
        goal_tolerance_yaw: Orientation tolerance in radians
        default_timeout: Default navigation timeout in seconds
    """
    action_server: str = "/navigate_to_pose"
    goal_frame: str = "map"
    robot_base_frame: str = "body_link"
    goal_tolerance_xy: float = 0.1
    goal_tolerance_yaw: float = 0.1
    default_timeout: float = 120.0


class NavInterface:
    """
    High-level interface for Nav2 navigation.

    This class wraps the NavigateToPose action client to provide
    simple methods for autonomous navigation. It handles goal
    construction, sending, and result monitoring.
    """

    def __init__(
        self,
        node: Node,
        config: Optional[NavConfig] = None,
        feedback_callback: Optional[Callable] = None
    ):
        """
        Initialize the navigation interface.

        Args:
            node: ROS2 node for creating clients
            config: Navigation configuration (uses defaults if None)
            feedback_callback: Optional callback for navigation feedback
        """
        self.node = node
        self.logger = node.get_logger()
        self.config = config or NavConfig()

        # State tracking
        self.state = NavState.IDLE
        self._state_lock = threading.Lock()

        # Current goal tracking
        self._current_goal_handle: Optional[ClientGoalHandle] = None
        self._goal_lock = threading.Lock()

        # Feedback callback
        self._feedback_callback = feedback_callback

        # Navigation result
        self._last_result = None

        # Create action client
        self.nav_client = ActionClient(
            node,
            NavigateToPose,
            self.config.action_server
        )

        self.logger.info(
            f"Navigation interface initialized:\n"
            f"  Action server: {self.config.action_server}\n"
            f"  Goal frame: {self.config.goal_frame}\n"
            f"  Timeout: {self.config.default_timeout}s"
        )

    def wait_for_nav2(self, timeout: float = 10.0) -> bool:
        """
        Wait for Nav2 action server to become available.

        Args:
            timeout: Maximum time to wait in seconds

        Returns:
            True if server available, False on timeout
        """
        self.logger.info("Waiting for Nav2 action server...")

        if self.nav_client.wait_for_server(timeout_sec=timeout):
            self.logger.info("Nav2 action server ready")
            return True
        else:
            self.logger.error("Nav2 action server not available")
            return False

    @staticmethod
    def yaw_to_quaternion(yaw: float) -> Quaternion:
        """
        Convert yaw angle to quaternion.

        Args:
            yaw: Yaw angle in radians

        Returns:
            Quaternion message representing the orientation
        """
        # Calculate quaternion for rotation around Z axis
        # q = [w, x, y, z] where:
        # w = cos(yaw/2), z = sin(yaw/2), x = y = 0
        q = Quaternion()
        q.w = math.cos(yaw / 2.0)
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2.0)
        return q

    def create_goal_pose(
        self, x: float, y: float, theta: float
    ) -> PoseStamped:
        """
        Create a PoseStamped goal message.

        Args:
            x: X position in goal frame (meters)
            y: Y position in goal frame (meters)
            theta: Orientation as yaw angle (radians)

        Returns:
            PoseStamped message ready for navigation
        """
        pose = PoseStamped()
        pose.header.frame_id = self.config.goal_frame
        pose.header.stamp = self.node.get_clock().now().to_msg()

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0

        pose.pose.orientation = self.yaw_to_quaternion(theta)

        return pose

    def navigate_to_pose(
        self,
        x: float,
        y: float,
        theta: float,
        timeout: Optional[float] = None,
        wait: bool = True
    ) -> bool:
        """
        Navigate robot to specified pose.

        Args:
            x: X position in map frame (meters)
            y: Y position in map frame (meters)
            theta: Orientation as yaw angle (radians)
            timeout: Navigation timeout (uses default if None)
            wait: If True, block until navigation completes

        Returns:
            True if navigation succeeded, False otherwise
        """
        with self._state_lock:
            if self.state == NavState.NAVIGATING:
                self.logger.warning("Already navigating, canceling current goal")
                self.cancel_navigation()

            self.state = NavState.NAVIGATING

        timeout = timeout or self.config.default_timeout

        # Create goal message
        goal = NavigateToPose.Goal()
        goal.pose = self.create_goal_pose(x, y, theta)

        self.logger.info(
            f"Navigating to: x={x:.2f}, y={y:.2f}, theta={theta:.2f}"
        )

        try:
            # Send goal
            send_goal_future = self.nav_client.send_goal_async(
                goal,
                feedback_callback=self._internal_feedback_callback
            )

            rclpy.spin_until_future_complete(
                self.node, send_goal_future, timeout_sec=10.0
            )

            if send_goal_future.result() is None:
                self.logger.error("Failed to send navigation goal")
                self._set_state(NavState.FAILED)
                return False

            goal_handle = send_goal_future.result()

            if not goal_handle.accepted:
                self.logger.error("Navigation goal rejected")
                self._set_state(NavState.FAILED)
                return False

            with self._goal_lock:
                self._current_goal_handle = goal_handle

            self.logger.info("Navigation goal accepted")

            if not wait:
                return True

            # Wait for result
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(
                self.node, result_future, timeout_sec=timeout
            )

            if result_future.result() is None:
                self.logger.error("Navigation timed out")
                self._set_state(NavState.FAILED)
                return False

            result = result_future.result()
            self._last_result = result

            # Check result status
            if result.status == GoalStatus.STATUS_SUCCEEDED:
                self.logger.info("Navigation succeeded")
                self._set_state(NavState.SUCCEEDED)
                return True
            elif result.status == GoalStatus.STATUS_CANCELED:
                self.logger.info("Navigation canceled")
                self._set_state(NavState.CANCELED)
                return False
            else:
                self.logger.error(f"Navigation failed with status: {result.status}")
                self._set_state(NavState.FAILED)
                return False

        except Exception as e:
            self.logger.error(f"Navigation error: {e}")
            self._set_state(NavState.FAILED)
            return False

        finally:
            with self._goal_lock:
                self._current_goal_handle = None

    def navigate_to_location(
        self,
        location: LocationCoordinates,
        timeout: Optional[float] = None,
        wait: bool = True
    ) -> bool:
        """
        Navigate robot to a LocationCoordinates object.

        Args:
            location: Location with x, y, theta coordinates
            timeout: Navigation timeout
            wait: If True, block until navigation completes

        Returns:
            True if navigation succeeded, False otherwise
        """
        self.logger.info(f"Navigating to location: {location.name}")
        return self.navigate_to_pose(
            location.x, location.y, location.theta,
            timeout=timeout, wait=wait
        )

    def cancel_navigation(self) -> bool:
        """
        Cancel current navigation goal.

        Returns:
            True if cancellation was sent, False if no active goal
        """
        with self._goal_lock:
            if self._current_goal_handle is None:
                self.logger.warning("No active navigation goal to cancel")
                return False

            self.logger.info("Canceling navigation")
            cancel_future = self._current_goal_handle.cancel_goal_async()

        rclpy.spin_until_future_complete(
            self.node, cancel_future, timeout_sec=5.0
        )

        self._set_state(NavState.CANCELED)
        return True

    def _internal_feedback_callback(self, feedback_msg) -> None:
        """
        Internal callback for navigation feedback.

        Args:
            feedback_msg: Navigation feedback message
        """
        feedback = feedback_msg.feedback

        # Calculate distance to goal
        current_pose = feedback.current_pose.pose
        # Could add distance calculation here

        # Call user callback if provided
        if self._feedback_callback:
            try:
                self._feedback_callback(feedback)
            except Exception as e:
                self.logger.warning(f"Feedback callback error: {e}")

    def _set_state(self, new_state: NavState) -> None:
        """Thread-safe state update."""
        with self._state_lock:
            self.state = new_state

    def is_navigating(self) -> bool:
        """Check if navigation is in progress."""
        with self._state_lock:
            return self.state == NavState.NAVIGATING

    def get_state(self) -> NavState:
        """Get current navigation state."""
        with self._state_lock:
            return self.state

    def get_last_result(self):
        """Get result of last navigation attempt."""
        return self._last_result


# =============================================================================
# Standalone testing
# =============================================================================
if __name__ == '__main__':
    import sys

    rclpy.init()

    class TestNode(Node):
        def __init__(self):
            super().__init__('nav_interface_test')

    node = TestNode()
    nav = NavInterface(node)

    print("Waiting for Nav2...")
    if nav.wait_for_nav2():
        # Parse coordinates from args
        if len(sys.argv) >= 4:
            x = float(sys.argv[1])
            y = float(sys.argv[2])
            theta = float(sys.argv[3])

            print(f"Navigating to: x={x}, y={y}, theta={theta}")
            success = nav.navigate_to_pose(x, y, theta)
            print(f"Navigation {'succeeded' if success else 'failed'}")
        else:
            print("Usage: python3 nav_interface.py <x> <y> <theta>")
    else:
        print("Nav2 not available")

    node.destroy_node()
    rclpy.shutdown()
