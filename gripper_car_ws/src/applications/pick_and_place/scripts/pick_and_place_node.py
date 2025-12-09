#!/usr/bin/env python3
"""
Pick and Place State Machine Node
==================================
Main orchestrator for the pick-and-place system.

This node implements a state machine that coordinates:
- Navigation to pickup/dropoff locations
- Arm movements for picking and placing objects
- QR code scanning to determine destinations
- Error handling and recovery

State Machine:
    IDLE → GO_PICKUP → PICK_READY → PICKING → SCAN_POSE → SCANNING
         → GO_DROPOFF → PLACING → RETURN → (loop back to GO_PICKUP)

Topics:
    Subscribed:
        /pick_and_place/command (std_msgs/String): Commands (start/stop/pause)

    Published:
        /pick_and_place/state (std_msgs/String): Current state name
        /pick_and_place/status (std_msgs/String): Detailed status messages

Services Called:
    /qr_scanner/scan (std_srvs/Trigger): Scan for QR code

Usage:
    # Start the robot system first
    ros2 launch mobile_manipulator_bringup bringup.launch.py \\
        use_moveit:=true use_nav:=true use_sensors:=true

    # Launch pick-and-place node
    ros2 launch pick_and_place pick_and_place.launch.py

    # Send start command
    ros2 topic pub /pick_and_place/command std_msgs/String "data: 'start'" --once

    # Monitor state
    ros2 topic echo /pick_and_place/state
"""

import os
import time
import yaml
import threading
from enum import Enum, auto
from typing import Optional, Dict, Any
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import String
from std_srvs.srv import Trigger
from ament_index_python.packages import get_package_share_directory

# Import local modules (will be in same install location)
from location_manager import LocationManager, LocationCoordinates
from arm_interface import ArmInterface, ArmConfig
from nav_interface import NavInterface, NavConfig


class State(Enum):
    """
    Pick-and-place state machine states.

    Each state represents a phase in the pick-and-place cycle:
    - IDLE: Waiting for start command
    - GO_PICKUP: Navigating to pickup location
    - PICK_READY: Preparing arm for picking
    - PICKING: Executing pick sequence
    - SCAN_POSE: Moving arm to show QR to camera
    - SCANNING: Scanning QR code
    - GO_DROPOFF: Navigating to destination
    - PLACING: Executing place sequence
    - RETURN: Returning to pickup location
    - ERROR: Error state requiring intervention
    - PAUSED: Temporarily paused
    """
    IDLE = auto()
    GO_PICKUP = auto()
    PICK_READY = auto()
    PICKING = auto()
    SCAN_POSE = auto()
    SCANNING = auto()
    GO_DROPOFF = auto()
    PLACING = auto()
    RETURN = auto()
    ERROR = auto()
    PAUSED = auto()


@dataclass
class SystemConfig:
    """
    Complete system configuration loaded from pick_and_place.yaml.

    Contains all configurable parameters for timeouts, retries,
    and behavior settings.
    """
    continuous_mode: bool = True

    # Timeouts (seconds)
    navigation_timeout: float = 120.0
    arm_motion_timeout: float = 30.0
    qr_scan_timeout: float = 10.0
    gripper_timeout: float = 5.0

    # Retry counts
    navigation_retries: int = 3
    arm_motion_retries: int = 2
    pick_attempt_retries: int = 2
    qr_scan_retries: int = 3

    # Behavior
    use_default_on_unknown_qr: bool = True
    return_on_scan_failure: bool = True
    state_transition_delay: float = 0.5
    gripper_delay: float = 0.3

    @classmethod
    def from_yaml(cls, config_path: str) -> 'SystemConfig':
        """Load configuration from YAML file."""
        with open(config_path, 'r') as f:
            data = yaml.safe_load(f)

        cfg = data.get('pick_and_place', {})
        timeouts = cfg.get('timeouts', {})
        retries = cfg.get('retries', {})
        behavior = cfg.get('behavior', {})

        return cls(
            continuous_mode=cfg.get('continuous_mode', True),
            navigation_timeout=timeouts.get('navigation', 120.0),
            arm_motion_timeout=timeouts.get('arm_motion', 30.0),
            qr_scan_timeout=timeouts.get('qr_scan', 10.0),
            gripper_timeout=timeouts.get('gripper_action', 5.0),
            navigation_retries=retries.get('navigation', 3),
            arm_motion_retries=retries.get('arm_motion', 2),
            pick_attempt_retries=retries.get('pick_attempt', 2),
            qr_scan_retries=retries.get('qr_scan', 3),
            use_default_on_unknown_qr=behavior.get('use_default_on_unknown_qr', True),
            return_on_scan_failure=behavior.get('return_on_scan_failure', True),
            state_transition_delay=behavior.get('state_transition_delay', 0.5),
            gripper_delay=behavior.get('gripper_delay', 0.3),
        )


class PickAndPlaceNode(Node):
    """
    Main pick-and-place state machine node.

    Coordinates arm control, navigation, and QR scanning to implement
    a complete pick-and-place workflow. Objects with QR codes are picked
    up, scanned to determine destination, delivered, and the robot returns
    for the next cycle.
    """

    def __init__(self):
        super().__init__('pick_and_place_node')

        # Callback group for concurrent operations
        self.callback_group = ReentrantCallbackGroup()

        # Get package paths
        try:
            pkg_share = get_package_share_directory('pick_and_place')
            config_dir = os.path.join(pkg_share, 'config')
        except Exception:
            # Fallback for development
            config_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
            config_dir = os.path.join(config_dir, 'config')

        # Load configurations
        self._load_configs(config_dir)

        # State machine
        self._state = State.IDLE
        self._state_lock = threading.Lock()
        self._running = False
        self._paused = False

        # Current operation data
        self._current_destination: Optional[LocationCoordinates] = None
        self._retry_count = 0
        self._cycle_count = 0

        # Initialize interfaces
        self._init_interfaces(config_dir)

        # Create publishers
        self.state_pub = self.create_publisher(String, '/pick_and_place/state', 10)
        self.status_pub = self.create_publisher(String, '/pick_and_place/status', 10)

        # Create command subscriber
        self.command_sub = self.create_subscription(
            String,
            '/pick_and_place/command',
            self._command_callback,
            10,
            callback_group=self.callback_group
        )

        # Create QR scanner service client
        self.qr_scan_client = self.create_client(
            Trigger,
            '/qr_scanner/scan',
            callback_group=self.callback_group
        )

        # State machine timer (runs the main loop)
        self.state_timer = self.create_timer(
            0.1,  # 10 Hz
            self._state_machine_tick,
            callback_group=self.callback_group
        )

        # Status publishing timer
        self.status_timer = self.create_timer(1.0, self._publish_state)

        self.get_logger().info(
            f"Pick and Place node initialized\n"
            f"  Continuous mode: {self.config.continuous_mode}\n"
            f"  Locations: {self.location_manager.list_locations()}\n"
            f"  Send 'start' to /pick_and_place/command to begin"
        )

    def _load_configs(self, config_dir: str) -> None:
        """Load all configuration files."""
        # System config
        config_path = os.path.join(config_dir, 'pick_and_place.yaml')
        if os.path.exists(config_path):
            self.config = SystemConfig.from_yaml(config_path)
            self.get_logger().info(f"Loaded system config from {config_path}")
        else:
            self.config = SystemConfig()
            self.get_logger().warning("Using default system config")

        # Location manager
        locations_path = os.path.join(config_dir, 'locations.yaml')
        if os.path.exists(locations_path):
            self.location_manager = LocationManager(locations_path)
            self.get_logger().info(f"Loaded locations from {locations_path}")
        else:
            raise FileNotFoundError(f"Locations config not found: {locations_path}")

        # Arm poses path (passed to arm interface)
        self.arm_poses_path = os.path.join(config_dir, 'arm_poses.yaml')

    def _init_interfaces(self, config_dir: str) -> None:
        """Initialize arm and navigation interfaces."""
        # Arm interface
        arm_config = ArmConfig(
            arm_group="arm",
            gripper_group="gripper",
            velocity_scaling=0.5,
            acceleration_scaling=0.5,
            motion_timeout=self.config.arm_motion_timeout
        )
        self.arm = ArmInterface(
            self,
            poses_config_path=self.arm_poses_path if os.path.exists(self.arm_poses_path) else None,
            arm_config=arm_config
        )

        # Navigation interface
        nav_config = NavConfig(
            action_server="/navigate_to_pose",
            goal_frame="map",
            robot_base_frame="body_link",
            default_timeout=self.config.navigation_timeout
        )
        self.nav = NavInterface(self, config=nav_config)

    def _command_callback(self, msg: String) -> None:
        """
        Handle incoming commands.

        Supported commands:
        - start: Begin pick-and-place cycle
        - stop: Stop and return to IDLE
        - pause: Pause current operation
        - resume: Resume from pause
        - reset: Reset to IDLE state

        Args:
            msg: Command string message
        """
        command = msg.data.lower().strip()
        self.get_logger().info(f"Received command: {command}")

        if command == 'start':
            self._start_operation()
        elif command == 'stop':
            self._stop_operation()
        elif command == 'pause':
            self._pause_operation()
        elif command == 'resume':
            self._resume_operation()
        elif command == 'reset':
            self._reset_operation()
        else:
            self.get_logger().warning(f"Unknown command: {command}")

    def _start_operation(self) -> None:
        """Start the pick-and-place cycle."""
        if self._running:
            self.get_logger().warning("Already running")
            return

        self._running = True
        self._paused = False
        self._set_state(State.GO_PICKUP)
        self._publish_status("Starting pick-and-place operation")

    def _stop_operation(self) -> None:
        """Stop the operation and return to IDLE."""
        self._running = False
        self._paused = False

        # Cancel any ongoing navigation
        if self.nav.is_navigating():
            self.nav.cancel_navigation()

        # Stow arm
        self.arm.stow()

        self._set_state(State.IDLE)
        self._publish_status("Operation stopped")

    def _pause_operation(self) -> None:
        """Pause the current operation."""
        if not self._running:
            return

        self._paused = True
        self._set_state(State.PAUSED)
        self._publish_status("Operation paused")

    def _resume_operation(self) -> None:
        """Resume from pause."""
        if not self._paused:
            return

        self._paused = False
        # Return to previous operational state
        self._set_state(State.GO_PICKUP)
        self._publish_status("Operation resumed")

    def _reset_operation(self) -> None:
        """Reset to initial state."""
        self._running = False
        self._paused = False
        self._current_destination = None
        self._retry_count = 0
        self._cycle_count = 0
        self._set_state(State.IDLE)
        self._publish_status("System reset")

    def _set_state(self, new_state: State) -> None:
        """Thread-safe state update."""
        with self._state_lock:
            old_state = self._state
            self._state = new_state

        if old_state != new_state:
            self.get_logger().info(f"State: {old_state.name} → {new_state.name}")
            self._publish_state()

    def _get_state(self) -> State:
        """Thread-safe state read."""
        with self._state_lock:
            return self._state

    def _publish_state(self) -> None:
        """Publish current state."""
        msg = String()
        msg.data = self._get_state().name
        self.state_pub.publish(msg)

    def _publish_status(self, status: str) -> None:
        """Publish status message."""
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)
        self.get_logger().info(f"Status: {status}")

    def _state_machine_tick(self) -> None:
        """
        Main state machine execution loop.

        Called periodically by timer. Checks current state and
        executes appropriate action.
        """
        if not self._running or self._paused:
            return

        current_state = self._get_state()

        try:
            if current_state == State.IDLE:
                pass  # Waiting for start command

            elif current_state == State.GO_PICKUP:
                self._execute_go_pickup()

            elif current_state == State.PICK_READY:
                self._execute_pick_ready()

            elif current_state == State.PICKING:
                self._execute_picking()

            elif current_state == State.SCAN_POSE:
                self._execute_scan_pose()

            elif current_state == State.SCANNING:
                self._execute_scanning()

            elif current_state == State.GO_DROPOFF:
                self._execute_go_dropoff()

            elif current_state == State.PLACING:
                self._execute_placing()

            elif current_state == State.RETURN:
                self._execute_return()

            elif current_state == State.ERROR:
                self._handle_error()

        except Exception as e:
            self.get_logger().error(f"State machine error: {e}")
            self._set_state(State.ERROR)

    # =========================================================================
    # State Execution Methods
    # =========================================================================

    def _execute_go_pickup(self) -> None:
        """Navigate to pickup location."""
        self._publish_status("Navigating to pickup location")

        pickup = self.location_manager.get_pickup_location()

        if self.nav.navigate_to_location(
            pickup,
            timeout=self.config.navigation_timeout
        ):
            self._retry_count = 0
            time.sleep(self.config.state_transition_delay)
            self._set_state(State.PICK_READY)
        else:
            self._retry_count += 1
            if self._retry_count >= self.config.navigation_retries:
                self._publish_status("Navigation to pickup failed after retries")
                self._set_state(State.ERROR)
            else:
                self._publish_status(f"Navigation failed, retry {self._retry_count}")
                # Stay in GO_PICKUP to retry

    def _execute_pick_ready(self) -> None:
        """Move arm to ready position for picking."""
        self._publish_status("Moving arm to ready position")

        # Move to approach pose
        if self.arm.move_to_config_pose('approach_pose'):
            # Open gripper
            time.sleep(self.config.gripper_delay)
            if self.arm.open_gripper():
                self._retry_count = 0
                time.sleep(self.config.state_transition_delay)
                self._set_state(State.PICKING)
                return

        self._retry_count += 1
        if self._retry_count >= self.config.arm_motion_retries:
            self._publish_status("Arm motion failed after retries")
            self._set_state(State.ERROR)

    def _execute_picking(self) -> None:
        """Execute pick sequence: approach → grasp → lift."""
        self._publish_status("Picking object")

        # Move to pick position
        if not self.arm.move_to_config_pose('pick_pose'):
            self._handle_pick_failure()
            return

        # Close gripper to grasp
        time.sleep(self.config.gripper_delay)
        if not self.arm.close_gripper():
            self._handle_pick_failure()
            return

        # Lift object
        time.sleep(self.config.gripper_delay)
        if not self.arm.move_to_config_pose('lift_pose'):
            self._handle_pick_failure()
            return

        self._retry_count = 0
        self._publish_status("Object picked successfully")
        time.sleep(self.config.state_transition_delay)
        self._set_state(State.SCAN_POSE)

    def _handle_pick_failure(self) -> None:
        """Handle failure during pick sequence."""
        self._retry_count += 1
        if self._retry_count >= self.config.pick_attempt_retries:
            self._publish_status("Pick sequence failed after retries")
            self.arm.open_gripper()  # Release if anything was grabbed
            self.arm.stow()
            self._set_state(State.ERROR)
        else:
            self._publish_status(f"Pick failed, retry {self._retry_count}")
            self.arm.move_to_config_pose('approach_pose')
            self._set_state(State.PICK_READY)

    def _execute_scan_pose(self) -> None:
        """Move arm to show QR code to camera."""
        self._publish_status("Moving to scan pose")

        if self.arm.move_to_config_pose('scan_pose'):
            self._retry_count = 0
            time.sleep(self.config.state_transition_delay)
            self._set_state(State.SCANNING)
        else:
            self._retry_count += 1
            if self._retry_count >= self.config.arm_motion_retries:
                # Can't scan, use default location
                self._publish_status("Scan pose failed, using default location")
                self._current_destination = self.location_manager.get_default_location()
                self._set_state(State.GO_DROPOFF)

    def _execute_scanning(self) -> None:
        """Scan QR code to determine destination."""
        self._publish_status("Scanning QR code")

        # Wait for QR scanner service
        if not self.qr_scan_client.wait_for_service(timeout_sec=5.0):
            self._publish_status("QR scanner service not available")
            self._handle_scan_failure()
            return

        # Call scan service
        request = Trigger.Request()
        future = self.qr_scan_client.call_async(request)

        # Wait for result
        rclpy.spin_until_future_complete(
            self, future, timeout_sec=self.config.qr_scan_timeout
        )

        if future.result() is None:
            self._publish_status("QR scan timed out")
            self._handle_scan_failure()
            return

        response = future.result()

        if response.success:
            qr_content = response.message
            self._publish_status(f"QR code detected: '{qr_content}'")

            # Look up location
            location = self.location_manager.get_location(qr_content)

            if location:
                self._current_destination = location
                self._retry_count = 0
                time.sleep(self.config.state_transition_delay)
                self._set_state(State.GO_DROPOFF)
            elif self.config.use_default_on_unknown_qr:
                self._publish_status(f"Unknown location '{qr_content}', using default")
                self._current_destination = self.location_manager.get_default_location()
                self._set_state(State.GO_DROPOFF)
            else:
                self._handle_scan_failure()
        else:
            self._publish_status("No QR code detected")
            self._handle_scan_failure()

    def _handle_scan_failure(self) -> None:
        """Handle failure to scan QR code."""
        self._retry_count += 1

        if self._retry_count >= self.config.qr_scan_retries:
            if self.config.return_on_scan_failure:
                self._publish_status("Scan failed, returning object to pickup")
                # Place object back at pickup
                self._current_destination = self.location_manager.get_pickup_location()
                self._set_state(State.PLACING)
            elif self.config.use_default_on_unknown_qr:
                self._publish_status("Scan failed, using default location")
                self._current_destination = self.location_manager.get_default_location()
                self._set_state(State.GO_DROPOFF)
            else:
                self._set_state(State.ERROR)
        # else: stay in SCANNING to retry

    def _execute_go_dropoff(self) -> None:
        """Navigate to dropoff location."""
        if not self._current_destination:
            self._publish_status("No destination set")
            self._set_state(State.ERROR)
            return

        self._publish_status(
            f"Navigating to {self._current_destination.name}"
        )

        # First stow arm for safe navigation
        self.arm.move_to_named_pose('stowed')

        if self.nav.navigate_to_location(
            self._current_destination,
            timeout=self.config.navigation_timeout
        ):
            self._retry_count = 0
            time.sleep(self.config.state_transition_delay)
            self._set_state(State.PLACING)
        else:
            self._retry_count += 1
            if self._retry_count >= self.config.navigation_retries:
                self._publish_status("Navigation to dropoff failed")
                self._set_state(State.ERROR)

    def _execute_placing(self) -> None:
        """Execute place sequence: approach → release → retract."""
        self._publish_status("Placing object")

        # Move to place approach
        if not self.arm.move_to_config_pose('place_approach_pose'):
            self._set_state(State.ERROR)
            return

        # Move to place position
        if not self.arm.move_to_config_pose('place_pose'):
            self._set_state(State.ERROR)
            return

        # Open gripper to release
        time.sleep(self.config.gripper_delay)
        self.arm.open_gripper()

        # Retract
        time.sleep(self.config.gripper_delay)
        self.arm.move_to_config_pose('lift_pose')

        # Stow arm
        self.arm.stow()

        self._cycle_count += 1
        self._current_destination = None
        self._publish_status(f"Cycle {self._cycle_count} complete")

        time.sleep(self.config.state_transition_delay)

        if self.config.continuous_mode:
            self._set_state(State.RETURN)
        else:
            self._publish_status("Single cycle complete, stopping")
            self._running = False
            self._set_state(State.IDLE)

    def _execute_return(self) -> None:
        """Return to pickup location for next cycle."""
        self._publish_status("Returning to pickup for next cycle")

        pickup = self.location_manager.get_pickup_location()

        if self.nav.navigate_to_location(
            pickup,
            timeout=self.config.navigation_timeout
        ):
            self._retry_count = 0
            time.sleep(self.config.state_transition_delay)
            self._set_state(State.PICK_READY)  # Start next pick
        else:
            self._retry_count += 1
            if self._retry_count >= self.config.navigation_retries:
                self._publish_status("Return navigation failed")
                self._set_state(State.ERROR)

    def _handle_error(self) -> None:
        """Handle error state."""
        self._publish_status("ERROR state - send 'reset' to recover")
        self._running = False

        # Try to stow arm safely
        try:
            self.arm.open_gripper()
            self.arm.stow()
        except Exception:
            pass


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)

    node = PickAndPlaceNode()

    # Wait for required services
    node.get_logger().info("Waiting for arm action servers...")
    node.arm.wait_for_action_servers(timeout=30.0)

    node.get_logger().info("Waiting for Nav2...")
    node.nav.wait_for_nav2(timeout=30.0)

    node.get_logger().info("Pick and Place node ready!")

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
