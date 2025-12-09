#!/usr/bin/env python3

"""
ROS2 ↔ Arduino Servo Bridge (MoveIt2 Ready)
- Publishes /joint_states for RViz + MoveIt2
- Subscribes to MoveIt2 joint commands
- Handles Arduino serial protocol:
      SERVO,<i>,<angle>
      SET_ALL_SERVOS,<6 values>
      SERVO_POS,<6 values>
      HEARTBEAT,<t>
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import serial
import threading
import time

class ServoBridge(Node):

    def __init__(self):
        super().__init__('servo_bridge')

        # PARAMETERS
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)

        port = self.get_parameter('serial_port').get_parameter_value().string_value
        baud = self.get_parameter('baud_rate').get_parameter_value().integer_value

        # JOINT STATE MESSAGE
        self.joint_state = JointState()
        self.joint_state.name = [
            "joint_1", "joint_2", "joint_3",
            "joint_4", "gripper_base_joint", "left_gear_joint"
        ]
        self.joint_state.position = [0.0] * 6

        # PUBLISHERS
        self.pub_joint_state = self.create_publisher(JointState, "/joint_states", 10)
        self.pub_ack = self.create_publisher(String, "/arduino_ack", 10)
        self.pub_heartbeat = self.create_publisher(String, "/arduino_heartbeat", 10)

        # SUBSCRIBERS (MoveIt2 / terminal commands)
        self.sub_commands = self.create_subscription(
            String,
            "/arduino_cmd",
            self.arduino_command_callback,
            10
        )

        # SERIAL SETUP
        try:
            self.serial = serial.Serial(port, baud, timeout=1)
            time.sleep(2)
        except Exception as e:
            self.get_logger().error(f"Serial open failed: {e}")
            self.serial = None

        # SERIAL THREAD
        self.running = True
        self.thread = threading.Thread(target=self.serial_loop)
        self.thread.daemon = True
        self.thread.start()

        # TIMER FOR JOINT STATE REPUBLISH
        self.timer = self.create_timer(0.02, self.publish_joint_states)

        self.get_logger().info("MoveIt2 Servo Bridge Ready")

    # ---------------------------------------------------------
    # ROS2 → ARDUINO COMMAND HANDLER
    # ---------------------------------------------------------
    def arduino_command_callback(self, msg):
        """Send string commands directly to Arduino"""
        text = msg.data.strip()
        if self.serial:
            self.serial.write((text + "\n").encode())
            self.get_logger().info(f"Sent to Arduino: {text}")

    # ---------------------------------------------------------
    # SERIAL THREAD: ARDUINO → ROS2
    # ---------------------------------------------------------
    def serial_loop(self):
        while self.running and self.serial:
            try:
                line = self.serial.readline().decode().strip()
                if not line:
                    continue
                # Servo positions
                if line.startswith("SERVO_POS"):
                    parts = line.split(",")
                    if len(parts) == 7:
                        servo_angles = [float(x) for x in parts[1:7]]
                        self.update_joint_states(servo_angles)

                # Heartbeat
                elif line.startswith("HEARTBEAT"):
                    hb_msg = String()
                    hb_msg.data = line
                    self.pub_heartbeat.publish(hb_msg)

                # Command acknowledgments
                elif line.startswith("OK") or line.startswith("ERROR"):
                    ack_msg = String()
                    ack_msg.data = line
                    self.pub_ack.publish(ack_msg)

            except Exception as e:
                self.get_logger().error(f"Serial error: {e}")

    # ---------------------------------------------------------
    def update_joint_states(self, servo_angles):
        """Convert Arduino servo degrees → MoveIt2 joint radians"""
        out = []
        for i, deg in enumerate(servo_angles):
            if i < 4:
                angle_deg = (deg * 2.0) - 180.0
            elif i == 4:
                angle_deg = deg - 90.0
            elif i == 5:
                angle_deg = deg - 90.0
            out.append(angle_deg * 3.14159 / 180.0)
        self.joint_state.position = out

    # ---------------------------------------------------------
    # PUBLISH JOINT STATES
    # ---------------------------------------------------------
    def publish_joint_states(self):
        self.joint_state.header.stamp = self.get_clock().now().to_msg()
        self.pub_joint_state.publish(self.joint_state)

    # ---------------------------------------------------------
    def destroy_node(self):
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
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
