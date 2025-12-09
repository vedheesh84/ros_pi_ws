#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
import serial, threading, time, errno, signal, atexit

PRIMARY_PORT  = "/dev/serial/by-id/usb-Arduino__www.arduino.cc__0042_5573232353035180F1C1-if00"
FALLBACK_PORT = "/dev/ttyACM1"

WHEEL_NAMES = ["wheel_lf", "wheel_rf", "wheel_lb", "wheel_rb"]

class FourWheelBridge(Node):
    def __init__(self):
        super().__init__('arduino_bridge')

        # ---------- Parameters ----------
        self.declare_parameter('port', PRIMARY_PORT)
        self.declare_parameter('baud_rate', 57600)
        self.declare_parameter('wheel_radius', 0.05)
        self.declare_parameter('encoder_resolution', 360)
        self.declare_parameter('cmd_timeout', 0.30)
        self.declare_parameter('send_rate', 30.0)
        self.declare_parameter('joint_pub_rate', 30.0)

        self.port     = self.get_parameter('port').value
        self.baud     = int(self.get_parameter('baud_rate').value)
        self.radius   = float(self.get_parameter('wheel_radius').value)
        self.enc_res  = int(self.get_parameter('encoder_resolution').value)
        self.timeout  = float(self.get_parameter('cmd_timeout').value)
        self.dt_cmd   = 1.0 / float(self.get_parameter('send_rate').value)
        self.dt_joint = 1.0 / float(self.get_parameter('joint_pub_rate').value)

        # ---------- Serial retry ----------
        self.ser = self._open_serial_with_retry(self.port) or \
                    self._open_serial_with_retry(FALLBACK_PORT, note="fallback")
        if not self.ser:
            raise RuntimeError("Cannot open Arduino serial port.")

        time.sleep(2)
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()

        self.get_logger().info(f"Connected to Arduino: {self.port}")

        # ---------- Internal state ----------
        self.lock = threading.Lock()
        self.last_cmd_time = self.get_clock().now()

        # For 4 wheels: LF, RF, LB, RB
        self.enc_counts = [0, 0, 0, 0]
        self.wheel_pos  = [0.0, 0.0, 0.0, 0.0]
        self.wheel_vel  = [0.0, 0.0, 0.0, 0.0]

        # ---------- ROS I/O ----------
        self.cmd_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_cb, 10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)

        self.timer_cmd   = self.create_timer(self.dt_cmd, self.send_loop)
        self.timer_joint = self.create_timer(self.dt_joint, self.publish_joint_states)

        # Start thread to read encoder feedback
        self.reader_thread = threading.Thread(target=self.serial_reader_loop, daemon=True)
        self.reader_thread.start()

        self.get_logger().info("Bridge running with encoder support.")

    # -------------------------------------------------------
    # SERIAL OPEN
    # -------------------------------------------------------
    def _open_serial_with_retry(self, port, note="primary", retries=5):
        for i in range(retries):
            try:
                ser = serial.Serial(port, self.baud, timeout=0.05, exclusive=True)
                self.port = port
                return ser
            except Exception as e:
                self.get_logger().warn(f"Open {note} port failed: {e}")
                time.sleep(0.4 * (i+1))
        return None

    # -------------------------------------------------------
    # CMD_VEL CALLBACK
    # -------------------------------------------------------
    def cmd_vel_cb(self, msg: Twist):
        with self.lock:
            self.last_linear  = msg.linear.x
            self.last_angular = msg.angular.z
            self.last_cmd_time = self.get_clock().now()

    # -------------------------------------------------------
    # SEND VELOCITY COMMANDS
    # -------------------------------------------------------
    def send_loop(self):
        with self.lock:
            age = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
            if age > self.timeout:
                self._send("STOP\n")
                return

            line = f"VEL,{self.last_linear:.4f},{self.last_angular:.4f}\n"
            self._send(line)

    def _send(self, s):
        try:
            self.ser.write(s.encode())
            self.ser.flush()
        except Exception as e:
            self.get_logger().error(f"Write error: {e}")

    # -------------------------------------------------------
    # READ FROM ARDUINO (ENCODERS)
    # -------------------------------------------------------
    def serial_reader_loop(self):
        while True:
            try:
                raw = self.ser.readline().decode().strip()
                if raw.startswith("ENC,"):
                    # Example from Arduino: ENC,123,456,-78,910
                    parts = raw.split(",")
                    if len(parts) == 5:
                        counts = list(map(int, parts[1:]))
                        with self.lock:
                            self._update_encoder_states(counts)
            except Exception:
                pass

    # -------------------------------------------------------
    # UPDATE WHEEL POSITIONS AND VELOCITIES
    # -------------------------------------------------------
    def _update_encoder_states(self, new_counts):
        now = self.get_clock().now().nanoseconds / 1e9

        for i in range(4):
            diff = new_counts[i] - self.enc_counts[i]
            self.enc_counts[i] = new_counts[i]

            # radians per encoder tick = 2π / resolution
            rad = diff * (2 * 3.1415926535 / self.enc_res)

            self.wheel_pos[i] += rad
            self.wheel_vel[i] = rad / self.dt_joint

    # -------------------------------------------------------
    # PUBLISH JOINT STATES
    # -------------------------------------------------------
    def publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = WHEEL_NAMES
        msg.position = self.wheel_pos
        msg.velocity = self.wheel_vel
        msg.effort = []
        self.joint_pub.publish(msg)

    # -------------------------------------------------------
    def destroy_node(self):
        try: self._send("STOP\n")
        except: pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FourWheelBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
