#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

class ArduinoNode(Node):
    def __init__(self):
        super().__init__('arduino_node')
        
        # Replace with your Arduino port
        self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
        self.get_logger().info("Connected to Arduino")

        # Publisher: publish Arduino responses
        self.pub = self.create_publisher(String, 'chatter', 10)
        
        # Subscriber: receive commands to send to Arduino
        self.sub = self.create_subscription(String, 'arduino_cmd', self.send_command, 10)
        
        # Timer to read Arduino output
        self.timer = self.create_timer(0.1, self.read_serial)  # 10 Hz

    def read_serial(self):
        while self.ser.in_waiting:
            line = self.ser.readline().decode('utf-8').strip()
            if line:
                msg = String()
                msg.data = line
                self.pub.publish(msg)
                self.get_logger().info(f"Received from Arduino: {line}")

    def send_command(self, msg):
        # Append newline for Arduino readStringUntil('\n')
        command = msg.data.strip() + '\n'
        self.ser.write(command.encode('utf-8'))
        self.get_logger().info(f"Sent to Arduino: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = ArduinoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
