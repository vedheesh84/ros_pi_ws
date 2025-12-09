# Arduino Uno with PCA9685 Integration for ROS2 MoveIt2

This document describes how to integrate an Arduino Uno with PCA9685 servo driver board with your ROS2 MoveIt2 mobile arm manipulator for real-time control and sensor feedback.

## Overview

The Arduino PCA9685 integration provides:
- **Real-time servo control** for robot joints via PCA9685 driver
- **Smooth servo movement** with interpolation and acceleration control
- **Analog sensor reading** (potentiometers, force sensors)
- **Digital sensor reading** (limit switches, emergency stop)
- **PWM output control** for LEDs, motors, etc.
- **Safety monitoring** with emergency stop functionality
- **Bidirectional communication** between ROS2 and Arduino
- **I2C-based servo control** for better precision and stability

## Hardware Requirements

### Arduino Uno with PCA9685 Setup
- Arduino Uno or compatible board
- **PCA9685 Servo Driver Board** (16-channel PWM driver)
- 6x Servo motors (SG90 or similar)
- 6x Analog sensors (potentiometers, force sensors, etc.)
- 6x Digital sensors (limit switches, buttons, etc.)
- Breadboard and jumper wires
- Power supply for servos and PCA9685
- I2C connections (SDA, SCL)

### Pin Connections

#### PCA9685 Servo Driver Channels
- Servo 0 (joint_1): PCA9685 Channel 0
- Servo 1 (joint_2): PCA9685 Channel 1
- Servo 2 (joint_3): PCA9685 Channel 2
- Servo 3 (joint_4): PCA9685 Channel 4
- Servo 4 (gripper_base_joint): PCA9685 Channel 5
- Servo 5 (left_gear_joint): PCA9685 Channel 6

#### I2C Connections
- PCA9685 VCC → 5V or 3.3V
- PCA9685 GND → GND
- PCA9685 SDA → Arduino A4 (or SDA pin)
- PCA9685 SCL → Arduino A5 (or SCL pin)
- PCA9685 Address → 0x40 (default)

#### Analog Sensors
- Potentiometer 1: Pin A0
- Potentiometer 2: Pin A1
- Potentiometer 3: Pin A2
- Potentiometer 4: Pin A3
- Force sensor 1: Pin A4
- Force sensor 2: Pin A5

#### Digital Sensors
- Limit switch 1: Pin 2
- Limit switch 2: Pin 4
- Limit switch 3: Pin 7
- Limit switch 4: Pin 8
- Emergency stop: Pin 12
- Gripper sensor: Pin 13

## Software Setup

### 1. Install Arduino IDE
```bash
# Install Arduino IDE
sudo apt install arduino

# Add user to dialout group for serial access
sudo usermod -a -G dialout $USER
# Log out and back in for changes to take effect
```

### 2. Upload Arduino Sketch
1. Open Arduino IDE
2. Load the sketch: `arduino/mobile_arm_arduino_ros2.ino`
3. Select your Arduino board and port
4. Upload the sketch

### 3. Install Python Dependencies
```bash
# Install required Python packages
pip3 install pyserial

# Or install via apt
sudo apt install python3-serial
```

### 4. Install ROS2 Dependencies
```bash
# Install required ROS2 packages
sudo apt install ros-humble-serial-driver ros-humble-joint-state-publisher
```

## Usage

### 1. Arduino Bridge Only
Launch just the Arduino communication bridge:
```bash
ros2 launch mobile_arm_manipulator_config arduino_bridge.launch.py
```

### 2. Complete Arduino Real-Time Demo
Launch the full system with Arduino and MoveIt2:
```bash
ros2 launch mobile_arm_manipulator_config arduino_realtime_demo.launch.py
```

### 3. With Custom Serial Port
Specify a different serial port:
```bash
ros2 launch mobile_arm_manipulator_config arduino_bridge.launch.py serial_port:=/dev/ttyACM0
```

### 4. With Fake Hardware (Testing)
Test without Arduino hardware:
```bash
ros2 launch mobile_arm_manipulator_config arduino_realtime_demo.launch.py use_fake_hardware:=true
```

## Launch Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `serial_port` | `/dev/ttyUSB0` | Serial port for Arduino |
| `baud_rate` | `115200` | Serial communication baud rate |
| `timeout` | `1.0` | Serial timeout in seconds |
| `publish_rate` | `50.0` | Sensor data publish rate (Hz) |
| `emergency_stop_enabled` | `true` | Enable emergency stop functionality |
| `use_fake_hardware` | `false` | Use fake hardware instead of Arduino |
| `launch_rviz` | `true` | Launch RViz2 visualization |

## Communication Protocol

### Commands from ROS2 to Arduino
```
SERVO,<index>,<angle>          # Set servo position (0-5, 0-180 degrees)
DIGITAL,<pin>,<value>          # Set digital pin (HIGH=1, LOW=0)
PWM,<pin>,<value>              # Set PWM pin (0-255)
SET_ALL_SERVOS,<a1>,<a2>,<a3>,<a4>,<a5>,<a6>  # Set all servos at once
GET_STATUS                     # Request status information
RESET_EMERGENCY               # Reset emergency stop
```

### Data from Arduino to ROS2
```
ANALOG,<val1>,<val2>,<val3>,<val4>,<val5>,<val6>     # Analog sensor values
DIGITAL,<val1>,<val2>,<val3>,<val4>,<val5>,<val6>    # Digital sensor values
SERVO_POS,<pos1>,<pos2>,<pos3>,<pos4>,<pos5>,<pos6>  # Current servo positions
EMERGENCY_STOP,ACTIVATED       # Emergency stop triggered
HEARTBEAT,<timestamp>          # Keep-alive message
OK,<command>                   # Command acknowledged
ERROR,<message>                # Error occurred
```

## ROS2 Topics

### Published Topics
- `/arduino/joint_states` - Current joint positions and states
- `/arduino/analog_sensors` - Analog sensor readings
- `/arduino/digital_sensors` - Digital sensor readings
- `/arduino/status` - Arduino status information

### Subscribed Topics
- `/arduino/servo_commands` - Servo position commands
- `/arduino/digital_outputs` - Digital output commands
- `/arduino/pwm_commands` - PWM output commands

## Testing

### 1. Test Arduino Communication
```bash
# Test PCA9685 communication
python3 scripts/test_pca9685_integration.py

# Or run the ROS2 node directly
ros2 run mobile_arm_manipulator_config arduino_ros2_bridge.py
```

### 2. Monitor Topics
```bash
# Monitor joint states
ros2 topic echo /arduino/joint_states

# Monitor analog sensors
ros2 topic echo /arduino/analog_sensors

# Monitor digital sensors
ros2 topic echo /arduino/digital_sensors

# Monitor status
ros2 topic echo /arduino/status
```

### 3. Send Test Commands
```bash
# Send servo command
ros2 topic pub /arduino/servo_commands sensor_msgs/msg/JointState "
name: ['joint_1']
position: [90.0]
velocity: [0.0]
effort: [0.0]
"

# Send digital output
ros2 topic pub /arduino/digital_outputs std_msgs/msg/Int32MultiArray "
data: [13, 1]
"
```

## Integration with MoveIt2

The Arduino bridge integrates seamlessly with MoveIt2:

1. **Joint State Publishing**: Arduino joint positions are published to `/joint_states`
2. **Trajectory Execution**: MoveIt2 trajectory commands are sent to Arduino servos
3. **Real-time Control**: High-frequency control loop for smooth motion
4. **Safety Integration**: Emergency stop affects both Arduino and MoveIt2

### MoveIt2 Integration Example
```bash
# Launch complete system
ros2 launch mobile_arm_manipulator_config arduino_realtime_demo.launch.py

# In another terminal, plan and execute motion
ros2 run moveit_commander moveit_commander_cmdline.py
```

## Safety Features

### Emergency Stop
- Hardware emergency stop button on pin 12
- Software emergency stop via ROS2 topics
- Automatic servo disabling on emergency stop
- Status monitoring and reporting

### Safety Limits
- Servo angle constraints (0-180 degrees)
- Communication timeout detection
- Heartbeat monitoring
- Error handling and recovery

## Troubleshooting

### Common Issues

1. **Serial Port Not Found**
   ```bash
   # List available serial ports
   ls /dev/tty*
   
   # Check Arduino connection
   dmesg | grep tty
   
   # Add user to dialout group
   sudo usermod -a -G dialout $USER
   ```

2. **Permission Denied**
   ```bash
   # Fix serial port permissions
   sudo chmod 666 /dev/ttyUSB0
   
   # Or use udev rules for permanent fix
   sudo nano /etc/udev/rules.d/99-arduino.rules
   # Add: SUBSYSTEM=="tty", ATTRS{idVendor}=="2341", MODE="0666"
   ```

3. **Communication Timeout**
   - Check baud rate matches (115200)
   - Verify Arduino sketch is uploaded correctly
   - Check cable connections
   - Monitor serial output in Arduino IDE

4. **Servo Not Moving**
   - Check power supply to servos
   - Verify servo connections
   - Test servos individually
   - Check servo specifications

### Debug Commands
```bash
# Monitor serial communication
screen /dev/ttyUSB0 115200

# Check ROS2 topics
ros2 topic list
ros2 topic info /arduino/joint_states

# Check node status
ros2 node list
ros2 node info /arduino_ros2_bridge

# Monitor system resources
htop
```

## Advanced Configuration

### Custom Joint Mapping
Edit `config/arduino_serial.yaml` to customize joint mappings:
```yaml
joint_mapping:
  ros__parameters:
    arduino_to_robot_joints:
      servo_0: "joint_1"
      servo_1: "joint_2"
      # Add your custom mappings
```

### Custom Sensor Configuration
```yaml
arduino_sensors:
  ros__parameters:
    sensors:
      - name: "custom_sensor"
        topic: "/arduino/custom_sensor"
        msg_type: "sensor_msgs/msg/JointState"
```

### Performance Tuning
- Adjust `publish_rate` for sensor data frequency
- Modify Arduino loop delay for control frequency
- Use real-time kernel for better performance
- Optimize serial communication baud rate

## Examples

### Basic Servo Control
```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class SimpleServoController(Node):
    def __init__(self):
        super().__init__('simple_servo_controller')
        self.pub = self.create_publisher(JointState, '/arduino/servo_commands', 10)
        
    def move_servo(self, joint_name, angle):
        msg = JointState()
        msg.name = [joint_name]
        msg.position = [angle]
        msg.velocity = [0.0]
        msg.effort = [0.0]
        self.pub.publish(msg)
```

### Sensor Monitoring
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray

class SensorMonitor(Node):
    def __init__(self):
        super().__init__('sensor_monitor')
        self.sub = self.create_subscription(
            Int32MultiArray, 
            '/arduino/analog_sensors', 
            self.sensor_callback, 
            10
        )
    
    def sensor_callback(self, msg):
        self.get_logger().info(f'Analog sensors: {msg.data}')
```

This integration provides a robust foundation for Arduino-based robot control with ROS2 MoveIt2, enabling real-time hardware control with comprehensive sensor feedback and safety features.
