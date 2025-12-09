# ros2_arduino_bridge

ROS 2 Python package that bridges differential-drive motor control on Arduino with ROS 2 topics. Includes a motor bridge node that speaks the simple serial protocol implemented in `arduino/ros_arduino_bridge.ino` and a CLI teleoperation node.

## Features
- Converts `geometry_msgs/Twist` velocity commands into PWM signals for dual motor drivers.
- Publishes encoder tick counts as `std_msgs/Int64MultiArray` and wheel joint states as `sensor_msgs/JointState`.
- Optional direct PWM control through the `motor_cmd` topic or the keyboard teleop node.
- Parameter file (`config/arduino_params.yaml`) for serial port, wheel geometry, and encoder resolution.
- Launch file to start both bridge and teleop nodes.

## Directory Layout
```
ros2_arduino_bridge/
├── arduino/                # Arduino sketch for the motor controller
├── config/                 # YAML parameter files
├── launch/                 # Launch descriptions
├── resource/               # Ament resource index hook
├── ros2_arduino_bridge/    # Python package sources
├── package.xml             # ROS 2 package manifest
└── setup.py                # Python package definition
```

## Prerequisites
- ROS 2 Humble (or later) desktop installation
- Python 3 with `pyserial`
- Arduino-compatible board flashed with `arduino/ros_arduino_bridge.ino`
- Differential drive base using an L298N (or compatible) H-bridge

## Build Instructions
1. Place the package in your workspace `src` folder.
2. From the workspace root run:
   ```bash
   colcon build --packages-select ros2_arduino_bridge
   source install/setup.bash
   ```

## Running
### Launch Both Nodes
```bash
ros2 launch ros2_arduino_bridge arduino_bridge.launch.py port:=/dev/ttyACM0
```
- `arduino_bridge` connects to the board and handles command / encoder traffic.
- `teleop_keyboard` opens an `xterm` window for keyboard control (requires `xterm`).

### Run Individually
```bash
ros2 run ros2_arduino_bridge arduino_bridge --ros-args -p port:=/dev/ttyACM0
ros2 run ros2_arduino_bridge teleop_keyboard
```

## Topics
- Published
  - `/encoder_ticks` (`std_msgs/Int64MultiArray`)
  - `/joint_states` (`sensor_msgs/JointState`)
- Subscribed
  - `/cmd_vel` (`geometry_msgs/Twist`)
  - `/motor_cmd` (`std_msgs/Int32MultiArray`)

## Serial Permissions
If you encounter `Permission denied` for the USB device:
```bash
sudo usermod -a -G dialout $USER
sudo chmod 666 /dev/ttyACM0
```
Log out/in for group changes to apply.

## Troubleshooting
- Verify the correct serial port with `ls -l /dev/ttyACM*` or `ttyUSB*` before launching.
- Ensure the Arduino sketch baud rate matches the ROS parameter (`57600`).
- If encoder ticks do not change, confirm the interrupt-capable pins match your Arduino model.
