# Mobile Base System

4-wheel differential drive mobile robot with Arduino control, Gazebo simulation, and Nav2 integration.

## System Architecture

```
                    ┌──────────────────────────────────────────────────┐
                    │                   ROS2 Humble                      │
                    │                                                    │
  ┌─────────┐       │   ┌───────────────┐    ┌────────────────┐        │
  │ Teleop  │──────────▶│ serial_bridge │───▶│ odom_publisher │        │
  │Keyboard │ cmd_vel   │               │    │                │        │
  └─────────┘       │   │  /encoder_    │    │   /odom        │        │
                    │   │   ticks       │    │   /joint_states│        │
                    │   └───────┬───────┘    └────────────────┘        │
                    │           │                                       │
                    └───────────┼───────────────────────────────────────┘
                                │ Serial (115200 baud)
                    ┌───────────▼───────────────────────────────────────┐
                    │            Arduino Mega 2560                       │
                    │                                                    │
                    │   ┌──────────────────────────────────────────┐    │
                    │   │           PID Controller                  │    │
                    │   │  VEL,x,z ──▶ Wheel velocities ──▶ PWM    │    │
                    │   └──────────────────────────────────────────┘    │
                    │                                                    │
                    │   ┌──────────────────────────────────────────┐    │
                    │   │         Encoder ISRs                      │    │
                    │   │  Quadrature ──▶ ENC,fl,fr,bl,br          │    │
                    │   └──────────────────────────────────────────┘    │
                    └────────────────────────────────────────────────────┘
                                │
                    ┌───────────▼───────────────────────────────────────┐
                    │                   Hardware                         │
                    │                                                    │
                    │   ┌────────────┐    ┌────────────────────────┐    │
                    │   │ 2x L298N   │───▶│  4x DC Motors + Encoders│    │
                    │   │ Drivers    │    │  (360 CPR)              │    │
                    │   └────────────┘    └────────────────────────┘    │
                    └────────────────────────────────────────────────────┘
```

## Packages

| Package | Description |
|---------|-------------|
| `mobile_base_description` | URDF/Xacro robot model |
| `base_controller` | Hardware control nodes and launch files |
| `mobile_base_gazebo` | Gazebo simulation with sensors |
| `navigation_bringup` | Nav2 configuration and launch files |

## Robot Specifications

| Parameter | Value |
|-----------|-------|
| Wheel Radius | 0.075m (75mm) |
| Wheel Base (track width) | 0.35m (350mm) |
| Encoder Resolution | 360 CPR |
| Max Linear Velocity | 1.0 m/s |
| Max Angular Velocity | 2.0 rad/s |

## Quick Start

### 1. Build the Workspace

```bash
cd ~/ros2_doc/gripper_car_ws
colcon build --packages-select mobile_base_description base_controller mobile_base_gazebo navigation_bringup
source install/setup.bash
```

### 2. Simulation (Gazebo)

```bash
# Basic simulation with RViz
ros2 launch mobile_base_gazebo gazebo.launch.py

# With SLAM mapping
ros2 launch mobile_base_gazebo gazebo_with_nav.launch.py slam:=true

# Control with teleop
ros2 run base_controller teleop_keyboard.py
```

### 3. Real Hardware (Arduino)

```bash
# Upload Arduino code first (see arduino/ folder)

# Launch hardware nodes
ros2 launch base_controller hardware.launch.py serial_port:=/dev/ttyUSB0

# Enable TEST mode (simulated encoders - no motors needed)
ros2 service call /base/set_test_mode std_srvs/srv/SetBool "{data: true}"

# Control with teleop
ros2 run base_controller teleop_keyboard.py
```

## Serial Protocol

Communication between ROS2 and Arduino uses a simple text-based protocol.

### Commands (ROS2 → Arduino)

| Command | Description | Example |
|---------|-------------|---------|
| `VEL,<x>,<z>` | Velocity command (m/s, rad/s) | `VEL,0.3,0.5` |
| `STOP` | Emergency stop | `STOP` |
| `PID,<kp>,<ki>,<kd>` | Update PID gains | `PID,2.0,0.5,0.1` |
| `RST` | Reset encoder counts | `RST` |
| `TEST,ON` | Enable simulated encoder mode | `TEST,ON` |
| `TEST,OFF` | Use real encoders | `TEST,OFF` |
| `STATUS` | Get current status | `STATUS` |

### Responses (Arduino → ROS2)

| Response | Description | Example |
|----------|-------------|---------|
| `ENC,<fl>,<fr>,<bl>,<br>` | Cumulative encoder ticks | `ENC,1234,1230,1234,1230` |
| `STATUS,<message>` | Status message | `STATUS,Test mode ENABLED` |
| `ERROR,<message>` | Error message | `ERROR,Invalid command` |

## TF Tree

```
map (from SLAM/AMCL)
 └── odom (from odom_publisher or Gazebo)
      └── body_link (robot base)
           ├── front_left_link
           ├── front_right_link
           ├── back_left_link
           ├── back_right_link
           ├── lidar_link
           │    └── laser_frame
           └── camera_link
                └── camera_link_optical
```

## Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/cmd_vel` | geometry_msgs/Twist | Velocity commands |
| `/odom` | nav_msgs/Odometry | Odometry from encoders |
| `/encoder_ticks` | std_msgs/Int64MultiArray | Raw encoder counts |
| `/joint_states` | sensor_msgs/JointState | Wheel positions |
| `/scan` | sensor_msgs/LaserScan | LiDAR data (simulation) |
| `/camera/image_raw` | sensor_msgs/Image | Camera (simulation) |

## Hardware Setup

### Arduino Mega 2560 Pin Connections

**Motor Drivers (2x L298N):**

Note: On this hardware the driver ENABLE (PWM) pins are bridged to HIGH, so
motors are controlled via IN1/IN2 only (no PWM). Use the IN pins for
direction control.

| Motor | IN1 | IN2 |
|-------|-----|-----|
| Front Left | 22 | 23 |
| Front Right | 24 | 25 |
| Back Left | 26 | 27 |
| Back Right | 28 | 29 |

**Encoders (Interrupt Pins):**

| Encoder | Channel A (INT) | Channel B |
|---------|-----------------|-----------|
| Front Left | 18 (INT3) | 31 |
| Front Right | 19 (INT2) | 33 |
| Back Left | 20 (INT1) | 35 |
| Back Right | 21 (INT0) | 37 |

### TEST Mode

TEST mode allows testing the full ROS2 stack without motors or encoders connected. Arduino simulates encoder feedback based on velocity commands.

```bash
# Enable TEST mode
ros2 service call /base/set_test_mode std_srvs/srv/SetBool "{data: true}"

# Send velocity command
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}}" -r 10

# Check encoder ticks (should be incrementing)
ros2 topic echo /encoder_ticks

# Check odometry (should show movement)
ros2 topic echo /odom
```

## Troubleshooting

### Serial Connection Issues

1. **Check port:** `ls -la /dev/ttyACM* /dev/ttyUSB*`
2. **Check permissions:** `sudo usermod -a -G dialout $USER` (logout/login required)
3. **Monitor serial:** `minicom -D /dev/ttyACM0 -b 115200`

### Arduino Not Responding

- Arduino needs DTR toggle to reset. The serial_bridge does this automatically.
- Open Arduino Serial Monitor once to verify connection, then close it.
- Check baud rate matches (115200).

### Odometry Drift

- Calibrate `wheel_radius` and `wheel_base` in `base_params.yaml`
- Check encoder direction (motors may be wired opposite)
- Tune PID gains via `PID,kp,ki,kd` command

## Files Structure

```
mobile_base/
├── README.md                    # This file
├── mobile_base_description/     # Robot URDF model
│   ├── urdf/
│   │   └── mobile_base.urdf.xacro
│   ├── meshes/                  # 3D models (STL files)
│   └── config/display.rviz
├── base_controller/             # Hardware control
│   ├── scripts/
│   │   ├── serial_bridge_node.py
│   │   ├── odom_publisher_node.py
│   │   └── teleop_keyboard.py
│   ├── config/base_params.yaml
│   └── launch/
│       ├── hardware.launch.py
│       └── teleop.launch.py
├── mobile_base_gazebo/          # Gazebo simulation
│   ├── urdf/gazebo.xacro        # Gazebo plugins
│   ├── worlds/                  # World files
│   ├── config/sim.rviz
│   └── launch/
│       ├── gazebo.launch.py
│       └── gazebo_with_nav.launch.py
└── navigation_bringup/          # Nav2 configuration
    ├── config/
    │   ├── nav2_params.yaml
    │   └── slam_params.yaml
    └── launch/
        └── navigation.launch.py

arduino/
└── mobile_base_controller/
    └── mobile_base_controller.ino  # Arduino code
```

## License

MIT License
