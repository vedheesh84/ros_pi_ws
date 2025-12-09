# Real-Time Controller Interface for ROS2 MoveIt2

This document describes how to use the real-time controller interface with the mobile arm manipulator and MoveIt2.

## Overview

The real-time controller interface provides:
- High-frequency control loops (1000 Hz)
- Support for position, velocity, and effort control interfaces
- Real-time trajectory execution
- Force/torque sensor integration (if available)
- Optimized performance for industrial applications

## Configuration Files

### Real-Time Controllers Configuration
- `config/ros2_realtime_controllers.yaml` - Real-time controller parameters
- `config/moveit_realtime_controllers.yaml` - MoveIt2 integration with real-time controllers
- `config/mobile_arm_bot_2.ros2_control.xacro` - Updated with velocity and effort interfaces

### Launch Files
- `launch/realtime_controllers.launch.py` - Launch real-time controllers only
- `launch/realtime_demo.launch.py` - Complete real-time demo with MoveIt2 and RViz

## Usage

### 1. Real-Time Controllers Only
Launch just the real-time controllers:
```bash
ros2 launch mobile_arm_manipulator_config realtime_controllers.launch.py
```

### 2. Complete Real-Time Demo
Launch the full real-time demo with MoveIt2:
```bash
ros2 launch mobile_arm_manipulator_config realtime_demo.launch.py
```

### 3. With Real Hardware
For real hardware, disable fake hardware:
```bash
ros2 launch mobile_arm_manipulator_config realtime_demo.launch.py use_fake_hardware:=false robot_ip:=<YOUR_ROBOT_IP>
```

### 4. With Simulation Time
For Gazebo simulation:
```bash
ros2 launch mobile_arm_manipulator_config realtime_demo.launch.py use_sim_time:=true
```

## Launch Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `use_sim_time` | `false` | Use simulation (Gazebo) clock |
| `use_fake_hardware` | `true` | Use fake hardware for testing |
| `fake_sensor_commands` | `false` | Enable fake sensor commands |
| `robot_ip` | `192.168.1.100` | Robot server IP address |
| `launch_rviz` | `true` | Launch RViz2 with MoveIt configuration |

## Real-Time Features

### High-Frequency Control
- Controller update rate: 1000 Hz
- State publish rate: 50 Hz
- Action monitor rate: 20 Hz

### Control Interfaces
Each joint supports:
- Position control
- Velocity control
- Effort control

### Trajectory Constraints
- Stopped velocity tolerance: 0.01 rad/s
- Goal time tolerance: 0.6 seconds
- Joint-specific trajectory and goal tolerances

### Force/Torque Sensor Support
- Optional force/torque sensor integration
- Publish rate: 100 Hz
- Frame ID: `force_torque_sensor_link`

## Controller Management

### Available Controllers
- `mobile_arm_realtime_controller` - Main arm trajectory controller
- `gripper_realtime_controller` - Gripper position controller
- `joint_state_broadcaster` - Joint state publisher
- `force_torque_sensor_controller` - Force/torque sensor (if available)

### Controller Commands
```bash
# List controllers
ros2 control list_controllers

# Check controller state
ros2 control list_controllers --state

# Switch controllers
ros2 control switch_controllers --activate mobile_arm_realtime_controller

# Load controller
ros2 control load_controller mobile_arm_realtime_controller

# Unload controller
ros2 control unload_controller mobile_arm_realtime_controller
```

## Integration with MoveIt2

The real-time controllers are fully integrated with MoveIt2:
- Trajectory execution through real-time controllers
- Real-time motion planning feedback
- Collision avoidance with real-time constraints
- Force/torque-aware planning (if sensor available)

## Performance Optimization

### Real-Time Kernel
For optimal performance, use a real-time kernel:
```bash
# Check if real-time kernel is running
uname -a

# Install real-time kernel (Ubuntu)
sudo apt install linux-image-rt linux-headers-rt
```

### CPU Isolation
Isolate CPU cores for real-time tasks:
```bash
# Add to GRUB configuration
GRUB_CMDLINE_LINUX_DEFAULT="quiet splash isolcpus=2,3 rcu_nocbs=2,3"
```

### Process Priority
Set high priority for controller processes:
```bash
# Set real-time priority
chrt -f 99 ros2 launch mobile_arm_manipulator_config realtime_controllers.launch.py
```

## Troubleshooting

### Common Issues

1. **Controller not starting**
   - Check if `joint_state_broadcaster` is active first
   - Verify hardware connection
   - Check controller configuration

2. **High latency**
   - Use real-time kernel
   - Isolate CPU cores
   - Check system load

3. **Trajectory execution failures**
   - Verify joint limits
   - Check trajectory constraints
   - Monitor controller state

### Debug Commands
```bash
# Monitor controller performance
ros2 topic hz /joint_states

# Check controller state
ros2 control list_controllers --state

# Monitor trajectory execution
ros2 topic echo /mobile_arm_realtime_controller/follow_joint_trajectory/feedback
```

## Dependencies

Required packages:
- `joint_trajectory_controller`
- `position_controllers`
- `force_torque_sensor_broadcaster`
- `ros2_controllers`
- `realtime_tools`
- `controller_manager`

## Safety Considerations

- Always test with fake hardware first
- Verify joint limits and safety constraints
- Monitor system performance and latency
- Use appropriate emergency stop mechanisms
- Follow industrial safety standards

