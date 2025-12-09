# four_wheel_bot

A minimal ROS 2 package for simulating a 4-wheel skid-steer mobile robot in **Gazebo Classic**,
drivable via keyboard using `teleop_twist_keyboard`.

## Requirements
- ROS 2 (tested with Humble/Iron)
- Gazebo Classic (`gazebo_ros`)
- Packages:
  ```bash
  sudo apt install ros-${ROS_DISTRO}-gazebo-ros \
                   ros-${ROS_DISTRO}-teleop-twist-keyboard \
                   ros-${ROS_DISTRO}-xacro \
                   ros-${ROS_DISTRO}-robot-state-publisher \
                   ros-${ROS_DISTRO}-joint-state-publisher-gui
  ```

## Build
Place this package inside your colcon workspace `src/` and build:
```bash
mkdir -p ~/ws_mobile/src
cp -r four_wheel_bot ~/ws_mobile/src/
cd ~/ws_mobile
colcon build
source install/setup.bash
```

## Run simulation (spawns Gazebo + robot)
```bash
ros2 launch four_wheel_bot gazebo_model.launch.py
```

## Run RViz visualization
```bash
ros2 launch four_wheel_bot rviz.launch.py use_sim_time:=true
```

## Run combined Gazebo + RViz
```bash
ros2 launch four_wheel_bot gazebo_rviz.launch.py
```

## Run teleop (in another terminal)
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/cmd_vel
```
Controls:
- `i`/`,` : forward/backward
- `j`/`l` : turn left/right
- `k`     : stop
- `q/z`   : increase/decrease speed
- `space` : emergency stop

## Robot Description
- **Robot Type**: 4-wheel differential drive robot
- **Dimensions**: 1.0m x 0.6m x 0.3m body
- **Wheels**: 4 wheels (0.3m diameter, 0.1m width)
- **TF Tree**: `odom → dummy → body_link → wheel_links`

## Topics
- `/cmd_vel` - Velocity commands (geometry_msgs/Twist)
- `/joint_states` - Joint positions (sensor_msgs/JointState)
- `/odom` - Odometry data (nav_msgs/Odometry)
- `/tf` - Transform tree

## Notes
- This package uses the `gazebo_ros_diff_drive` plugin and groups the front+rear wheels per side.
- The `dummy` link is required by Gazebo and serves as the robot base frame.
- If the robot slides, try lowering max speeds in teleop and tweak friction parameters on the wheel collisions.
- For Gazebo (Ignition/Garden), use `ros_gz_sim` and a different plugin/bridge.

## Troubleshooting
If you encounter issues with existing Gazebo processes:
```bash
pkill -f gazebo
pkill -f robot_state_publisher
``` 