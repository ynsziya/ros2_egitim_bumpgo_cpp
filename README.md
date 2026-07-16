# ros2_egitim_bumpgo_cpp

A ROS 2 C++ package that implements the classic **Bump & Go** behavior for the TIAGo robot.

The robot uses lidar to detect obstacles ahead. If the path is clear it drives forward; when an obstacle is detected it backs up, turns, and continues.

## Features

- Front-region obstacle detection from lidar (`sensor_msgs/LaserScan`)
- Three-state behavior machine: `DRIVING` → `BACKING` → `TURNING`
- Tunable speeds, distance threshold, and timings via ROS parameters
- Launch files for real robot and Gazebo simulation

## Requirements

- ROS 2 (Humble or newer recommended)
- Dependencies: `rclcpp`, `sensor_msgs`, `geometry_msgs`, `launch`, `launch_ros`
- For simulation: the [`br2_tiago`](https://github.com/fmrico/book_ros2) package (TIAGo + Gazebo)

## Installation

Clone the package into a ROS 2 workspace and build it:

```bash
cd ~/tiago_ws/src
git clone <repo-url> ros2_egitim_bumpgo_cpp

cd ~/tiago_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select ros2_egitim_bumpgo_cpp
source install/setup.bash
```

## Usage

### Bump & Go node only

```bash
ros2 launch ros2_egitim_bumpgo_cpp egitim_bumpgo.launch.py
```

### With simulation (Gazebo + TIAGo)

```bash
ros2 launch ros2_egitim_bumpgo_cpp egitim_sim_bumpgo.launch.py
```

You can override parameters via launch arguments:

```bash
ros2 launch ros2_egitim_bumpgo_cpp egitim_bumpgo.launch.py \
  obstacle_distance:=0.6 \
  linear_speed:=0.25 \
  backing_time:=1.5 \
  turning_time:=2.0
```

## Behavior

| State | Description |
|-------|-------------|
| `DRIVING` | Moves forward. Switches to `BACKING` when the front distance falls below the threshold. |
| `BACKING` | Moves backward for a fixed duration, then switches to `TURNING`. |
| `TURNING` | Rotates for a fixed duration, then returns to `DRIVING`. |

## Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `obstacle_distance` | `0.5` | Obstacle distance threshold (m) |
| `front_angle` | `45.0` | Front scan half-angle (± degrees) |
| `linear_speed` | `0.2` | Forward speed (m/s) |
| `angular_speed` | `0.3` | Turning speed (rad/s) |
| `backup_speed` | `0.1` | Backward speed (m/s) |
| `backing_time` | `2.0` | Backup duration (s) |
| `turning_time` | `2.0` | Turning duration (s) |

## Topics

| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `scan_raw` | `sensor_msgs/LaserScan` | Subscribe | Lidar data |
| `/cmd_vel` | `geometry_msgs/Twist` | Publish | Velocity command |

## Package layout

```
ros2_egitim_bumpgo_cpp/
├── include/ros2_egitim_bumpgo_cpp/
│   └── EgitimBumpGoNode.hpp
├── src/
│   ├── egitim_bumpgo_main.cpp
│   └── ros2_egitim_bumpgo_cpp/
│       └── EgitimBumpGoNode.cpp
├── launch/
│   ├── egitim_bumpgo.launch.py
│   └── egitim_sim_bumpgo.launch.py
├── CMakeLists.txt
└── package.xml
```

## License

This package is intended for educational use. A license can be declared in `package.xml`.
