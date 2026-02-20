# LDLiDAR LD19 Setup & Demo

360-degree 2D DToF lidar running on ROS 2 Humble via the
[ldrobot-lidar-ros2](https://github.com/Myzhar/ldrobot-lidar-ros2) driver.

## Hardware

| Item | Value |
|------|-------|
| Sensor | LDRobot LD19 |
| Interface | USB (CP2102 UART bridge) |
| Default device | `/dev/ttyUSB0` |
| Baud rate | 230400 |
| Scan rate | ~10 Hz |
| Range | 0.03 – 15 m |
| Field of view | 360° |

## Prerequisites

```bash
# ROS 2 Humble must be sourced
source /opt/ros/humble/setup.bash

# System dependency
sudo apt install -y libudev-dev

# ROS dependencies required by ldlidar_component
sudo apt install -y \
  ros-humble-nav2-util \
  ros-humble-nav2-msgs \
  ros-humble-bond \
  ros-humble-bondcpp
```

## Installation

### 1. Clone the driver

```bash
cd ~/px4_ros_ws3/src/
git clone https://github.com/Myzhar/ldrobot-lidar-ros2.git
```

### 2. Install udev rules (creates `/dev/ldlidar` symlink)

```bash
cd ~/px4_ros_ws3/src/ldrobot-lidar-ros2/scripts/
./create_udev_rules.sh
```

After this, the LD19 will also appear as `/dev/ldlidar` whenever it is plugged in.

### 3. Add your user to the dialout group (if not already)

```bash
sudo usermod -aG dialout $USER
# Log out and back in for the group change to take effect
```

### 4. Build

```bash
cd ~/px4_ros_ws3
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --cmake-args=-DCMAKE_BUILD_TYPE=Release \
  --packages-up-to px4_sensor_viz_bringup
source install/setup.bash
```

## Running the Demo

### Launch with RViz2 (default)

```bash
cd ~/px4_ros_ws3
source install/setup.bash
ros2 launch px4_sensor_viz_bringup ldlidar_ld19_demo.launch.py
```

This will:
- Start the `ldlidar_node` composable component
- Publish the LD19 URDF via `robot_state_publisher`
- Automatically configure and activate the lifecycle node
- Open RViz2 with a top-down laser scan view

### Launch arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `serial_port` | `/dev/ttyUSB0` | Serial port for the LD19 |
| `start_rviz` | `true` | Whether to start RViz2 |

Examples:

```bash
# Use the udev symlink instead of ttyUSB0
ros2 launch px4_sensor_viz_bringup ldlidar_ld19_demo.launch.py serial_port:=/dev/ldlidar

# Headless (no RViz) — useful on a Jetson without a display
ros2 launch px4_sensor_viz_bringup ldlidar_ld19_demo.launch.py start_rviz:=false
```

### Verify scan data (separate terminal)

```bash
source ~/px4_ros_ws3/install/setup.bash

# Check publish rate (~10 Hz expected)
ros2 topic hz /ldlidar_node/scan

# Print one complete scan message
ros2 topic echo /ldlidar_node/scan --once

# List all active topics
ros2 topic list
```

### Stop

Press **Ctrl+C** in the launch terminal. The `buffer overflow detected` message on
shutdown is a known upstream bug in the driver's bond cleanup — it does not affect
normal operation.

## Configuration

The lidar parameters live in `config/ldlidar_ld19.yaml`:

```yaml
/**:
  ros__parameters:
    general:
      debug_mode: false
    comm:
      serial_port: '/dev/ttyUSB0'
      baudrate: 230400
      timeout_msec: 1000
    lidar:
      model: 'LDLiDAR_LD19'
      rot_verse: 'CCW'        # CCW = standard ROS convention
      units: 'M'              # meters
      frame_id: 'ldlidar_link'
      bins: 455                # fixed bin count for SLAM compatibility
      range_min: 0.03
      range_max: 15.0
      enable_angle_crop: false
      angle_crop_min: 90.0
      angle_crop_max: 270.0
```

Key parameters to adjust:
- **`rot_verse`**: Set to `CW` if the lidar is mounted upside-down.
- **`bins`**: Set to `0` for dynamic scan size, or `455` for SLAM Toolbox compatibility.
- **`enable_angle_crop`**: Set to `true` to mask a range of angles (e.g. to ignore
  the drone body).

## Files Created

```
src/px4_sensor_viz_bringup/
├── config/
│   ├── ldlidar_ld19.yaml          # LD19 parameters
│   └── ldlidar_ld19_demo.rviz     # RViz2 config (top-down laser scan view)
└── launch/
    └── ldlidar_ld19_demo.launch.py # Self-contained demo launch
```

## Troubleshooting

| Problem | Solution |
|---------|----------|
| `Transitioning failed` on configure | Check that `/dev/ttyUSB0` exists (`ls /dev/ttyUSB*`) and your user is in the `dialout` group (`groups`) |
| No scan data in RViz | Make sure the Fixed Frame is set to `ldlidar_base` and the LaserScan topic is `/ldlidar_node/scan` |
| Permission denied on serial port | `sudo chmod 666 /dev/ttyUSB0` (temporary) or add udev rules (permanent) |
| Device not found | Unplug and replug the USB cable, then check `dmesg \| tail` |
