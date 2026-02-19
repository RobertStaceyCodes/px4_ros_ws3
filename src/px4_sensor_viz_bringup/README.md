# px4_sensor_viz_bringup

ROS 2 Humble bringup package for visualizing the custom PX4 SITL model:
- `x500_oak_tfluna_d500`

It launches:
- `ros_gz_bridge` topic bridges for lidar and camera topics
- Gazebo dynamic TF bridge (`/world/<world>/dynamic_pose/info -> /tf_raw`)
- `tf_sanitizer` node to republish clean TF and alias short frame names to `/tf`
- static TFs for sensor frames (`base_link -> camera/lidar/tf_luna`)
- RViz with a ready display config for lidar scans and camera streams

## Prerequisites

- PX4 SITL running with:
  - `make px4_sitl gz_x500_oak_tfluna_d500`
- ROS 2 Humble
- `ros_gz_bridge` installed
- `rviz2` installed

## Build

```bash
cd /home/rob/px4_ros_ws
colcon build --packages-select px4_sensor_viz_bringup
source install/setup.bash
```

## Launch

```bash
ros2 launch px4_sensor_viz_bringup sitl_oak_lidar_viz.launch.py
```

## Useful launch arguments

- `world:=default`
- `model_instance:=x500_oak_tfluna_d500_0`
- `gz_partition:=<partition_name>` (if using non-default Gazebo partition)
- `start_bridge:=true|false`
- `start_tf_sanitizer:=true|false`
- `start_rviz:=true|false`
- `start_static_tf:=true|false` (default true)

Example with explicit model instance:

```bash
ros2 launch px4_sensor_viz_bringup sitl_oak_lidar_viz.launch.py \
  model_instance:=x500_oak_tfluna_d500_0
```

## Verification

Check ROS topics:

```bash
ros2 topic list | grep -E "^/tf$|/d500/scan|/tf_luna|/oak/"
```

Inspect data rates:

```bash
ros2 topic hz /d500/scan
ros2 topic hz /oak/rgb/image_raw
```

## Notes

- The launch file is intentionally deterministic: set `model_instance` to match your spawned model.
- RViz uses `world` as fixed frame; vehicle motion comes from Gazebo dynamic poses bridged into `/tf_raw` and sanitized into `/tf`.
- Depth image display uses `/oak/depth/image_raw`, remapped from `/depth_camera`.

## Running in the walls world

Start PX4 SITL in walls world first:

```bash
cd /home/rob/px4_ros_ws/PX4-Autopilot
PX4_GZ_WORLD=walls make px4_sitl gz_x500_oak_tfluna_d500
```

Then launch this package with matching world:

```bash
cd /home/rob/px4_ros_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch px4_sensor_viz_bringup sitl_oak_lidar_viz.launch.py world:=walls
```
