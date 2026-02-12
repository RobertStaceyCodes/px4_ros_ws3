# SITL Integration Notes: x500 + Oak-D-Lite + dual TF-Luna + D500 stand-in

## Context

Goal: create a single Gazebo SITL model for PX4 that combines:
- Oak-D-Lite camera
- two 1D range sensors (TF-Luna stand-ins: up and down)
- one 2D lidar (Waveshare D500 stand-in)

The 2D lidar is represented by the existing `lidar_2d_v2` Gazebo model.
The two TF-Luna sensors are represented by custom 1-sample `gpu_lidar` sensors.

## What was added

### New Gazebo model (in PX4 gazebo models submodule)

- `PX4-Autopilot/Tools/simulation/gz/models/x500_oak_tfluna_d500/model.config`
- `PX4-Autopilot/Tools/simulation/gz/models/x500_oak_tfluna_d500/model.sdf`

Model composition in `model.sdf`:
- Includes `x500` as base
- Includes `model://OakD-Lite` and fixed joint to `camera_link`
- Includes `model://lidar_2d_v2` and fixed joint to its `link`
- Adds `tf_luna_down_link` + `tf_luna_down` (`gpu_lidar`, 1 beam)
- Adds `tf_luna_up_link` + `tf_luna_up` (`gpu_lidar`, 1 beam)

### New PX4 SITL airframe entry

- `PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/4022_gz_x500_oak_tfluna_d500`

This airframe sets:
- `PX4_SIM_MODEL=${PX4_SIM_MODEL:=x500_oak_tfluna_d500}`
- then sources `4001_gz_x500` defaults.

## How PX4 discovers the target

`gz_<model>` make targets are auto-generated from files matching:
- `PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/*_gz_*`

Adding the new `4022_gz_x500_oak_tfluna_d500` file enables:
- `make px4_sitl gz_x500_oak_tfluna_d500`

## Important path issue encountered

PX4 checkouts on disk:
- `/home/rob/px4_ros_ws/PX4-Autopilot` (where changes were made)


Error seen:
- `ninja: error: unknown target 'gz_x500_oak_tfluna_d500'`

Reason:
- build executed in the checkout that did not contain the new airframe/model files.

## Run instructions (correct checkout)

```bash
cd /home/rob/px4_ros_ws/PX4-Autopilot
rm -rf build/px4_sitl_default
make px4_sitl gz_x500_oak_tfluna_d500
```

## Quick verification during SITL

In PX4 shell:
- `listener distance_sensor 5`
- `listener obstacle_distance 5`

In Gazebo:
- `gz topic -l | rg "depth_camera|lidar|tf_luna"`

## Running with WASD teleop

Use four terminals:

1) PX4 + Gazebo
```bash
cd /home/rob/px4_ros_ws_broken_20260211_221222/PX4-Autopilot
PX4_GZ_WORLD=forest PX4_GZ_MODEL_POSE="0,0,3,0,0,0" make px4_sitl gz_x500_oak_tfluna_d500
```

2) Sensor bridge + RViz
```bash
cd /home/rob/px4_ros_ws_broken_20260211_221222
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch px4_sensor_viz_bringup sitl_oak_lidar_viz.launch.py world:=forest
```

3) Start Micro XRCE-DDS Agent (required for `/fmu/in/*` ROS topics)
```bash
MicroXRCEAgent udp4 -p 8888
```

4) Keyboard teleop (WASD)
```bash
cd /home/rob/px4_ros_ws_broken_20260211_221222
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run px4_ros_com wasd_teleop.py
```

Note: run teleop with `ros2 run` (not `ros2 launch`) so the node keeps an interactive TTY for keypress input.

## Repository note

`Tools/simulation/gz` is a submodule. If committing later, changes are split:
- commit inside `PX4-Autopilot/Tools/simulation/gz` (new model folder)
- commit in `PX4-Autopilot` (new airframe + submodule pointer update)
