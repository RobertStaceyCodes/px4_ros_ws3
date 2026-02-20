# Luxonis OAK-D Lite Demo — Setup & Usage

## Prerequisites

- Ubuntu 22.04
- ROS 2 Humble
- Luxonis OAK-D Lite connected via USB 3.0 (USB-C)

## 1. Install depthai-ros (already done)

The packages are installed via apt:

```bash
sudo apt install ros-humble-depthai-ros
```

## 2. USB permissions (one-time)

The depthai installer sets up udev rules automatically. If your camera
is not detected, run:

```bash
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' \
  | sudo tee /etc/udev/rules.d/80-movidius.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

Then **unplug and replug** the camera.

## 3. Build the workspace

```bash
cd ~/px4_ros_ws3
colcon build --packages-select px4_sensor_viz_bringup
source install/setup.bash
```

## 4. Run the demo

### Full demo (RGB + depth + point cloud + RViz)

```bash
ros2 launch px4_sensor_viz_bringup oak_d_lite_demo.launch.py
```

### Without point cloud (lower resource usage)

```bash
ros2 launch px4_sensor_viz_bringup oak_d_lite_demo.launch.py enable_pointcloud:=false
```

### Without RViz (headless / just publish topics)

```bash
ros2 launch px4_sensor_viz_bringup oak_d_lite_demo.launch.py start_rviz:=false
```

## 5. Published topics

| Topic | Type | Description |
|---|---|---|
| `/oak/rgb/image_raw` | `sensor_msgs/Image` | Raw RGB image (1080p) |
| `/oak/rgb/camera_info` | `sensor_msgs/CameraInfo` | RGB camera intrinsics |
| `/oak/rgb/image_rect` | `sensor_msgs/Image` | Rectified RGB image |
| `/oak/stereo/image_raw` | `sensor_msgs/Image` | Stereo depth image |
| `/oak/stereo/camera_info` | `sensor_msgs/CameraInfo` | Stereo camera intrinsics |
| `/oak/imu/data` | `sensor_msgs/Imu` | IMU data (BMI270) |
| `/oak/points` | `sensor_msgs/PointCloud2` | Coloured XYZRGB point cloud |

## 6. Verify with CLI

```bash
# List active topics
ros2 topic list | grep oak

# Check RGB image rate
ros2 topic hz /oak/rgb/image_raw

# Check depth image rate
ros2 topic hz /oak/stereo/image_raw

# View an image with rqt
ros2 run rqt_image_view rqt_image_view
```

## 7. Launch arguments

| Argument | Default | Description |
|---|---|---|
| `start_rviz` | `true` | Open RViz2 with preconfigured display |
| `enable_pointcloud` | `true` | Generate XYZRGB point cloud |
| `rectify_rgb` | `true` | Rectify RGB (needed for point cloud) |

## Troubleshooting

- **"No device found"** — Check USB cable is USB 3.0, try a different port,
  verify udev rules (see step 2).
- **Low FPS** — Reduce resolution in `config/oak_d_lite_demo.yaml`
  (e.g. change `i_resolution` to `"720P"` for RGB).
- **RViz blank panels** — Wait a few seconds after launch for the camera
  to initialise. Check `ros2 topic hz /oak/rgb/image_raw`.
- **"Failed to find device"** — Run `lsusb | grep 03e7` to verify the
  camera is visible to the OS.
