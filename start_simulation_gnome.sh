#!/bin/bash

WS_DIR="$HOME/px4_ros_ws"
PX4_DIR="$HOME/PX4-Autopilot"

# Open a new window for PX4 SITL
gnome-terminal --tab --title="PX4 SITL" -- bash -c "cd $PX4_DIR; make px4_sitl gz_x500_lidar_2d_walls; exec bash"

# Open a tab for MicroXRCEAgent
gnome-terminal --tab --title="MicroXRCEAgent" -- bash -c "MicroXRCEAgent udp4 -p 8888; exec bash"

# Open a tab for ROS Bridge
gnome-terminal --tab --title="ROS Bridge" -- bash -c "source $WS_DIR/install/setup.bash; ros2 run ros_gz_bridge parameter_bridge '/world/walls/model/x500_lidar_2d_0/link/link/sensor/lidar_2d_v2/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan' --ros-args -r /world/walls/model/x500_lidar_2d_0/link/link/sensor/lidar_2d_v2/scan:=/scan; exec bash"

# Open a tab for Simple Avoidance
gnome-terminal --tab --title="Avoidance" -- bash -c "cd $WS_DIR; source install/setup.bash; python3 simple_avoidance.py; exec bash"
