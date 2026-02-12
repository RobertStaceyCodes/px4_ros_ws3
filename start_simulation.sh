#!/bin/bash

# Define Session Name
SESSION_NAME="px4_sim"
WS_DIR="$HOME/px4_ros_ws"
PX4_DIR="$HOME/PX4-Autopilot"

# Check if tmux is installed
if ! command -v tmux &> /dev/null; then
    echo "Error: tmux is not installed."
    echo "Install it with: sudo apt update && sudo apt install tmux -y"
    exit 1
fi

# Check if session exists to prevent double creation
tmux has-session -t $SESSION_NAME 2>/dev/null

if [ $? != 0 ]; then
    # 1. Create new session and run PX4 SITL in the first window
    tmux new-session -d -s $SESSION_NAME -n "PX4_SITL"
    tmux send-keys -t $SESSION_NAME:0 "cd $PX4_DIR" C-m
    tmux send-keys -t $SESSION_NAME:0 "make px4_sitl gz_x500_lidar_2d_walls" C-m

    # 2. Create window for MicroXRCEAgent
    tmux new-window -t $SESSION_NAME:1 -n "MicroXRCEAgent"
    tmux send-keys -t $SESSION_NAME:1 "MicroXRCEAgent udp4 -p 8888" C-m

    # 3. Create window for ROS GZ Bridge
    tmux new-window -t $SESSION_NAME:2 -n "ROS_Bridge"
    tmux send-keys -t $SESSION_NAME:2 "source $WS_DIR/install/setup.bash" C-m
    # Note: Using quotes around the bridge topic argument to handle special characters like '['
    tmux send-keys -t $SESSION_NAME:2 "ros2 run ros_gz_bridge parameter_bridge '/world/walls/model/x500_lidar_2d_0/link/link/sensor/lidar_2d_v2/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan' --ros-args -r /world/walls/model/x500_lidar_2d_0/link/link/sensor/lidar_2d_v2/scan:=/scan" C-m

    # 4. Create window for Simple Avoidance Script
    tmux new-window -t $SESSION_NAME:3 -n "Byth_Script"
    tmux send-keys -t $SESSION_NAME:3 "cd $WS_DIR" C-m
    tmux send-keys -t $SESSION_NAME:3 "source install/setup.bash" C-m
    tmux send-keys -t $SESSION_NAME:3 "python3 simple_avoidance.py" C-m

    # Select the first window
    tmux select-window -t $SESSION_NAME:0
fi

# Attach to the session
tmux attach-session -t $SESSION_NAME
