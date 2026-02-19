#!/bin/bash

echo "Stopping Simulation processes..."

# 1. Kill PX4 Autopilot and Make
pkill -f "make px4_sitl"
pkill -f "px4" 

# 2. Kill Gazebo / GZ Sim
pkill -f "gz sim"
pkill -f "ruby"
pkill -f "gz-sim-gui"
pkill -f "gz-sim-server"

# 3. Kill MicroXRCEAgent
pkill -f "MicroXRCEAgent"

# 4. Kill ROS 2 Bridge
pkill -f "parameter_bridge"

# 5. Kill the Python Avoidance Script
pkill -f "simple_avoidance.py"

echo "All simulation processes have been signaled to stop."
