# Maze Navigator - Waypoint and Map Visualization Guide

This guide explains how to send waypoint coordinates to the maze navigator and visualize the map.

## Features Added

1. **Dynamic Waypoint Support**: The navigator can now receive goal coordinates via ROS2 topics
2. **Occupancy Grid Visualization**: Real-time map published for visualization in RViz2
3. **Obstacle Distance**: Obstacle distance messages published for QGroundControl visualization

## Setting Waypoints

### Method 1: Using the Command Line Tool

Use the provided `send_waypoint.py` script:

```bash
# Navigate to the workspace
cd ~/px4_ros_ws3
source install/setup.bash

# Send a waypoint (NED coordinates in meters)
ros2 run px4_ros_com send_waypoint.py <x> <y> [z]

# Examples:
ros2 run px4_ros_com send_waypoint.py 0.0 24.0      # Default goal
ros2 run px4_ros_com send_waypoint.py 10.0 15.0      # Custom waypoint
ros2 run px4_ros_com send_waypoint.py 5.0 20.0 0.0  # With z coordinate
```

### Method 2: Using ROS2 Topic Command

You can also publish directly using `ros2 topic pub`:

```bash
ros2 topic pub --once /maze_navigator/set_goal geometry_msgs/PointStamped \
  "{header: {frame_id: 'map'}, point: {x: 0.0, y: 24.0, z: 0.0}}"
```

### Method 3: From QGroundControl (via MAVLink Bridge)

A bridge node is provided to forward waypoints from QGC to the maze navigator.

**Setup:**
1. Ensure QGC is connected to PX4 via MAVLink (over Tailscale network)
2. Start the bridge node on the Jetson:
```bash
# On Jetson (where simulation runs)
cd ~/px4_ros_ws3
source install/setup.bash

# Start the bridge (default: listens on UDP port 14540)
ros2 run px4_ros_com qgc_waypoint_bridge.py

# Or specify custom MAVLink port:
ros2 run px4_ros_com qgc_waypoint_bridge.py --mavlink-port udp:14550
```

3. In QGC, create a mission with waypoints as usual
4. When you upload/start the mission in QGC, the bridge will automatically:
   - Receive MISSION_ITEM_INT messages from QGC
   - Convert global coordinates to local NED
   - Forward waypoints to the maze navigator

**Note:** The bridge uses the first valid GPS position as the home reference for coordinate conversion. Make sure PX4 has GPS lock or is using simulated GPS before sending waypoints.

## Visualizing the Map

### In RViz2

1. Start RViz2:
```bash
rviz2
```

2. Add a "Map" display:
   - Click "Add" → "By display type" → "Map"
   - Set the topic to `/maze_navigator/occupancy_grid`
   - Set the fixed frame to `map`

3. The occupancy grid will show:
   - **White (0)**: Free space
   - **Black (100)**: Occupied/obstacles
   - **Gray (-1)**: Unknown space

### In QGroundControl

The obstacle distance data is published to `/fmu/in/obstacle_distance`, which PX4 can forward to QGC via MAVLink. However, QGC's visualization of obstacle distance is limited compared to RViz2's occupancy grid display.

To enable obstacle distance visualization in QGC:
1. Ensure PX4 is configured to forward obstacle distance messages
2. QGC should display obstacle information in the HUD (if supported)

**Note**: For best map visualization, use RViz2 with the occupancy grid topic.

## Coordinate System

- **Frame**: NED (North-East-Down) local frame
- **X**: North (positive = north)
- **Y**: East (positive = east)
- **Z**: Down (negative = up)

The maze world uses:
- Start: (0, 0) in Gazebo ENU → (0, 0) in NED
- Default Goal: (24, 0) in Gazebo ENU → (0, 24) in NED

## Example Workflow

### Basic Workflow (Command Line)

1. Start the maze navigator:
```bash
ros2 run px4_ros_com maze_navigator.py
```

2. In another terminal, send a waypoint:
```bash
ros2 run px4_ros_com send_waypoint.py 10.0 15.0
```

3. The navigator will:
   - Log the new goal: `New goal set: NED (10.00, 15.00)`
   - Replan the path to the new goal
   - Navigate to the waypoint

4. Visualize the map in RViz2 (see above)

### QGC Workflow (MacBook + Jetson via Tailscale)

**On Jetson (simulation side):**
1. Start PX4 SITL with MAVLink enabled (listening on UDP port 14540)
2. Start the maze navigator:
```bash
ros2 run px4_ros_com maze_navigator.py
```
3. Start the QGC bridge:
```bash
ros2 run px4_ros_com qgc_waypoint_bridge.py
```

**On MacBook (QGC side):**
1. Connect QGC to PX4 via Tailscale:
   - In QGC: Application Settings → Comm Links
   - Add UDP connection: `Jetson_Tailscale_IP:14540`
   - Connect to the vehicle
2. Create a mission in QGC:
   - Plan → Add Waypoints
   - Set waypoint coordinates (lat/lon/alt)
   - Upload mission to vehicle
3. Start the mission in QGC
4. The bridge will forward waypoints to the maze navigator automatically

**Visualization:**
- RViz2 on Jetson: `/maze_navigator/occupancy_grid` for detailed map
- QGC on MacBook: Standard QGC mission view (limited obstacle visualization)

## Troubleshooting

- **Waypoint not updating**: 
  - Ensure the navigator is running and subscribed to `/maze_navigator/set_goal`
  - Check bridge logs: `ros2 topic echo /maze_navigator/set_goal` to see if waypoints are being published
  
- **QGC bridge not connecting**:
  - Verify PX4 MAVLink is listening: `netstat -ulnp | grep 14540`
  - Check Tailscale connection: `ping Jetson_IP` from MacBook
  - Try different MAVLink port: `--mavlink-port udp:14550`
  - Check bridge logs for connection errors
  
- **Coordinates seem wrong**:
  - Remember the coordinate system is NED, not ENU
  - Bridge converts global (lat/lon) to local NED using home position
  - Ensure PX4 has valid GPS/home position before sending waypoints
  
- **Map not showing**: 
  - Check that RViz2 is subscribed to `/maze_navigator/occupancy_grid` 
  - Verify fixed frame is set correctly (should be `map`)
  - Ensure navigator is running and publishing map updates

## Topics

### Subscribed Topics
- `/maze_navigator/set_goal` (geometry_msgs/PointStamped): Set new goal waypoint

### Published Topics
- `/maze_navigator/occupancy_grid` (nav_msgs/OccupancyGrid): Occupancy grid map for visualization
- `/fmu/in/obstacle_distance` (px4_msgs/ObstacleDistance): Obstacle distance for QGC
