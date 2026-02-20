# QGC Waypoint Bridge Setup Guide

This guide explains how to set up the QGC waypoint bridge for sending waypoints from QGroundControl (running on MacBook via Tailscale) to the maze navigator (running on Jetson).

## Prerequisites

1. **QGC installed on MacBook** and connected to PX4 via Tailscale
2. **PX4 SITL running on Jetson** with MAVLink enabled
3. **pymavlink installed** on Jetson

## Installation

### Install pymavlink on Jetson

```bash
# On Jetson
pip3 install pymavlink
# Or if using system Python:
sudo pip3 install pymavlink
```

## Network Setup

### 1. Verify Tailscale Connection

**On MacBook:**
```bash
# Find Jetson's Tailscale IP
ping <jetson_tailscale_hostname>
```

**On Jetson:**
```bash
# Find Jetson's Tailscale IP
tailscale ip
```

### 2. Configure PX4 MAVLink

Ensure PX4 SITL is configured to listen on UDP port 14540 (default):

```bash
# In PX4 SITL launch, MAVLink should be configured to listen on:
# UDP port 14540 (default)
```

If using a different port, adjust the bridge accordingly.

### 3. Configure QGC Connection

**On MacBook:**
1. Open QGroundControl
2. Go to: **Application Settings → Comm Links**
3. Add new UDP connection:
   - **Name**: Jetson PX4
   - **Type**: UDP
   - **Listening Port**: 14540
   - **Target Host**: `<jetson_tailscale_ip>`
   - **Target Port**: 14540
4. Click **OK** and **Connect**

## Running the Bridge

### On Jetson (Terminal 1): Start PX4 SITL
```bash
cd ~/px4_ros_ws3/PX4-Autopilot
PX4_GZ_WORLD=maze make px4_sitl gz_x500_oak_tfluna_d500
```

### On Jetson (Terminal 2): Start Maze Navigator
```bash
cd ~/px4_ros_ws3
source install/setup.bash
ros2 run px4_ros_com maze_navigator.py
```

### On Jetson (Terminal 3): Start QGC Bridge
```bash
cd ~/px4_ros_ws3
source install/setup.bash

# Default (UDP port 14540)
ros2 run px4_ros_com qgc_waypoint_bridge.py

# Custom port
ros2 run px4_ros_com qgc_waypoint_bridge.py --mavlink-port udp:14550
```

### On Jetson (Terminal 4, Optional): Start RViz2 for Map Visualization
```bash
cd ~/px4_ros_ws3
source install/setup.bash
rviz2
# Then add Map display with topic: /maze_navigator/occupancy_grid
```

## Using QGC to Send Waypoints

1. **In QGC (MacBook):**
   - Go to **Plan** view
   - Click **Add Waypoint** on the map
   - Set waypoint coordinates (lat/lon/alt)
   - Add multiple waypoints as needed
   - Click **Upload** to send mission to vehicle

2. **The bridge will automatically:**
   - Receive MISSION_ITEM_INT messages from QGC
   - Convert global coordinates (lat/lon) to local NED
   - Forward waypoints to `/maze_navigator/set_goal`
   - Log waypoint conversions in the bridge terminal

3. **The maze navigator will:**
   - Receive new waypoint goals
   - Replan path using A* planner
   - Navigate to the waypoint autonomously

## Verification

### Check Bridge is Receiving MAVLink Messages

**On Jetson:**
```bash
# Watch bridge logs for waypoint messages
# You should see logs like:
# "Received waypoint from QGC: seq=1, global=(47.397742, 8.545594, 500.00), NED=(0.00, 24.00, -500.00)"
```

### Check Waypoints are Being Published

**On Jetson:**
```bash
ros2 topic echo /maze_navigator/set_goal
# You should see PointStamped messages with NED coordinates
```

### Check Navigator is Receiving Waypoints

**In maze navigator logs, you should see:**
```
New goal set: NED (0.00, 24.00) from frame 'map'
```

## Troubleshooting

### Bridge Can't Connect to MAVLink

**Symptoms:** Bridge logs show "Failed to connect to MAVLink"

**Solutions:**
1. Verify PX4 is running and MAVLink is active:
   ```bash
   netstat -ulnp | grep 14540
   ```
2. Check firewall rules (MAVLink uses UDP)
3. Verify Tailscale connection:
   ```bash
   ping <jetson_tailscale_ip>
   ```
4. Try different MAVLink port:
   ```bash
   ros2 run px4_ros_com qgc_waypoint_bridge.py --mavlink-port udp:14550
   ```

### Waypoints Not Converting Correctly

**Symptoms:** Coordinates seem wrong or waypoints are at (0,0,0)

**Solutions:**
1. Ensure PX4 has GPS lock or simulated GPS enabled
2. Check bridge logs for home position:
   ```
   Home position set: lat=47.397742, lon=8.545594, alt=500.00
   ```
3. Verify global position is being received:
   ```bash
   ros2 topic echo /fmu/out/vehicle_global_position
   ```

### QGC Can't Connect to PX4

**Symptoms:** QGC shows "No connection" or "No heartbeat"

**Solutions:**
1. Verify PX4 MAVLink is running:
   ```bash
   # Check PX4 logs for MAVLink messages
   ```
2. Check Tailscale connection from MacBook to Jetson
3. Verify UDP port 14540 is not blocked by firewall
4. Try connecting from MacBook terminal:
   ```bash
   # On MacBook
   nc -u <jetson_tailscale_ip> 14540
   ```

## Coordinate System Notes

- **QGC uses**: Global WGS84 coordinates (latitude, longitude, altitude)
- **Maze navigator uses**: Local NED coordinates (North, East, Down)
- **Bridge converts**: Global → Local NED using home position as reference
- **Home position**: Set automatically from first valid GPS position

## Advanced Configuration

### Custom MAVLink Port

If PX4 is configured to use a different MAVLink port:

```bash
ros2 run px4_ros_com qgc_waypoint_bridge.py --mavlink-port udp:14550
```

### Serial MAVLink Connection

For serial connections (not typical for SITL):

```bash
ros2 run px4_ros_com qgc_waypoint_bridge.py --mavlink-port /dev/ttyUSB0 --mavlink-baud 57600
```

## See Also

- [Maze Navigator Waypoints Guide](MAZE_NAVIGATOR_WAYPOINTS.md) - General waypoint usage
- [PX4 MAVLink Documentation](https://dev.px4.io/v1.13/en/middleware/mavlink.html)
