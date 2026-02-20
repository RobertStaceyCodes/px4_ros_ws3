#!/usr/bin/env python3
"""
QGC Waypoint Bridge for Maze Navigator
=======================================

This node bridges waypoints from QGroundControl (via MAVLink) to the maze navigator.
It listens for MISSION_ITEM_INT messages from QGC and converts them to local NED
coordinates for the maze navigator.

Usage:
    ros2 run px4_ros_com qgc_waypoint_bridge.py [--mavlink-port PORT] [--mavlink-baud BAUDRATE]

The bridge connects to PX4's MAVLink stream and forwards waypoint commands to
/maze_navigator/set_goal when QGC sends mission items.

Requirements:
    pip install pymavlink
"""

import argparse
import math
import sys
import time
from typing import Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Header

try:
    from pymavlink import mavutil
except ImportError:
    print("ERROR: pymavlink not installed. Install with: pip install pymavlink")
    sys.exit(1)

from px4_msgs.msg import VehicleGlobalPosition, VehicleLocalPosition


class QGCWaypointBridge(Node):
    """Bridge QGC waypoints (MAVLink) to maze navigator (ROS2)."""

    def __init__(self, mavlink_port: str = "udp:14540", mavlink_baud: int = 57600):
        super().__init__("qgc_waypoint_bridge")
        
        self.mavlink_port = mavlink_port
        self.mavlink_baud = mavlink_baud
        
        # --- ROS2 publishers ---
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._pub_goal = self.create_publisher(
            PointStamped, "/maze_navigator/set_goal", map_qos
        )
        
        # --- ROS2 subscribers for coordinate conversion ---
        sub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(
            VehicleGlobalPosition,
            "/fmu/out/vehicle_global_position",
            self._cb_global_pos,
            sub_qos,
        )
        self.create_subscription(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position",
            self._cb_local_pos,
            sub_qos,
        )
        
        # --- State ---
        self._home_lat: Optional[float] = None
        self._home_lon: Optional[float] = None
        self._home_alt: Optional[float] = None
        self._current_lat: Optional[float] = None
        self._current_lon: Optional[float] = None
        self._current_alt: Optional[float] = None
        self._local_pos_valid = False
        self._global_pos_valid = False
        
        # --- MAVLink connection ---
        self._mavlink_conn: Optional[mavutil.mavlink_connection] = None
        self._last_mission_seq = -1
        
        # --- Timer for MAVLink message processing ---
        self.create_timer(0.1, self._process_mavlink)  # 10 Hz
        
        # --- Tick counter for reconnection attempts ---
        self._tick = 0
        
        self.get_logger().info(
            f"QGC Waypoint Bridge initialized — connecting to MAVLink on {mavlink_port}"
        )
        self._connect_mavlink()
    
    def _connect_mavlink(self) -> None:
        """Connect to MAVLink stream."""
        try:
            if self.mavlink_port.startswith("udp"):
                # UDP connection (e.g., "udp:14540" or "udp:0.0.0.0:14540")
                parts = self.mavlink_port.split(":")
                if len(parts) == 2:
                    port = int(parts[1])
                    self._mavlink_conn = mavutil.mavlink_connection(
                        f"udp:0.0.0.0:{port}", baud=self.mavlink_baud
                    )
                elif len(parts) == 3:
                    host, port = parts[1], int(parts[2])
                    self._mavlink_conn = mavutil.mavlink_connection(
                        f"udp:{host}:{port}", baud=self.mavlink_baud
                    )
                else:
                    self._mavlink_conn = mavutil.mavlink_connection(
                        self.mavlink_port, baud=self.mavlink_baud
                    )
            else:
                # Serial or TCP connection
                self._mavlink_conn = mavutil.mavlink_connection(
                    self.mavlink_port, baud=self.mavlink_baud
                )
            
            self.get_logger().info(f"Connected to MAVLink on {self.mavlink_port}")
            # Wait for heartbeat to confirm connection
            self._mavlink_conn.wait_heartbeat(timeout=5.0)
            self.get_logger().info("MAVLink heartbeat received")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to MAVLink: {e}")
            self.get_logger().warn(
                "Will retry connection. Make sure PX4 MAVLink is running and "
                f"listening on {self.mavlink_port}"
            )
            self._mavlink_conn = None
    
    def _cb_global_pos(self, msg: VehicleGlobalPosition) -> None:
        """Update global position for coordinate conversion."""
        if msg.timestamp > 0:  # Valid timestamp
            self._current_lat = msg.lat
            self._current_lon = msg.lon
            self._current_alt = msg.alt
            self._global_pos_valid = True
            
            # Use first valid position as home if not set
            if self._home_lat is None:
                self._home_lat = msg.lat
                self._home_lon = msg.lon
                self._home_alt = msg.alt
                self.get_logger().info(
                    f"Home position set: lat={self._home_lat:.6f}, "
                    f"lon={self._home_lon:.6f}, alt={self._home_alt:.2f}"
                )
    
    def _cb_local_pos(self, msg: VehicleLocalPosition) -> None:
        """Update local position."""
        if msg.xy_valid and msg.z_valid:
            self._local_pos_valid = True
    
    def _global_to_local_ned(
        self, lat: float, lon: float, alt: float
    ) -> Tuple[float, float, float]:
        """Convert global WGS84 coordinates to local NED.
        
        Uses a simple flat-earth approximation. For better accuracy over
        longer distances, use a proper geodetic conversion library.
        """
        if self._home_lat is None or self._home_lon is None:
            self.get_logger().warn(
                "Home position not set, cannot convert coordinates. "
                "Using (0, 0, 0) as fallback."
            )
            return (0.0, 0.0, 0.0)
        
        # Earth radius in meters
        R = 6378137.0
        
        # Convert to radians
        lat_home_rad = math.radians(self._home_lat)
        lon_home_rad = math.radians(self._home_lon)
        lat_rad = math.radians(lat)
        lon_rad = math.radians(lon)
        
        # Flat-earth approximation (good for distances < 10 km)
        d_lat = lat_rad - lat_home_rad
        d_lon = lon_rad - lon_home_rad
        
        # North (x) and East (y) in meters
        north = d_lat * R
        east = d_lon * R * math.cos(lat_home_rad)
        
        # Down (z) - positive down, negative up
        down = alt - (self._home_alt or 0.0)
        
        return (north, east, down)
    
    def _process_mavlink(self) -> None:
        """Process incoming MAVLink messages."""
        self._tick += 1
        
        if self._mavlink_conn is None:
            # Try to reconnect
            if self._tick % 50 == 0:  # Every 5 seconds
                self._connect_mavlink()
            return
        
        try:
            # Read all available messages
            while True:
                msg = self._mavlink_conn.recv_match(
                    type=["MISSION_ITEM_INT", "MISSION_CURRENT", "HEARTBEAT"],
                    blocking=False,
                    timeout=0.01,
                )
                
                if msg is None:
                    break
                
                if msg.get_type() == "MISSION_ITEM_INT":
                    self._handle_mission_item(msg)
                elif msg.get_type() == "MISSION_CURRENT":
                    # Track current mission sequence
                    if hasattr(msg, "seq"):
                        self._last_mission_seq = msg.seq
                        self.get_logger().debug(f"Current mission seq: {msg.seq}")
                elif msg.get_type() == "HEARTBEAT":
                    # Connection is alive
                    pass
                    
        except Exception as e:
            self.get_logger().error(f"Error processing MAVLink message: {e}", exc_info=True)
            # Try to reconnect on error
            self._mavlink_conn = None
    
    def _handle_mission_item(self, msg) -> None:
        """Handle MISSION_ITEM_INT message from QGC."""
        # Only process waypoint commands (MAV_CMD_NAV_WAYPOINT = 16)
        if msg.command != 16:  # MAV_CMD_NAV_WAYPOINT
            return
        
        # Check if this is a new waypoint (not the same as last processed)
        if msg.seq == self._last_mission_seq:
            return  # Already processed
        
        # Convert coordinates based on frame
        if msg.frame in [
            mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        ]:
            # Global coordinates (lat/lon in degrees * 1e7)
            lat = msg.x / 1e7
            lon = msg.y / 1e7
            alt = msg.z  # Altitude in meters
            
            # Convert to local NED
            ned_x, ned_y, ned_z = self._global_to_local_ned(lat, lon, alt)
            
            self.get_logger().info(
                f"Received waypoint from QGC: seq={msg.seq}, "
                f"global=({lat:.6f}, {lon:.6f}, {alt:.2f}), "
                f"NED=({ned_x:.2f}, {ned_y:.2f}, {ned_z:.2f})"
            )
            
            # Publish to maze navigator
            goal_msg = PointStamped()
            goal_msg.header = Header()
            goal_msg.header.stamp = self.get_clock().now().to_msg()
            goal_msg.header.frame_id = "map"
            goal_msg.point.x = ned_x
            goal_msg.point.y = ned_y
            goal_msg.point.z = ned_z
            
            self._pub_goal.publish(goal_msg)
            self._last_mission_seq = msg.seq
            
        elif msg.frame == mavutil.mavlink.MAV_FRAME_LOCAL_NED:
            # Already in local NED, use directly
            self.get_logger().info(
                f"Received local NED waypoint from QGC: seq={msg.seq}, "
                f"NED=({msg.x:.2f}, {msg.y:.2f}, {msg.z:.2f})"
            )
            
            goal_msg = PointStamped()
            goal_msg.header = Header()
            goal_msg.header.stamp = self.get_clock().now().to_msg()
            goal_msg.header.frame_id = "map"
            goal_msg.point.x = msg.x
            goal_msg.point.y = msg.y
            goal_msg.point.z = msg.z
            
            self._pub_goal.publish(goal_msg)
            self._last_mission_seq = msg.seq
        else:
            self.get_logger().warn(
                f"Unsupported mission frame: {msg.frame} for waypoint seq={msg.seq}"
            )


def main(args=None):
    parser = argparse.ArgumentParser(
        description="Bridge QGC waypoints (MAVLink) to maze navigator (ROS2)"
    )
    parser.add_argument(
        "--mavlink-port",
        type=str,
        default="udp:14540",
        help="MAVLink connection string (default: udp:14540)",
    )
    parser.add_argument(
        "--mavlink-baud",
        type=int,
        default=57600,
        help="MAVLink baud rate for serial connections (default: 57600)",
    )
    
    # Parse ROS2 args
    rclpy.init(args=args)
    parsed_args, unknown = parser.parse_known_args(args)
    
    node = QGCWaypointBridge(
        mavlink_port=parsed_args.mavlink_port,
        mavlink_baud=parsed_args.mavlink_baud,
    )
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
