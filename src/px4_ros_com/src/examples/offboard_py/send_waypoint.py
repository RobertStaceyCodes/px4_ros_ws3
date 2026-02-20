#!/usr/bin/env python3
"""
Helper script to send waypoint commands to the maze navigator.

Usage:
    python3 send_waypoint.py <x> <y> [z]

Example:
    python3 send_waypoint.py 0.0 24.0
    python3 send_waypoint.py 10.0 15.0 0.0
"""

import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Header


def main():
    if len(sys.argv) < 3:
        print("Usage: send_waypoint.py <x> <y> [z]")
        print("  x, y: NED coordinates in meters")
        print("  z: optional z coordinate (default: 0.0)")
        sys.exit(1)
    
    x = float(sys.argv[1])
    y = float(sys.argv[2])
    z = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0
    
    rclpy.init()
    node = Node("waypoint_sender")
    
    pub = node.create_publisher(
        PointStamped,
        "/maze_navigator/set_goal",
        10
    )
    
    # Wait for subscriber
    import time
    time.sleep(0.5)
    
    msg = PointStamped()
    msg.header = Header()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.header.frame_id = "map"
    msg.point.x = x
    msg.point.y = y
    msg.point.z = z
    
    pub.publish(msg)
    node.get_logger().info(f"Sent waypoint: NED ({x:.2f}, {y:.2f}, {z:.2f})")
    
    time.sleep(0.1)  # Ensure message is sent
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
