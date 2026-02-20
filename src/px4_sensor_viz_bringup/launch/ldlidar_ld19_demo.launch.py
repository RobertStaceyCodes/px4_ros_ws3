#!/usr/bin/env python3
"""
Launch file to demonstrate the LDLiDAR LD19 360° lidar.

Starts the ldlidar_component in a composable-node container,
publishes the URDF via robot_state_publisher, manages lifecycle
transitions (configure → activate), and opens RViz2 with a
preconfigured laser-scan view.

No nav2_lifecycle_manager dependency required.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    LogInfo,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def _setup(context, *args, **kwargs):
    serial_port = LaunchConfiguration("serial_port").perform(context)
    start_rviz = LaunchConfiguration("start_rviz").perform(context).lower() == "true"

    pkg_share = get_package_share_directory("px4_sensor_viz_bringup")
    ldlidar_share = get_package_share_directory("ldlidar_node")

    lidar_config = os.path.join(pkg_share, "config", "ldlidar_ld19.yaml")
    rviz_config = os.path.join(pkg_share, "config", "ldlidar_ld19_demo.rviz")

    urdf_path = os.path.join(ldlidar_share, "urdf", "ldlidar_descr.urdf.xml")
    with open(urdf_path, "r") as f:
        robot_desc = f.read()

    actions = [
        LogInfo(msg=f"[ldlidar_ld19_demo] serial_port={serial_port}"),
    ]

    # Robot state publisher (provides ldlidar_base → ldlidar_link TF)
    actions.append(
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="ldlidar_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_desc}],
        )
    )

    # Composable-node container with the LDLidar component
    actions.append(
        ComposableNodeContainer(
            name="ldlidar_container",
            namespace="",
            package="rclcpp_components",
            executable="component_container_isolated",
            composable_node_descriptions=[
                ComposableNode(
                    package="ldlidar_component",
                    plugin="ldlidar::LdLidarComponent",
                    name="ldlidar_node",
                    parameters=[
                        lidar_config,
                        {"comm.serial_port": serial_port},
                    ],
                    extra_arguments=[{"use_intra_process_comms": True}],
                ),
            ],
            output="screen",
        )
    )

    # Lifecycle: configure after 3 s, activate after 5 s
    actions.append(
        TimerAction(
            period=3.0,
            actions=[
                LogInfo(msg="[ldlidar_ld19_demo] Configuring ldlidar_node ..."),
                ExecuteProcess(
                    cmd=["ros2", "lifecycle", "set", "/ldlidar_node", "configure"],
                    output="screen",
                ),
            ],
        )
    )
    actions.append(
        TimerAction(
            period=5.0,
            actions=[
                LogInfo(msg="[ldlidar_ld19_demo] Activating ldlidar_node ..."),
                ExecuteProcess(
                    cmd=["ros2", "lifecycle", "set", "/ldlidar_node", "activate"],
                    output="screen",
                ),
            ],
        )
    )

    if start_rviz:
        actions.append(
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
            )
        )

    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            SetEnvironmentVariable(name="RCUTILS_COLORIZED_OUTPUT", value="1"),
            DeclareLaunchArgument(
                "serial_port",
                default_value="/dev/ttyUSB0",
                description="Serial port for the LD19 lidar",
            ),
            DeclareLaunchArgument(
                "start_rviz",
                default_value="true",
                description="Start RViz2 with laser scan display",
            ),
            OpaqueFunction(function=_setup),
        ]
    )
