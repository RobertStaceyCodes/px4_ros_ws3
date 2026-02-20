#!/usr/bin/env python3
"""
Launch file to demonstrate the Luxonis OAK-D Lite camera.

Starts the depthai_ros_driver with RGBD pipeline, generates a
depth-only point cloud (XYZ, no colour), publishes the camera
URDF, and opens RViz2 showing the RGB image + 3-D point cloud.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode, ParameterFile


def _setup(context, *args, **kwargs):
    start_rviz = LaunchConfiguration("start_rviz").perform(context).lower() == "true"

    pkg_share = get_package_share_directory("px4_sensor_viz_bringup")
    depthai_desc_share = get_package_share_directory("depthai_descriptions")

    params_file = os.path.join(pkg_share, "config", "oak_d_lite_demo.yaml")
    rviz_config = os.path.join(pkg_share, "config", "oak_d_lite_demo.rviz")

    name = "oak"
    camera_model = "OAK-D-LITE"

    actions = []

    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(depthai_desc_share, "launch", "urdf_launch.py")
            ),
            launch_arguments={
                "tf_prefix": name,
                "camera_model": camera_model,
                "base_frame": name,
                "parent_frame": "oak-d-base-frame",
                "cam_pos_x": "0.0",
                "cam_pos_y": "0.0",
                "cam_pos_z": "0.0",
                "cam_roll": "0.0",
                "cam_pitch": "0.0",
                "cam_yaw": "0.0",
                "use_composition": "true",
            }.items(),
        )
    )

    # Multi-threaded container so camera + point cloud run in parallel
    actions.append(
        ComposableNodeContainer(
            name=f"{name}_container",
            namespace="",
            package="rclcpp_components",
            executable="component_container_isolated",
            composable_node_descriptions=[
                ComposableNode(
                    package="depthai_ros_driver",
                    plugin="depthai_ros_driver::Camera",
                    name=name,
                    parameters=[
                        ParameterFile(params_file, allow_substs=True),
                    ],
                ),
                ComposableNode(
                    package="depth_image_proc",
                    plugin="depth_image_proc::PointCloudXyzNode",
                    name="point_cloud_xyz_node",
                    remappings=[
                        ("image_rect", f"{name}/stereo/image_raw"),
                        ("camera_info", f"{name}/stereo/camera_info"),
                        ("points", f"{name}/points"),
                    ],
                ),
            ],
            output="both",
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
            DeclareLaunchArgument(
                "start_rviz",
                default_value="true",
                description="Launch RViz2 with RGB image and depth point cloud",
            ),
            OpaqueFunction(function=_setup),
        ]
    )
