#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context, *args, **kwargs):
    world = LaunchConfiguration("world").perform(context)
    model_instance = LaunchConfiguration("model_instance").perform(context)
    gz_partition = LaunchConfiguration("gz_partition").perform(context)
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context).lower() == "true"
    rviz_config = LaunchConfiguration("rviz_config").perform(context)
    start_bridge = LaunchConfiguration("start_bridge").perform(context).lower() == "true"
    start_rviz = LaunchConfiguration("start_rviz").perform(context).lower() == "true"
    start_static_tf = LaunchConfiguration("start_static_tf").perform(context).lower() == "true"
    start_tf_sanitizer = LaunchConfiguration("start_tf_sanitizer").perform(context).lower() == "true"

    root = f"/world/{world}/model/{model_instance}"
    actions = [
        LogInfo(msg=f"[px4_sensor_viz_bringup] world={world}"),
        LogInfo(msg=f"[px4_sensor_viz_bringup] model={model_instance}"),
    ]

    if start_bridge:
        bridge_args = [
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            f"/world/{world}/dynamic_pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
            f"{root}/link/link/sensor/lidar_2d_v2/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            f"{root}/link/tf_luna_down_link/sensor/tf_luna_down/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            f"{root}/link/tf_luna_up_link/sensor/tf_luna_up/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            f"{root}/link/camera_link/sensor/IMX214/image@sensor_msgs/msg/Image[gz.msgs.Image",
            f"{root}/link/camera_link/sensor/IMX214/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            "/depth_camera@sensor_msgs/msg/Image[gz.msgs.Image",
            "/depth_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            "/depth_camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
        ]
        bridge_remaps = [
            (f"/world/{world}/dynamic_pose/info", "/tf_raw"),
            (f"{root}/link/link/sensor/lidar_2d_v2/scan", "/d500/scan"),
            (f"{root}/link/tf_luna_down_link/sensor/tf_luna_down/scan", "/tf_luna/down/scan"),
            (f"{root}/link/tf_luna_up_link/sensor/tf_luna_up/scan", "/tf_luna/up/scan"),
            (f"{root}/link/camera_link/sensor/IMX214/image", "/oak/rgb/image_raw"),
            (f"{root}/link/camera_link/sensor/IMX214/camera_info", "/oak/rgb/camera_info"),
            ("/depth_camera", "/oak/depth/image_raw"),
            ("/depth_camera/camera_info", "/oak/depth/camera_info"),
            ("/depth_camera/points", "/oak/depth/points"),
        ]
        actions.append(
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="px4_sitl_sensor_bridge",
                output="screen",
                arguments=bridge_args,
                parameters=[{"use_sim_time": use_sim_time, "expand_gz_topic_names": True}],
                remappings=bridge_remaps,
                additional_env={"GZ_PARTITION": gz_partition} if gz_partition else {},
            )
        )

    if start_tf_sanitizer:
        actions.append(
            Node(
                package="px4_sensor_viz_bringup",
                executable="tf_sanitizer.py",
                name="tf_sanitizer",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
            )
        )

    if start_static_tf:
        static_tf_nodes = [
            ("0.12", "0.03", "0.242", "0", "0", "0", "base_link", "camera_link"),
            ("-0.1", "0", "0.26", "0", "0", "0", "base_link", "link"),
            ("0", "0", "-0.05", "0", "1.57", "0", "base_link", "tf_luna_down_link"),
            ("0", "0", "0.15", "0", "-1.57", "0", "base_link", "tf_luna_up_link"),
        ]
        for i, tf in enumerate(static_tf_nodes):
            actions.append(
                Node(
                    package="tf2_ros",
                    executable="static_transform_publisher",
                    name=f"sitl_static_tf_{i}",
                    output="screen",
                    arguments=list(tf),
                )
            )

    if start_rviz:
        actions.append(
            Node(
                package="rviz2",
                executable="rviz2",
                name="sitl_sensor_rviz",
                output="screen",
                arguments=["-d", rviz_config],
                parameters=[{"use_sim_time": use_sim_time}],
            )
        )

    return actions


def generate_launch_description():
    default_rviz = os.path.join(
        get_package_share_directory("px4_sensor_viz_bringup"),
        "config",
        "sitl_oak_lidar.rviz",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("world", default_value="default"),
            DeclareLaunchArgument("model_instance", default_value="x500_oak_tfluna_d500_0"),
            DeclareLaunchArgument("gz_partition", default_value=""),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("start_bridge", default_value="true"),
            DeclareLaunchArgument("start_tf_sanitizer", default_value="true"),
            DeclareLaunchArgument("start_rviz", default_value="true"),
            DeclareLaunchArgument("start_static_tf", default_value="true"),
            DeclareLaunchArgument("rviz_config", default_value=default_rviz),
            OpaqueFunction(function=_setup),
        ]
    )
