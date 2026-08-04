#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,
)
from launch_ros.actions import Node


def generate_launch_description():

    bringup_directory = get_package_share_directory(
        "jackson_bringup"
    )

    hardware_launch = os.path.join(
        bringup_directory,
        "launch",
        "hardware.launch.py",
    )

    ekf_config = os.path.join(
        bringup_directory,
        "config",
        "ekf.yaml",
    )

    start_hardware = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            hardware_launch
        )
    )

    base_to_body = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_basefootprint_to_baselink",
        arguments=[
            "--frame-id", "base_footprint",
            "--child-frame-id", "base_link",
            "--x", "0.0",
            "--y", "0.0",
            "--z", "0.033",
            "--roll", "0.0",
            "--pitch", "0.0",
            "--yaw", "0.0",
        ],
    )

    base_to_laser = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_baselink_to_laser",
        arguments=[
            "--frame-id", "base_link",
            "--child-frame-id", "laser",
            "--x", "0.0",
            "--y", "0.0",
            "--z", "0.214",
            "--roll", "0.0",
            "--pitch", "0.0",
            "--yaw", "3.141592653589793",
        ],
    )

    body_to_imu = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_baselink_to_imu",
        arguments=[
            "--frame-id", "base_link",
            "--child-frame-id", "imu_link",
            "--x", "-0.050",
            "--y", "-0.045",
            "--z", "0.139",
            "--roll", "0.0",
            "--pitch", "0.0",
            "--yaw", "0.0",
        ],
    )

    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[ekf_config],
    )

    return LaunchDescription([
        start_hardware,
        base_to_body,
        base_to_laser,
        body_to_imu,
        ekf_node,
    ])
