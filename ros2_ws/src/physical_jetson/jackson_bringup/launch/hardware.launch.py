#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    bringup_directory = get_package_share_directory(
        "jackson_bringup"
    )

    hardware_config = os.path.join(
        bringup_directory,
        "config",
        "hardware.yaml",
    )

    esp32_node = Node(
        package="jackson_hardware",
        executable="esp32_odom_imu",
        name="esp32_odom_imu",
        output="screen",
        parameters=[hardware_config],
    )

    motor_node = Node(
        package="jackson_hardware",
        executable="motor_driver",
        name="jackson_motor_driver",
        output="screen",
        parameters=[hardware_config],
    )

    return LaunchDescription([
        esp32_node,
        motor_node,
    ])
