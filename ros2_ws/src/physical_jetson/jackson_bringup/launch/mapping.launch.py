#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():

    jackson_bringup_share = get_package_share_directory(
        "jackson_bringup"
    )

    state_estimation_launch = os.path.join(
        jackson_bringup_share,
        "launch",
        "state_estimation.launch.py",
    )

    slam_config = os.path.join(
        jackson_bringup_share,
        "config",
        "slam.yaml",
    )

    # Hardware + TF estáticas + EKF.
    state_estimation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            state_estimation_launch
        )
    )

    # RPLIDAR A1 conectado mediante CP2102.
    # Se usa by-id para que el puerto sea estable.
    rplidar = Node(
        package="rplidar_ros",
        executable="rplidar_node",
        name="rplidar_node",
        output="screen",
        parameters=[
            {
                "channel_type": "serial",
                "serial_port": (
                    "/dev/serial/by-id/"
                    "usb-Silicon_Labs_CP2102_"
                    "USB_to_UART_Bridge_Controller_"
                    "0001-if00-port0"
                ),
                "serial_baudrate": 115200,
                "frame_id": "laser",
                "inverted": False,
                "angle_compensate": True,
                "scan_mode": "Standard",
            }
        ],
    )

    # SLAM Toolbox oficial en modo online asíncrono.
    slam_toolbox = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[
            slam_config,
            {
                "use_sim_time": False,
            },
        ],
    )

    return LaunchDescription(
        [
            state_estimation,
            rplidar,
            slam_toolbox,
        ]
    )
