#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    jackson_share = get_package_share_directory("jackson_bringup")
    nav2_share = get_package_share_directory("nav2_bringup")

    state_estimation_launch = os.path.join(
        jackson_share,
        "launch",
        "state_estimation.launch.py",
    )

    nav2_launch = os.path.join(
        nav2_share,
        "launch",
        "bringup_launch.py",
    )

    nav2_params = os.path.join(
        jackson_share,
        "config",
        "nav2_params.yaml",
    )

    default_map_yaml = os.path.expanduser(
        "~/jackson_dt_ws/maps/jackson_map_01.yaml"
    )

    declare_map = DeclareLaunchArgument(
        "map",
        default_value=default_map_yaml,
        description="Full path to map YAML file",
    )

    map_yaml = LaunchConfiguration("map")

    state_estimation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(state_estimation_launch)
    )

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

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch),
        launch_arguments={
            "map": map_yaml,
            "params_file": nav2_params,
            "use_sim_time": "false",
            "autostart": "true",
        }.items(),
    )

    return LaunchDescription(
        [
            declare_map,
            state_estimation,
            rplidar,
            navigation,
        ]
    )
