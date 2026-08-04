import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('jackson_slam')

    ekf_config = os.path.join(pkg_dir, 'config', 'ekf_wheel.yaml')
    slam_config = os.path.join(pkg_dir, 'config', 'slam_toolbox.yaml')

    return LaunchDescription([

        # TF estático: base_link -> lidar_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_lidar_tf',
            arguments=[
                '0', '0', '0.170',
                '0', '0', '0',
                'base_link', 'lidar_link'
            ],
            parameters=[{'use_sim_time': True}]
        ),

        # TF estático: base_link -> imu_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_imu_tf',
            arguments=[
                '0', '0', '0.100',
                '0', '0', '0',
                'base_link', 'imu_link'
            ],
            parameters=[{'use_sim_time': True}]
        ),

        # Convierte /joint_states en /wheel/odom
        Node(
            package='jackson_slam',
            executable='jointstates_to_wheel_odom',
            name='jointstates_to_wheel_odom',
            output='screen',
            parameters=[{
                'use_sim_time': True,

                'left_joint_name': 'left_wheel_joint',
                'right_joint_name': 'right_wheel_joint',

                'wheel_radius': 0.033,
                'wheel_separation': 0.192,

                'odom_frame': 'odom',
                'base_frame': 'base_link',
                'odom_topic': '/wheel/odom',

                # Importante: False porque robot_localization publicará odom -> base_link
                'publish_tf': False,

                # Signos iniciales. Si avanza al revés o gira invertido, se ajustan luego.
                'left_sign': 1.0,
                'right_sign': 1.0,
                'yaw_sign': 1.0,
            }]
        ),

        # EKF: recibe /wheel/odom y publica /odometry/filtered + TF odom -> base_link
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config]
        ),

        # SLAM Toolbox: recibe /scan y usa TF map -> odom -> base_link -> lidar_link
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_config]
        ),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            parameters=[{'use_sim_time': True}]
        )
    ])
