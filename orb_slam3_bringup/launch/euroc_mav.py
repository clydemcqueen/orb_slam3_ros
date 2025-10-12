#!/usr/bin/env python3

"""
Launch ORB-SLAM3 mono or mono_imu with configuration for EuRoC MAV V1_01_easy
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    project_dir = get_package_share_directory('orb_slam3_bringup')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_imu_data',
            default_value='False',
            description='Use IMU data?'
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', os.path.join(project_dir, 'rviz', 'euroc_mav.rviz')],
        ),

        Node(
            package='orb_slam3_ros',
            executable='orb_slam3_ros_mono',
            output='screen',
            parameters=[{
                'settings_file': os.path.join(project_dir, 'param', 'euroc_mav.yaml'),
                'world_frame_id': 'map',
            }],
            remappings=[
                ('/image_raw', '/cam0/image_raw'),
            ],
            condition=UnlessCondition(LaunchConfiguration('use_imu_data')),
        ),

        Node(
            package='orb_slam3_ros',
            executable='orb_slam3_ros_mono_imu',
            output='screen',
            parameters=[{
                'settings_file': os.path.join(project_dir, 'param', 'euroc_mav.yaml'),
                'world_frame_id': 'map',
            }],
            remappings=[
                ('/image_raw', '/cam0/image_raw'),
                ('/imu', '/imu0'),
            ],
            condition=IfCondition(LaunchConfiguration('use_imu_data')),
        ),
    ])