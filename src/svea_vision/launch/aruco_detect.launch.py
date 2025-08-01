#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument('image', description='Input image topic'),
        DeclareLaunchArgument('aruco_pose', default_value='aruco_pose', description='Output aruco pose topic'),
        DeclareLaunchArgument('aruco_dict', default_value='DICT_4X4_250', description='ArUco dictionary'),
        DeclareLaunchArgument('aruco_size', default_value='0.1', description='ArUco marker size'),
        DeclareLaunchArgument('aruco_tf_name', default_value='aruco', description='ArUco TF frame name'),

        # Launch the aruco_detect node
        Node(
            package='svea_vision',
            executable='aruco_detect',
            name='aruco_detect',
            output='screen',
            parameters=[{
                'sub_image': LaunchConfiguration('image'),
                'pub_aruco_pose': LaunchConfiguration('aruco_pose'),
                'aruco_dict': LaunchConfiguration('aruco_dict'),
                'aruco_size': LaunchConfiguration('aruco_size'),
                'aruco_tf_name': LaunchConfiguration('aruco_tf_name'),
            }]
        )
    ])
