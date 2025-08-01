#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument('use_cuda', default_value='true', description='Use CUDA for object detection'),
        DeclareLaunchArgument('enable_bbox_image', default_value='false', description='Enable bounding box image output'),
        DeclareLaunchArgument('frame_id', default_value='map', description='Frame ID for object poses'),
        DeclareLaunchArgument('image', default_value='image', description='Input image topic'),
        DeclareLaunchArgument('depth_image', default_value='depth_image', description='Input depth image topic'),
        DeclareLaunchArgument('objects', default_value='objects', description='Objects topic'),
        DeclareLaunchArgument('bbox_image', default_value='bbox_image', description='Output bounding box image topic'),
        DeclareLaunchArgument('objectposes', default_value='objectposes', description='Output object poses topic'),
        DeclareLaunchArgument('max_age', default_value='30', description='Maximum age for tracking'),
        DeclareLaunchArgument('model_path', default_value='yolov8n.pt', description='Path to YOLO model'),
        DeclareLaunchArgument('only_objects', default_value='', description='Only detect specified objects'),
        DeclareLaunchArgument('skip_objects', default_value='', description='Skip specified objects'),

        # Launch the object_detect node
        Node(
            package='svea_vision',
            executable='object_detect',
            name='object_detect',
            output='screen',
            parameters=[{
                'use_cuda': LaunchConfiguration('use_cuda'),
                'enable_bbox_image': LaunchConfiguration('enable_bbox_image'),
                'sub_image': LaunchConfiguration('image'),
                'pub_objects': LaunchConfiguration('objects'),
                'pub_bbox_image': LaunchConfiguration('bbox_image'),
                'only_objects': LaunchConfiguration('only_objects'),
                'skip_objects': LaunchConfiguration('skip_objects'),
                'max_age': LaunchConfiguration('max_age'),
                'model_path': LaunchConfiguration('model_path'),
            }]
        ),

        # Launch the object_pose node
        Node(
            package='svea_vision',
            executable='object_pose',
            name='object_pose',
            output='screen',
            parameters=[{
                'sub_depth_image': LaunchConfiguration('depth_image'),
                'sub_objects': LaunchConfiguration('objects'),
                'pub_objectposes': LaunchConfiguration('objectposes'),
                'frame_id': LaunchConfiguration('frame_id'),
            }]
        )
    ])
