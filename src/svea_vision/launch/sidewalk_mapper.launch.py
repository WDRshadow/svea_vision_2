#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments - Occupancy Grid Parameters
        DeclareLaunchArgument('world_frame', default_value='map', description='World frame'),
        DeclareLaunchArgument('base_frame', default_value='base_link', description='Base frame'),
        DeclareLaunchArgument('resolution', default_value='0.05', description='Grid resolution'),
        DeclareLaunchArgument('width', default_value='50', description='Grid width'),
        DeclareLaunchArgument('height', default_value='50', description='Grid height'),
        DeclareLaunchArgument('grid_origin', default_value='bottom', description='Grid origin: "center" or "bottom"'),

        # Sidewalk parameters
        DeclareLaunchArgument('sidewalk_z_min', default_value='-0.5', description='Minimum Z for sidewalk'),
        DeclareLaunchArgument('sidewalk_z_max', default_value='0.5', description='Maximum Z for sidewalk'),
        DeclareLaunchArgument('non_sidewalk_z_min', default_value='-1.0', description='Minimum Z for non-sidewalk'),
        DeclareLaunchArgument('non_sidewalk_z_max', default_value='1.0', description='Maximum Z for non-sidewalk'),
        DeclareLaunchArgument('pointcloud_max_distance', default_value='7.5', description='Maximum distance to consider points in meters'),

        # Topic parameters
        DeclareLaunchArgument('pointcloud_topic', default_value='/zed/zed_node/point_cloud/cloud_registered', description='Pointcloud topic'),
        DeclareLaunchArgument('sidewalk_mask_topic', default_value='sidewalk_mask', description='Sidewalk mask topic'),
        DeclareLaunchArgument('sidewalk_occupancy_grid_topic', default_value='sidewalk_occupancy_grid', description='Sidewalk occupancy grid topic'),
        DeclareLaunchArgument('filtered_pose_topic', default_value='/zed/zed_node/pose', description='Filtered pose topic'),
        DeclareLaunchArgument('verbose', default_value='false', description='Verbose output'),

        # Launch the sidewalk_mapper node
        Node(
            package='svea_vision',
            executable='sidewalk_mapper',
            name='sidewalk_mapper',
            output='screen',
            parameters=[{
                # Occupancy Grid Parameters
                'world_frame': LaunchConfiguration('world_frame'),
                'base_frame': LaunchConfiguration('base_frame'),
                'resolution': LaunchConfiguration('resolution'),
                'width': LaunchConfiguration('width'),
                'height': LaunchConfiguration('height'),
                'grid_origin': LaunchConfiguration('grid_origin'),
                # Sidewalk parameters
                'sidewalk_z_min': LaunchConfiguration('sidewalk_z_min'),
                'sidewalk_z_max': LaunchConfiguration('sidewalk_z_max'),
                'non_sidewalk_z_min': LaunchConfiguration('non_sidewalk_z_min'),
                'non_sidewalk_z_max': LaunchConfiguration('non_sidewalk_z_max'),
                'pointcloud_max_distance': LaunchConfiguration('pointcloud_max_distance'),
                # Topic parameters
                'pointcloud_topic': LaunchConfiguration('pointcloud_topic'),
                'sidewalk_mask_topic': LaunchConfiguration('sidewalk_mask_topic'),
                'sidewalk_occupancy_grid_topic': LaunchConfiguration('sidewalk_occupancy_grid_topic'),
                'filtered_pose_topic': LaunchConfiguration('filtered_pose_topic'),
                'verbose': LaunchConfiguration('verbose'),
            }]
        )
    ])