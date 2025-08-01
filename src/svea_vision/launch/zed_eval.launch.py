#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument('mocap_name', default_value='rsu', description='Motion capture name'),
        DeclareLaunchArgument('use_cuda', default_value='true', description='Use CUDA'),
        DeclareLaunchArgument('enable_bbox_image', default_value='true', description='Enable bbox image'),
        DeclareLaunchArgument('enable_aruco', default_value='false', description='Enable ArUco detection'),
        DeclareLaunchArgument('frame_id', default_value='mocap', description='Frame ID'),
        DeclareLaunchArgument('only_objects', default_value='', description='Only detect specified objects'),
        DeclareLaunchArgument('skip_objects', default_value='', description='Skip specified objects'),
        DeclareLaunchArgument('max_age', default_value='30', description='Maximum tracking age'),
        DeclareLaunchArgument('aruco_size', default_value='0.1', description='ArUco marker size'),
        DeclareLaunchArgument('mocap_address', default_value='10.0.0.10', description='Motion capture address'),
        DeclareLaunchArgument('map_path', default_value='', description='Map file path'),

        # Constants (topics)
        DeclareLaunchArgument('depth_image', default_value='/zed/zed_node/depth/depth_registered', description='Depth image topic'),
        DeclareLaunchArgument('image', default_value='/zed/zed_node/rgb/image_rect_color', description='RGB image topic'),

        # ZED Camera launch (commented out - needs zed_wrapper package for ROS2)
        # Note: This would need to be replaced with the ROS2 version of zed_wrapper
        # IncludeLaunchDescription(
        #     PathJoinSubstitution([
        #         FindPackageShare('zed_wrapper'),
        #         'launch',
        #         'zed.launch.py'
        #     ]),
        #     launch_arguments={
        #         'camera_name': 'zed',
        #     }.items()
        # ),

        # Conditional map server (commented out - needs nav2_map_server package)
        # Node(
        #     condition=IfCondition(LaunchConfiguration('map_path')),
        #     package='nav2_map_server',
        #     executable='map_server',
        #     name='map_server',
        #     parameters=[{'yaml_filename': LaunchConfiguration('map_path')}]
        # ),

        # Detection and state estimation nodes
        Node(
            package='svea_vision',
            executable='detection_splitter',
            name='detection_splitter',
            output='screen'
        ),

        Node(
            package='svea_vision',
            executable='person_state_estimation',
            name='person_state_estimation',
            output='screen'
        ),

        # Include object_pose launch
        IncludeLaunchDescription(
            PathJoinSubstitution([
                FindPackageShare('svea_vision'),
                'launch',
                'object_pose.launch.py'
            ]),
            launch_arguments={
                'use_cuda': LaunchConfiguration('use_cuda'),
                'enable_bbox_image': LaunchConfiguration('enable_bbox_image'),
                'frame_id': LaunchConfiguration('frame_id'),
                'image': LaunchConfiguration('image'),
                'depth_image': LaunchConfiguration('depth_image'),
                'only_objects': LaunchConfiguration('only_objects'),
                'skip_objects': LaunchConfiguration('skip_objects'),
                'max_age': LaunchConfiguration('max_age'),
            }.items()
        ),

        # Motion Capture System (commented out - needs mocap_qualisys package for ROS2)
        # Note: This would need to be replaced with the ROS2 version of mocap_qualisys
        # IncludeLaunchDescription(
        #     PathJoinSubstitution([
        #         FindPackageShare('mocap_qualisys'),
        #         'launch',
        #         'qualisys.launch.py'
        #     ]),
        #     launch_arguments={
        #         'server_address': LaunchConfiguration('mocap_address'),
        #     }.items()
        # ),

        # Record rosbag (ROS2 uses ros2 bag record instead of rosbag record)
        # Note: Uncomment and modify as needed for your specific recording requirements
        # ExecuteProcess(
        #     cmd=[
        #         FindExecutable(name='ros2'),
        #         'bag', 'record',
        #         '-o', PathJoinSubstitution([FindPackageShare('svea_vision'), 'out']),
        #         '/objectposes',
        #         '/person_state_estimation/person_states',
        #         '/qualisys/tinman/pose',
        #         '/qualisys/tinman/velocity',
        #         '/qualisys/pedestrian/pose',
        #         '/qualisys/pedestrian/velocity'
        #     ],
        #     output='screen'
        # )
    ])
