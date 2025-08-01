#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments - Model parameters
        DeclareLaunchArgument('sam_model_name', default_value='FastSAM-x.pt', description='SAM model name'),
        DeclareLaunchArgument('sam_conf', default_value='0.4', description='SAM confidence threshold'),
        DeclareLaunchArgument('sam_iou', default_value='0.9', description='SAM IoU threshold'),
        DeclareLaunchArgument('owl_model_name', default_value='google/owlvit-base-patch32', description='OWL model name'),
        DeclareLaunchArgument('owl_image_encoder_path', default_value='/opt/nanoowl/data/owl_image_encoder_patch32.engine', description='OWL image encoder path'),
        DeclareLaunchArgument('owl_threshold', default_value='0.1', description='OWL threshold'),
        DeclareLaunchArgument('owl_roi', default_value='[0.25, 0.50, 0.75, 0.95]', description='OWL ROI'),

        # Prompt parameters
        DeclareLaunchArgument('prompt_type', default_value='bbox', description='Prompt type'),
        DeclareLaunchArgument('prompt_bbox', default_value='[0.30, 0.50, 0.70, 0.90]', description='Prompt bounding box'),
        DeclareLaunchArgument('prompt_points', default_value='[[0.50, 0.90]]', description='Prompt points'),
        DeclareLaunchArgument('prompt_text', default_value='a sidewalk or footpath or walkway or paved path', description='Prompt text'),
        DeclareLaunchArgument('use_bbox_fallback', default_value='true', description='Use bbox fallback'),

        # Other parameters
        DeclareLaunchArgument('use_cuda', default_value='true', description='Use CUDA'),
        DeclareLaunchArgument('brightness_window', default_value='0.5', description='Brightness window'),
        DeclareLaunchArgument('mean_brightness', default_value='0.5', description='Mean brightness'),
        DeclareLaunchArgument('frame_id', default_value='', description='Frame ID'),
        DeclareLaunchArgument('verbose', default_value='false', description='Verbose output'),

        # Publish parameters
        DeclareLaunchArgument('publish_mask', default_value='true', description='Publish mask'),
        DeclareLaunchArgument('publish_image', default_value='true', description='Publish image'),
        DeclareLaunchArgument('publish_pointcloud', default_value='true', description='Publish pointcloud'),

        # Consumed topics
        DeclareLaunchArgument('rgb_topic', default_value='/zed/zed_node/rgb/image_rect_color', description='RGB topic'),
        DeclareLaunchArgument('pointcloud_topic', default_value='/zed/zed_node/point_cloud/cloud_registered', description='Pointcloud topic'),

        # Produced topics
        DeclareLaunchArgument('sidewalk_mask_topic', default_value='sidewalk_mask', description='Sidewalk mask topic'),
        DeclareLaunchArgument('sidewalk_image_topic', default_value='sidewalk_image', description='Sidewalk image topic'),
        DeclareLaunchArgument('sidewalk_pointcloud_topic', default_value='sidewalk_pointcloud', description='Sidewalk pointcloud topic'),

        # Launch the sidewalk_segmentation node
        Node(
            package='svea_vision',
            executable='segment_anything',
            name='sidewalk_segmentation',
            output='screen',
            parameters=[{
                # Model parameters
                'sam_model_name': LaunchConfiguration('sam_model_name'),
                'sam_conf': LaunchConfiguration('sam_conf'),
                'sam_iou': LaunchConfiguration('sam_iou'),
                'owl_model_name': LaunchConfiguration('owl_model_name'),
                'owl_image_encoder_path': LaunchConfiguration('owl_image_encoder_path'),
                'owl_threshold': LaunchConfiguration('owl_threshold'),
                'owl_roi': LaunchConfiguration('owl_roi'),
                # Prompt parameters
                'prompt_type': LaunchConfiguration('prompt_type'),
                'prompt_bbox': LaunchConfiguration('prompt_bbox'),
                'prompt_points': LaunchConfiguration('prompt_points'),
                'prompt_text': LaunchConfiguration('prompt_text'),
                'use_bbox_fallback': LaunchConfiguration('use_bbox_fallback'),
                # Other parameters
                'use_cuda': LaunchConfiguration('use_cuda'),
                'brightness_window': LaunchConfiguration('brightness_window'),
                'mean_brightness': LaunchConfiguration('mean_brightness'),
                'frame_id': LaunchConfiguration('frame_id'),
                'verbose': LaunchConfiguration('verbose'),
                # Publish parameters
                'publish_mask': LaunchConfiguration('publish_mask'),
                'publish_image': LaunchConfiguration('publish_image'),
                'publish_pointcloud': LaunchConfiguration('publish_pointcloud'),
                # Consumed topics
                'rgb_topic': LaunchConfiguration('rgb_topic'),
                'pointcloud_topic': LaunchConfiguration('pointcloud_topic'),
                # Produced topics
                'segmented_mask_topic': LaunchConfiguration('sidewalk_mask_topic'),
                'segmented_image_topic': LaunchConfiguration('sidewalk_image_topic'),
                'segmented_pointcloud_topic': LaunchConfiguration('sidewalk_pointcloud_topic'),
            }]
        )
    ])