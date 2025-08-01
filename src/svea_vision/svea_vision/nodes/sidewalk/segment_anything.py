#! /usr/bin/env python3

__author__ = "Sulthan Suresh Fazeela"
__email__ = "sultha@kth.se"
__license__ = "MIT"

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2


def load_param(node, name, default_value=None):
    node.declare_parameter(name, default_value)
    return node.get_parameter(name).get_parameter_value()


class SegmentAnything(Node):
    """
    SegmentAnything class is a ROS2 node that segments an object in an image based on a prompt.
    This is a simplified stub implementation for ROS2 migration.
    The full implementation requires complex deep learning models (FastSAM, nanoOWL) that need
    careful integration and testing.
    
    TODO: Complete implementation with:
    - FastSAM model integration
    - nanoOWL predictor
    - Point cloud processing
    - TF2 transformations
    """
    
    def __init__(self) -> None:
        try:
            # Initialize node
            super().__init__('segment_anything')
            
            # Topic Parameters
            self.rgb_topic = load_param(self, 'rgb_topic', 'image').string_value
            self.pointcloud_topic = load_param(self, 'pointcloud_topic', 'pointcloud').string_value
            
            self.segmented_mask_topic = load_param(self, 'segmented_mask_topic', 'segmented_mask').string_value
            self.segmented_image_topic = load_param(self, 'segmented_image_topic', 'segmented_image').string_value
            self.segmented_pointcloud_topic = load_param(self, 'segmented_pointcloud_topic', 'segmented_pointcloud').string_value
            
            # Model parameters (declared but not used in stub)
            self.sam_model_name = load_param(self, 'sam_model_name', 'FastSAM-x.pt').string_value
            self.sam_conf = load_param(self, 'sam_conf', 0.4).double_value
            self.sam_iou = load_param(self, 'sam_iou', 0.9).double_value
            
            self.owl_model_name = load_param(self, 'owl_model_name', 'google/owlvit-base-patch32').string_value
            self.owl_threshold = load_param(self, 'owl_threshold', 0.1).double_value
            
            # Prompt parameters
            self.prompt_type = load_param(self, 'prompt_type', 'bbox').string_value
            self.prompt_text = load_param(self, 'prompt_text', 'a person').string_value
            
            # Publishers (stub - not publishing anything yet)
            self.segmented_mask_pub = self.create_publisher(Image, self.segmented_mask_topic, 1)
            self.segmented_image_pub = self.create_publisher(Image, self.segmented_image_topic, 1)
            self.segmented_pointcloud_pub = self.create_publisher(PointCloud2, self.segmented_pointcloud_topic, 1)
            
            # Subscribers (stub - callback does nothing yet)
            self.rgb_sub = self.create_subscription(Image, self.rgb_topic, self.callback, 1)
            
            self.get_logger().info('SegmentAnything stub initialized successfully')
            self.get_logger().warn('This is a STUB implementation - segmentation functionality not yet implemented')
            
        except Exception as e:
            self.get_logger().fatal(f"Initialization failed: {e}")
            raise e
            
    def callback(self, image_msg):
        """
        Stub callback - does nothing yet.
        TODO: Implement actual segmentation pipeline:
        1. Convert image message to CV2/PIL format
        2. Apply FastSAM model with prompts
        3. Use nanoOWL for text-based detection if needed
        4. Process point clouds if available
        5. Publish segmented results
        """
        # For now, just log that we received an image
        self.get_logger().debug(f'Received image: {image_msg.width}x{image_msg.height}')


def main(args=None):
    rclpy.init(args=args)
    node = SegmentAnything()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
