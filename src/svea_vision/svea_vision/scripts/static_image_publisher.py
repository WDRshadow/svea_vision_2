import os
import cv2
import random

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class PublishImage(Node):
    """
    This class is a ROS2 node that publishes an image or a set of images to a topic at a specified rate. The image(s) can be specified as a single image path or a directory of images.
    
    Parameters:
        - image_topic (str): The topic name to publish the image(s) to. Default: 'static_image'
        - image_path (str): The path to a single image or a directory of images.
        - rate (int): The rate at which to publish the image(s). Default: 30 Hz
        
    Subscribed Topics:
        - None
        
    Published Topics:
        - image_topic (sensor_msgs/Image): The image(s) to be published.
    """

    def __init__(self):
        super().__init__('static_image_publisher')
        
        try:
            # Parameters
            self.declare_parameter('image_topic', 'static_image')
            self.declare_parameter('image_path', '')
            self.declare_parameter('rate', 30)
            
            self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
            self.image_path = self.get_parameter('image_path').get_parameter_value().string_value
            self.rate = self.get_parameter('rate').get_parameter_value().integer_value

            # CV Bridge
            self.cv_bridge = CvBridge()

            # Publisher
            self.img_pub = self.create_publisher(Image, self.image_topic, 1)

        except Exception as e:
            # Log error
            self.get_logger().fatal(f"{self.get_name()}: {e}")
            return

        else:
            # Log status
            self.get_logger().info(f'{self.get_name()} node initialized.')

    def run(self):
        # Check if image path is a directory
        if os.path.isdir(self.image_path):
            # Get all jpg, jpeg or png files in directory
            image_paths = [os.path.join(self.image_path, f) for f in os.listdir(self.image_path)
                                if f.endswith('.jpg') or f.endswith('.jpeg') or f.endswith('.png')]
        else:
            # Set single image path
            image_paths = [self.image_path]
            
        # Read all images
        imgs = [cv2.imread(image_path) for image_path in image_paths]
        img_msgs = [self.cv_bridge.cv2_to_imgmsg(img, encoding='bgr8') for img in imgs]

        # Publish and sleep loop
        timer_period = 1.0 / self.rate  # seconds
        self.timer = self.create_timer(timer_period, lambda: self.timer_callback(img_msgs))

    def timer_callback(self, img_msgs):
        # Publish a random image
        self.img_pub.publish(img_msgs[random.randint(0, len(img_msgs)-1)])


def main():
    rclpy.init()
    node = PublishImage()
    
    try:
        node.run()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()