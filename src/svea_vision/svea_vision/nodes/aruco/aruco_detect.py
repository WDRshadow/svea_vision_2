#! /usr/bin/env python3

import numpy as np
import cv2
from cv2 import aruco

import rclpy
from rclpy.node import Node
import tf2_ros
import tf_transformations
from message_filters import TimeSynchronizer, Subscriber
from sensor_msgs.msg import Image, CameraInfo
from aruco_msgs.msg import Marker
from geometry_msgs.msg import TransformStamped, Point, Quaternion
from cv_bridge import CvBridge


def replace_base(old, new):
    split_last = lambda xs: (xs[:-1], xs[-1])
    is_private = new.startswith("~")
    is_global = new.startswith("/")
    assert not (is_private or is_global)
    ns, _ = split_last(old.split("/"))
    ns += new.split("/")
    return "/".join(ns)


class ArucoDetect(Node):
    def __init__(self):
        super().__init__("aruco_detect")

        ## CV Bridge
        self.bridge = CvBridge()

        ## Parameters
        self.declare_parameter("sub_image", "")
        self.declare_parameter("aruco_dict", "DICT_4X4_250")
        self.declare_parameter("aruco_size", "0.05")
        self.declare_parameter("aruco_tf_name", "aruco")
        self.declare_parameter("pub_aruco_pose", "aruco_pose")

        self.SUB_IMAGE = self.get_parameter("sub_image").get_parameter_value().string_value
        self.SUB_CAMERA_INFO = replace_base(self.SUB_IMAGE, "camera_info")

        self.ARUCO_DICT_NAME = self.get_parameter("aruco_dict").get_parameter_value().string_value
        self.ARUCO_SIZE = self.get_parameter("aruco_size").get_parameter_value().string_value
        self.ARUCO_TF_NAME = self.get_parameter("aruco_tf_name").get_parameter_value().string_value

        self.PUB_ARUCO_POSE = self.get_parameter("pub_aruco_pose").get_parameter_value().string_value

        ## Aruco

        self.aruco_size = float(self.ARUCO_SIZE)

        dict_name = getattr(aruco, self.ARUCO_DICT_NAME)
        self.aruco_dict = aruco.Dictionary_get(dict_name)

        ## TF2

        self.br = tf2_ros.TransformBroadcaster(self)

        ## Publishers

        self.pub_aruco_pose = self.create_publisher(Marker, self.PUB_ARUCO_POSE, 5)
        self.get_logger().info(f"Publishing to: {self.PUB_ARUCO_POSE}")

        ## Subscribers

        self.ts = TimeSynchronizer(
            [
                Subscriber(self, Image, self.SUB_IMAGE),
                Subscriber(self, CameraInfo, self.SUB_CAMERA_INFO),
            ],
            queue_size=1,
        )
        self.ts.registerCallback(self.callback)
        self.get_logger().info(f"Subscribing to: {self.SUB_IMAGE}")

    def run(self):
        rclpy.spin(self)

    def callback(self, image, camera_info):
        # convert to grayscale
        gray = self.bridge.imgmsg_to_cv2(image, "mono8")

        corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict)

        if ids is None:
            return

        rvecs, tvecs = aruco.estimatePoseSingleMarkers(
            corners,
            self.aruco_size,
            np.array(camera_info.K).reshape((3, 3)),  # camera matrix
            np.array(camera_info.D).reshape((1, 5)),  # camera distortion
        )[
            :2
        ]  # [:2] due to python2/python3 compatibility

        for aruco_id, rvec, tvec in zip(ids, rvecs, tvecs):
            mtx = np.zeros((4, 4))
            mtx[:3, :3] = cv2.Rodrigues(rvec)[0]
            mtx[:3, 3] = tvec
            mtx[3, 3] = 1
            translation = tf_transformations.translation_from_matrix(mtx)
            rotation = tf_transformations.quaternion_from_matrix(mtx)

            ## Broadcast

            t = TransformStamped()
            t.header = image.header
            t.child_frame_id = self.ARUCO_TF_NAME + str(aruco_id)
            t.transform.translation = Point(*translation)
            t.transform.rotation = Quaternion(x=rotation[0], y=rotation[1], z=rotation[2], w=rotation[3])

            self.br.sendTransform(t)

            ## Publish

            marker = Marker()
            marker.header = image.header
            marker.id = int(aruco_id)
            marker.pose.pose.position = Point(*translation)
            marker.pose.pose.orientation = Quaternion(x=rotation[0], y=rotation[1], z=rotation[2], w=rotation[3])
            marker.confidence = 1  # NOTE: Set this to something more relevant?

            self.pub_aruco_pose.publish(marker)


def main():
    ##  Start node  ##

    rclpy.init()
    node = ArucoDetect()
    
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
