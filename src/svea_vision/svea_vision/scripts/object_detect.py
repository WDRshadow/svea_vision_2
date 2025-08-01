#! /usr/bin/env python3

import rospy
from sensor_msgs.msg import Image, CameraInfo

import cv2
import numpy as np

from ultralytics import YOLO

from svea_vision_msgs.msg import Object, StampedObjectArray


def load_param(name, value=None):
    if value is None:
        assert rospy.has_param(name), f'Missing parameter "{name}"'
    return rospy.get_param(name, value)


def replace_base(old, new):
    split_last = lambda xs: (xs[:-1], xs[-1])
    is_private = new.startswith("~")
    is_global = new.startswith("/")
    assert not (is_private or is_global)
    ns, _ = split_last(old.split("/"))
    ns += new.split("/")
    return "/".join(ns)


def iou(box1, box2):
    u11, v11, u21, v21 = box1
    u12, v12, u22, v22 = box2

    u1_max = max(u11, u12)
    v1_max = max(v11, v12)
    u2_min = min(u21, u22)
    v2_min = min(v21, v22)

    # area of intersection
    w = abs(u1_max - u2_min)
    h = abs(v1_max - v2_min)
    area_i = w * h

    # area of box1
    w = abs(u21 - u11)
    h = abs(v21 - v11)
    area_1 = w * h

    # area of box2
    w = abs(u22 - u12)
    h = abs(v22 - v12)
    area_2 = w * h

    # area of union
    area_u = area_1 + area_2 - area_i

    # intersection-over-union
    return area_i / area_u


class object_detect:
    def __init__(self):
        ## Initialize node

        rospy.init_node("object_detect")

        ## Parameters

        self.ENABLE_BBOX_IMAGE = load_param("~enable_bbox_image", False)

        self.SUB_IMAGE = load_param("~sub_image", "image")
        self.SUB_CAMERA_INFO = replace_base(self.SUB_IMAGE, "camera_info")

        self.IMAGE_WIDTH = load_param("~image_width", 640)
        self.IMAGE_HEIGHT = load_param("~image_height", 480)

        self.PUB_BBOX_IMAGE = load_param("~pub_bbox_image", "bbox_image")
        self.PUB_CAMERA_INFO = replace_base(self.PUB_BBOX_IMAGE, "camera_info")

        self.USE_CUDA = load_param("~use_cuda", False)
        self.MODEL_PATH = load_param("~model_path", "yolov8n.pt")

        # Space separated list, e.g. 'bed cup dog'
        self.ONLY_OBJECTS = load_param("~only_objects", "").split()
        self.SKIP_OBJECTS = load_param("~skip_objects", "").split()

        self.MAX_AGE = load_param("~max_age", 30)

        self.PUB_OBJECTS = load_param("~pub_objects", "objects")

        ## Neural Network

        self.model = YOLO(self.MODEL_PATH)

        if self.USE_CUDA:
            rospy.loginfo("CUDA enabled")
            self.model.to("cuda")
        else:
            rospy.loginfo("CUDA disabled")

        classes = {lbl: cls for cls, lbl in self.model.names.items()}
        self.label_to_class = lambda label: classes[label]

        self.detect_kwargs = dict(persist=True, conf=0.5, verbose=False)

        if self.ONLY_OBJECTS:
            self.detect_kwargs.update(classes=list(map(self.label_to_class, self.ONLY_OBJECTS)))

        ## Publishers

        self.pub_objects = rospy.Publisher(
            self.PUB_OBJECTS, StampedObjectArray, queue_size=10
        )
        rospy.loginfo(self.PUB_OBJECTS)

        if self.ENABLE_BBOX_IMAGE:
            self.pub_bbox_image = rospy.Publisher(
                self.PUB_BBOX_IMAGE, Image, queue_size=1
            )
            rospy.loginfo(self.PUB_BBOX_IMAGE)

        ## Subscribers

        rospy.Subscriber(self.SUB_IMAGE, Image, self.callback, queue_size=1, buff_size=2**24)
        rospy.loginfo(self.SUB_IMAGE)

        ## Relay (sub->pub) camera info

        if self.ENABLE_BBOX_IMAGE:
            pub = rospy.Publisher(self.PUB_CAMERA_INFO, CameraInfo, queue_size=1)
            rospy.Subscriber(self.SUB_CAMERA_INFO, CameraInfo, pub.publish)

    def run(self):
        rospy.spin()

    def callback(self, image):
        ## Detect objects

        frame = np.frombuffer(image.data, dtype=np.uint8).reshape(
            image.height, image.width, -1
        )
        frame = cv2.resize(frame, (self.IMAGE_WIDTH, self.IMAGE_HEIGHT))
        frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2RGB)

        result = self.model.track(frame, **self.detect_kwargs)[0]

        # if enabled, modify frame (add bounding boxes)
        if self.ENABLE_BBOX_IMAGE:
            frame = result.plot()

        if self.ENABLE_BBOX_IMAGE:
            new_image = Image()
            new_image.header = image.header
            new_image.height = frame.shape[0]
            new_image.width = frame.shape[1]
            new_image.encoding = "rgb8"
            new_image.step = frame.size // new_image.height
            new_image.data = frame.tobytes()

            self.pub_bbox_image.publish(new_image)

        if len(result.boxes) == 0 or not result.boxes.is_track:
            return

        result = result.cpu().numpy()
        it = zip(result.boxes.xyxy,
                result.boxes.conf,
                result.boxes.cls,
                result.boxes.id)

        ## Create object messages

        objects = []
        for box, _pred, _cls, _id in it:
            u1, v1, u2, v2 = box

            # get the label name
            label = result.names[_cls]

            if self.SKIP_OBJECTS and label in self.SKIP_OBJECTS:
                continue

            if self.ONLY_OBJECTS and label not in self.ONLY_OBJECTS:
                continue

            # get real pixel coords
            u1, u2 = [round(u) for u in (u1, u2)]
            v1, v2 = [round(v) for v in (v1, v2)]

            obj = Object()
            obj.id = int(_id)
            obj.label = label
            obj.detection_conf = _pred
            obj.tracking_conf = _pred
            obj.image_width = self.IMAGE_WIDTH
            obj.image_height = self.IMAGE_HEIGHT
            obj.roi.x_offset = u1
            obj.roi.y_offset = v1
            obj.roi.width = u2 - u1
            obj.roi.height = v2 - v1
            objects.append(obj)

        # Publish objects
        if objects:
            object_array = StampedObjectArray()
            object_array.header = image.header
            object_array.objects = objects
            self.pub_objects.publish(object_array)


if __name__ == "__main__":
    ##  Start node  ##

    object_detect().run()
