#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from svea_vision_msgs.msg import StampedObjectPoseArray


class DataSplitting(Node):
    """ This class will split the posed object array into an array 
        with person objects and an array with other objects. These 
        arrays can be accessed using 'DataSplitting.GetPersons()' 
        and 'DataSplitting.GetOther()'"""

    def __init__(self):
        super().__init__('detection_splitter')
        self.person_pub = self.create_publisher(
            StampedObjectPoseArray, "~/persons", 10)
        self.vehicle_pub = self.create_publisher(
            StampedObjectPoseArray, "~/vehicles", 10)
        self.other_pub = self.create_publisher(
            StampedObjectPoseArray, "~/other", 10)
        self.persons = []
        self.vehicles = []
        self.other = []
        self.__listener()

    def __split(self, msg):
        """splitting the data depending on the label"""
        persons, persons_msg = [], StampedObjectPoseArray()
        vehicles, vehicle_msg = [], StampedObjectPoseArray()
        other, other_msg = [], StampedObjectPoseArray()
        
        for object in msg.objects:
            if object.object.label == "person":
                persons.append(object)
            if object.object.label == "car":
                vehicles.append(object)
            else:
                other.append(object)

        self.persons = persons
        self.other = other
        self.vehicles = vehicles

        persons_msg.header, persons_msg.objects = msg.header, persons
        vehicle_msg.header, vehicle_msg.objects = msg.header, vehicles
        other_msg.header, other_msg.objects = msg.header, other

        if persons:
            self.person_pub.publish(persons_msg)
        if vehicles:
            self.vehicle_pub.publish(vehicle_msg)
        if other:
            self.other_pub.publish(other_msg)

    # This initialises a node that listens to the topic where the object poses are published to
    def __listener(self):
        self.create_subscription(
            StampedObjectPoseArray,
            "/objectposes",
            self.__split,
            10)

        # spin() simply keeps python from exiting until this node is stopped


def main(args=None):
    rclpy.init(args=args)
    splitted_data = DataSplitting()
    try:
        rclpy.spin(splitted_data)
    except KeyboardInterrupt:
        pass
    finally:
        splitted_data.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
