#!/usr/bin/env python3

__author__ = "Sulthan Suresh Fazeela"
__email__ = "sultha@kth.se"
__license__ = "MIT"

import rclpy
from rclpy.node import Node
import tf2_ros
import tf_transformations as tr
import message_filters as mf
from sensor_msgs.msg import Image, PointCloud2
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Transform
from cv_bridge import CvBridge
import sensor_msgs_py.point_cloud2 as pc2

import time
import numpy as np
import numba as nb

def load_param(node, name, default_value=None):
    node.declare_parameter(name, default_value)
    return node.get_parameter(name).get_parameter_value()

def transform_pointcloud(pointcloud, transform):
    rotation_matrix = tr.quaternion_matrix([transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w])[:3, :3]
    translation = np.array([transform.translation.x, transform.translation.y, transform.translation.z])
    pointcloud = np.dot(rotation_matrix, pointcloud.T).T + translation
    return pointcloud


class SidewalkMapper(Node):
    """
    SidewalkMapper class is a ROS node that subscribes to a pointcloud topic, the corresponding sidewalk mask topic, and the filtered pose topic to create an occupancy grid of the sidewalk.
    Important NOTE: The filtered pose topic should be the pose of the frame in which the pointcloud is published.
    
    Parameters:
        - ~pointcloud_topic (str): The topic name of the pointcloud data. Default: 'pointcloud'
        - ~sidewalk_mask_topic (str): The topic name of the sidewalk mask data. Default: 'sidewalk_mask'
        - ~sidewalk_occupancy_grid_topic (str): The topic name of the sidewalk occupancy grid. Default: 'sidewalk_occupancy_grid'
        - ~filtered_pose_topic (str): The topic name of the filtered pose data. Default: 'filtered_pose'
        - ~sidewalk_z_min (float): The minimum z-value of the sidewalk. Default: -0.5
        - ~sidewalk_z_max (float): The maximum z-value of the sidewalk. Default: 0.5
        - ~non_sidewalk_z_min (float): The minimum z-value of the non-sidewalk. Default: -1.0
        - ~non_sidewalk_z_max (float): The maximum z-value of the non-sidewalk. Default: 1.0
        - ~world_frame (str): The frame id of the world. Default: 'map'
        - ~base_frame (str): The frame id of the base frame. Default: 'base_link'
        - ~resolution (float): The resolution of the occupancy grid. Default: 0.05
        - ~width (int): The width of the occupancy grid. Default: 50
        - ~height (int): The height of the occupancy grid. Default: 50
        - ~occupied_value (int): The value to be assigned for occupied cells in the occupancy grid. Default: 100
        - ~free_value (int): The value to be assigned for free cells in the occupancy grid. Default: 0
        - ~unknown_value (int): The value to be assigned for unknown cells in the occupancy grid. Default: -1
        - ~grid_origin (str): The origin of the grid. Default: 'center'. Options: 'center' or 'bottom'
        - ~pointcloud_max_distance (float): The maximum distance of the pointcloud data to be considered. Default: 7.5
        - ~verbose (bool): Verbose mode. Default: False
        
    Subscribed Topics:
        - pointcloud (sensor_msgs/PointCloud2): The pointcloud data.
        - sidewalk_mask (sensor_msgs/Image): The sidewalk mask data.
        - filtered_pose (geometry_msgs/PoseStamped): The filtered pose data.
        
    Published Topics:
        - sidewalk_occupancy_grid (nav_msgs/OccupancyGrid): The sidewalk occupancy grid.
    """
    
    def __init__(self):
        try:
            # Initialize node
            super().__init__('sidewalk_mapper')
            
            # Topic parameters
            self.pointcloud_topic = load_param(self, 'pointcloud_topic', 'pointcloud').string_value
            self.sidewalk_mask_topic = load_param(self, 'sidewalk_mask_topic', 'sidewalk_mask').string_value
            self.sidewalk_occupancy_grid_topic = load_param(self, 'sidewalk_occupancy_grid_topic', 'sidewalk_occupancy_grid').string_value
            self.filtered_pose_topic = load_param(self, 'filtered_pose_topic', 'filtered_pose').string_value
            
            # Sidewalk parameters
            self.sidewalk_z_min = load_param(self, 'sidewalk_z_min', -0.5).double_value
            self.sidewalk_z_max = load_param(self, 'sidewalk_z_max', 0.5).double_value
            self.non_sidewalk_z_min = load_param(self, 'non_sidewalk_z_min', -1.0).double_value
            self.non_sidewalk_z_max = load_param(self, 'non_sidewalk_z_max', 1.0).double_value
            
            # Occupancy grid parameters
            self.world_frame = load_param(self, 'world_frame', 'map').string_value
            self.base_frame = load_param(self, 'base_frame', 'base_link').string_value
            self.resolution = load_param(self, 'resolution', 0.05).double_value
            self.width = load_param(self, 'width', 50).double_value      # Width is along x-axis in ROS OccupancyGrid
            self.height = load_param(self, 'height', 50).double_value     # Height is along y-axis in ROS OccupancyGrid
            self.occupied_value = load_param(self, 'occupied_value', 100).integer_value
            self.free_value = load_param(self, 'free_value', 0).integer_value
            self.unknown_value = load_param(self, 'unknown_value', -1).integer_value
            self.gird_origin = load_param(self, 'grid_origin', "center").string_value    # "center" or "bottom"
            
            # Other parameters
            self.pointcloud_max_distance = load_param(self, 'pointcloud_max_distance', 7.5).double_value
            self.verbose = load_param(self, 'verbose', False).bool_value
            
            # Check parameters sanity
            if not self.world_frame:
                raise Exception('world_frame parameter not set. Exiting...')
            if not self.base_frame:
                raise Exception('base_frame parameter not set. Exiting...')
            
            # TF2
            self.tf_buf = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buf, self)
            # Sleep for 1 sec for tf2 to populate the buffer
            import time
            time.sleep(1.0)

            # Initialize CvBridge
            self.cv_bridge = CvBridge()
            
            # Initialize occupancy grid message
            self.sidewalk_occupancy_grid = OccupancyGrid()
            self.sidewalk_occupancy_grid.header.frame_id = self.world_frame
            self.sidewalk_occupancy_grid.info.resolution = self.resolution
            self.sidewalk_occupancy_grid.info.width = int(self.width/self.resolution)
            self.sidewalk_occupancy_grid.info.height = int(self.height/self.resolution)
            if self.gird_origin == "bottom":
                # Set world point (0, 0) to be the bottom-center of the grid
                self.sidewalk_occupancy_grid.info.origin.position.x = 0
                self.sidewalk_occupancy_grid.info.origin.position.y = -self.height/2
            else:
                if self.gird_origin != "center":
                    self.get_logger().warn("Invalid grid_origin parameter, defaulting to center")
                # Set world point (0, 0) to be the center of the grid
                self.sidewalk_occupancy_grid.info.origin.position.x = -self.width/2
                self.sidewalk_occupancy_grid.info.origin.position.y = -self.height/2                
            
            # Initialize variables
            self.grid_data = np.full((self.sidewalk_occupancy_grid.info.width, self.sidewalk_occupancy_grid.info.height, 2), (self.unknown_value, 0), dtype=float)  # (x,y) => (probability, no. of observations)
            
            # Publishers
            self.sidewalk_occupancy_grid_pub = self.create_publisher(OccupancyGrid, self.sidewalk_occupancy_grid_topic, 1)
            
            # Subscribers
            self.ts = mf.TimeSynchronizer([
                mf.Subscriber(self.pointcloud_topic, PointCloud2, queue_size=100),
                mf.Subscriber(self.sidewalk_mask_topic, Image, queue_size=100),
                mf.Subscriber(self.filtered_pose_topic, PoseStamped, queue_size=100)
            ], 100)
            self.ts.registerCallback(self.callback)
            
        except Exception as e:
            # Log error
            self.get_logger().fatal(f"Initialization failed: {e}")
            raise e

        else:
            # Log status
            self.get_logger().info("Initialized successfully")
            
    def run(self):
        pass  # ROS2 spinning handled by main function
            
    def callback(self, pointcloud_msg, sidewalk_mask_msg, filtered_pose_msg):
        callback_start = time.time()
        
        # Convert PoseStamped message to TransformStamped message
        transform = Transform()
        transform.translation = filtered_pose_msg.pose.position
        transform.rotation = filtered_pose_msg.pose.orientation
        
        # ===== Convert ROS2 PointCloud2 → numpy Nx3 =====
        points = list(pc2.read_points(pointcloud_msg, field_names=['x', 'y', 'z'], skip_nans=False))
        pointcloud_data = np.array(points, dtype=np.float32).reshape(-1, 3)

        # ===== Convert ROS2 Image → numpy（bool mask） =====
        sidewalk_mask = self.cv_bridge.imgmsg_to_cv2(sidewalk_mask_msg, desired_encoding='passthrough')
        sidewalk_mask = sidewalk_mask.astype(bool).reshape(-1)
        
        convert_time = time.time()
        
        # Update occupancy grid
        self.update_grid(pointcloud_data, sidewalk_mask, transform)
        
        update_grid_time = time.time()
        
        # Create occupancy grid
        self.sidewalk_occupancy_grid.header.stamp = sidewalk_mask_msg.header.stamp
        self.sidewalk_occupancy_grid.data = self.create_occupancy_grid()
        
        # Publish occupancy grid
        self.sidewalk_occupancy_grid_pub.publish(self.sidewalk_occupancy_grid)
        
        publish_time = time.time()
        
        # Log
        if self.verbose:
            self.get_logger().info("Callback time: {:.3f} s, Convert time: {:.3f} s, Update grid time: {:.3f} s, Publish time: {:.3f} s".format(publish_time - callback_start, convert_time - callback_start, update_grid_time - convert_time, publish_time - update_grid_time))

    def update_grid(self, pointcloud_data, sidewalk_mask, transform):
        # Separate sidewalk and non-sidewalk points
        sidewalk_pointcloud_data = pointcloud_data[sidewalk_mask]
        non_sidewalk_pointcloud_data = pointcloud_data[~sidewalk_mask]
        
        # Remove NaN values
        sidewalk_pointcloud_data = sidewalk_pointcloud_data[~np.isnan(sidewalk_pointcloud_data).any(axis=1)]
        non_sidewalk_pointcloud_data = non_sidewalk_pointcloud_data[~np.isnan(non_sidewalk_pointcloud_data).any(axis=1)]
        
        # Filter pointcloud data based on distance
        sidewalk_pointcloud_data = sidewalk_pointcloud_data[sidewalk_pointcloud_data[:, 0] <= self.pointcloud_max_distance]
        non_sidewalk_pointcloud_data = non_sidewalk_pointcloud_data[non_sidewalk_pointcloud_data[:, 0] <= self.pointcloud_max_distance]
        
        # Transform pointclouds
        sidewalk_pointcloud_data = transform_pointcloud(sidewalk_pointcloud_data, transform)
        non_sidewalk_pointcloud_data = transform_pointcloud(non_sidewalk_pointcloud_data, transform)
        
        # Fill sidewalk points in occupancy grid
        self.update_sidewalk_points(sidewalk_pointcloud_data)
        
        # Fill non-sidewalk points in occupancy grid
        self.update_non_sidewalk_points(non_sidewalk_pointcloud_data)        
        
    def update_sidewalk_points(self, sidewalk_pointcloud_data):
        grid_info = np.array([self.sidewalk_occupancy_grid.info.origin.position.x, self.sidewalk_occupancy_grid.info.origin.position.y, self.sidewalk_occupancy_grid.info.width, self.sidewalk_occupancy_grid.info.height, self.sidewalk_occupancy_grid.info.resolution])
        SidewalkMapper._update_sidewalk_points(sidewalk_pointcloud_data, self.grid_data, grid_info, self.sidewalk_z_min, self.sidewalk_z_max, self.free_value)
        
    def update_non_sidewalk_points(self, non_sidewalk_pointcloud_data):
        grid_info = np.array([self.sidewalk_occupancy_grid.info.origin.position.x, self.sidewalk_occupancy_grid.info.origin.position.y, self.sidewalk_occupancy_grid.info.width, self.sidewalk_occupancy_grid.info.height, self.sidewalk_occupancy_grid.info.resolution])
        SidewalkMapper._update_non_sidewalk_points(non_sidewalk_pointcloud_data, self.grid_data, grid_info, self.non_sidewalk_z_min, self.non_sidewalk_z_max, self.occupied_value)
    
    def create_occupancy_grid(self):
        # Flatten column-major order (Fortran-style) to match ROS OccupancyGrid
        # Refer to (https://robotics.stackexchange.com/a/66500) for a detailed explanation
        return self.grid_data[:, :, 0].astype(int).flatten(order='F').tolist()

    @staticmethod
    @nb.jit(nopython=True)
    def _update_sidewalk_points(sidewalk_pointcloud_data, grid_data, grid_info, sidewalk_z_min, sidewalk_z_max, free_value):
        # Extract grid origin and dimensions
        x_origin = grid_info[0]
        y_origin = grid_info[1]
        width = grid_info[2]
        height = grid_info[3]
        resolution = grid_info[4]
        
        for point in sidewalk_pointcloud_data:
            x, y, z = point
            # Convert world point to grid cell
            i = int((x - x_origin) / resolution)
            j = int((y - y_origin) / resolution)
            
            # Check if grid cell is within bounds
            if 0 <= i < width and 0 <= j < height:
                old_prob, n = grid_data[i, j]
                if sidewalk_z_min <= z < sidewalk_z_max:
                    new_prob = (old_prob * n + free_value) / (n + 1)
                    grid_data[i, j] = (new_prob, n + 1)
    
    @staticmethod
    @nb.jit(nopython=True)
    def _update_non_sidewalk_points(non_sidewalk_pointcloud_data, grid_data, grid_info, non_sidewalk_z_min, non_sidewalk_z_max, occupied_value):
        # Extract grid origin and dimensions
        x_origin = grid_info[0]
        y_origin = grid_info[1]
        width = grid_info[2]
        height = grid_info[3]
        resolution = grid_info[4]
        
        for point in non_sidewalk_pointcloud_data:
            x, y, z = point
            # Convert world point to grid cell
            i = int((x - x_origin) / resolution)
            j = int((y - y_origin) / resolution)
            
            # Check if grid cell is within bounds
            if 0 <= i < width and 0 <= j < height:
                old_prob, n = grid_data[i, j]
                if non_sidewalk_z_min <= z < non_sidewalk_z_max:
                    new_prob = (old_prob * n + occupied_value) / (n + 1)
                    grid_data[i, j] = (new_prob, n + 1)
                
    
def main(args=None):
    rclpy.init(args=args)
    node = SidewalkMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()