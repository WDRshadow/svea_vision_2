# svea_vision
This package contains the vision system for SVEA. It contains nodes for object detection, pose estimation, aruco markers and sidewalk segmentation. 

## Requirements
##### Python dependencies
- scipy
- filterpy
- shapely

##### Packages for the ZED object detection:
- aliencontrol
- ros_abconnect
- rsu_core
- vision_opencv
- zed-ros-wrapper
- realsense-ros
- image_common
- image_transport_plugins

## Usage and Installation

Please refer to [svea](https://github.com/KTH-SML/svea) on how to install and run SVEA software.

### 1. Basic use of Jetson and Zed Camera
- Plug in Nvidia Jetson and Zed camera accordingly and boot up the Jetson.

On your own machine:
- Connect to the router RUT360

- ensure you are on RSU branch (git checkout rsu)
- Open two cmd terminals 
- 1st terminal: ping "ubuntu" (need to fix network if not functioning)
- 1st terminal: ssh nvidia@ubuntu
- 2nd terminal: change directory (cd) into root folder of the workspace (infra_ws).
- 2nd terminal: util/remote_ros.sh ubuntu
- 1st terminal: cd into root of workspace on ubuntu 
- 1st terminal: sudo util/run
- 1st terminal: roslaunch infra_ws "pkg.launch" (roslaunch zed_main.launch or roslaunch camera2.launch)
- 2nd terminal: open rviz using "rviz" (can now visualize published messages in rviz)


### 2. Calibrating the Camera and Aruco Markers (prior to demo)

	Note: This needs to be performed for any new aruco marker that one desires
		to add to the testing environment

- Place/fix the aruco marker in a location that can be well seen by a camera. Ideally within 10 meters of the camera.
- Setup the camera somewhere within the map bounds, within clear view of the desired unconfigured aruco marker.

Determine the camera pose using rviz
- launch the package as done in 1. (roslaunch zed_main.launch) and launch rviz
- Using the map of the environment in rviz determine as well as possible the position (x,y,z) and orientation (roll pitch yaw) of the camera in the map.
- NOTE: The coordinates for the camera that are returned by 'calibrating' are still approximate as the aruco needs to be calibrated first. Use the measuring tool in RViz instead.
- cd into infra_ws/src/infra/launch/zed_main.launch 
- comment out the node "camera2map" and uncomment node "aruco2map"

- Add this line in the launch file: "<node pkg="tf2_ros" type="static_transform_publisher" name="link_map_camera" args="x y z yaw nick roll map zed_base_link"/>" using the pose of the camera
- Relaunch using the changed zed_main launch file
- Read the aruco coordinates returned in the terminal by echoing the topic rsu/arucoposes
- Note that this pose uses quaternions, this is however fine for tf2
- It is recommended to put the static publisher in all launch files to be able to switch cameras

- cd into infra_ws/src/infra/launch/zed_main.launch 
- add the associated aruco (id #) in the following format with the appropriate pose inserted:
	<node pkg="tf2_ros" type="static_transform_publisher" name="link_map_aruco#" args="x y z quatX quatY quatZ quatW map aruco#"/>
- comment out the node "aruco2map" and uncomment node "camera2map"
- Run the launch file as described in 1. and launch zed_main.launch, as well as rviz.
- Enable camera feed in rviz as well as the map topic.
- Check if the calibration is adequate be checking if the map is aligned with the camerafeed. Otherwise, the orientation of the aruco marker can be altered slightly in the launch file.

The camera is now calibrated, and can be dynamically placed wherever your heart desires (as long as the marker is visible).
In practice you shouldn't move the camera more than 10 centimeters as the calibration is quite bad


### 3. Adding additional cameras
- In order to launch the second camera, launch camera2.launch as in step 1.
- Note that it could be beneficial to have the second camera run on a seperate computer, since the object detection is computationally heavy. In this case, make sure that you are connected to the alternative computer instead.

## ROS Nodes

### aruco_detect.py
TODO

### detection_splitter.py
TODO

### object_detect.py
TODO

### object_pose.py
TODO

### person_state_estimation.py
TODO

### pedestrian_flow_estimate.py
This ROS node, `pedestrian_flow_estimate.py`, subscribes to topics that provide detected pedestrian data, and publishes estimated speed and acceleration of these pedestrians.

### segment_anything.py
The SegmentAnything ROS node performs object segmentation in images based on a given prompt, which can be a bounding box, points, or text. It uses the `FastSAM` model for segmentation, with optional use of the `NanoOwl` model to generate bounding boxes from text prompts. The node publishes the segmented mask, image, and point cloud data to specified ROS topics.

#### Key Parameters
- **Input Topics**
    - `~rgb_topic` (string, default: "image"): Input topic for RGB images.
    - `~pointcloud_topic` (string, default: "pointcloud"): Topic for point cloud data if 3D segmentation is required.

- **Model Configuration**
    - `~sam_model_name` (string, default: "FastSAM-x.pt"): Selects the FastSAM model to use, either FastSAM-x.pt for higher accuracy or FastSAM-s.pt for faster performance.
    - `~sam_conf` (float, default: 0.4): Confidence threshold for the segmentation model.
    - `~sam_iou` (float, default: 0.9): IoU threshold for the segmentation model.

- **Prompt Selection**
    - `~prompt_type` (string, default: "bbox"): Type of prompt to guide the segmentation, with options being bbox, points, or text.
    - `~prompt_bbox` (list of floats, default: [0.30, 0.50, 0.70, 0.90]): Bounding box prompt in the format [xmin, ymin, xmax, ymax] normalized to the image size.
    - `~prompt_text` (string, default: "a person"): Text prompt to guide the segmentation. If using a text prompt, the NanoOwl model generates bounding boxes, which are then passed to FastSAM for segmentation.

- **Publishing Options**
    - `~publish_mask` (bool, default: False): Publish the segmented mask.
    - `~publish_image` (bool, default: True): Publish the segmented image.
    - `~publish_pointcloud` (bool, default: False): Publish the segmented point cloud.

#### Performance Considerations
The bounding box prompt is the fastest, followed by the points prompt, and finally the text prompt. The average publishing time per frame using `FastSAM-x` model for the bounding box prompt is ~0.3 seconds, for the points prompt it is ~0.35 seconds, and for the text prompt using `NanoOwl` model is ~0.4 seconds when running on Zed Box with CUDA enabled and maximum power mode. `FastSAM-s` model is faster than `FastSAM-x` model, but it is less accurate.

### sidewalk_mapper.py
SidewalkMapper class is a ROS node that subscribes to a pointcloud topic, the corresponding sidewalk mask topic, and the filtered pose topic to create an occupancy grid of the sidewalk.  
_Important note_: The filtered pose topic should be the pose of the frame in which the pointcloud is published.

#### Key Parameters
- **Input Topics**
    - `~pointcloud_topic` (string, default: "pointcloud"): The topic name of the pointcloud data.
    - `~sidewalk_mask_topic` (string, default: "sidewalk_mask"): The topic name of the sidewalk mask data.
    - `~filtered_pose_topic` (string, default: "filtered_pose"): The topic name of the filtered pose data.

- **Sidewalk Configuration**
    - `~sidewalk_z_min` (float, default: -0.5): The minimum z-value of the sidewalk.
    - `~sidewalk_z_max` (float, default: 0.5): The maximum z-value of the sidewalk.
    - `~non_sidewalk_z_min` (float, default: -1.0): The minimum z-value of the non-sidewalk.
    - `~non_sidewalk_z_max` (float, default: 1.0): The maximum z-value of the non-sidewalk.

- **Occupancy Grid Configuration**
    - `~resolution` (float, default: 0.05): The resolution of the occupancy grid.
    - `~width` (int, default: 50): The width of the occupancy grid.
    - `~height` (int, default: 50): The height of the occupancy grid.
    - `~grid_origin` (string, default: "center"): The origin of the grid, either "center" or "bottom".

### static_image_publisher.py
This node publishes a static image or a set of static images from a directory to a topic at a fixed rate. The script is useful for testing other nodes that subscribe to image topics.

#### Key Parameters
- **Image Path**
    - `~image_path` (string): The path to a single image or a directory of images. If a directory is specified, the images are published in random order.
