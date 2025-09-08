# SVEA Vision Package

## Package Overview

`svea_vision` is a ROS2-based computer vision package designed specifically for the SVEA (Small Vehicles for Autonomous Navigation) project. This package provides a complete visual perception pipeline including object detection, pose estimation, pedestrian state estimation, sidewalk segmentation, and mapping functionalities.

## Package Information

- **Package Name**: svea_vision
- **License**: MIT
- **Build Type**: ament_python

## Dependencies

### ROS2 Dependencies
- rclpy, std_msgs, sensor_msgs, geometry_msgs
- visualization_msgs, nav_msgs, svea_vision_msgs
- cv_bridge, tf2_ros, tf2_geometry_msgs
- image_geometry, message_filters

### Python Dependencies
- ultralytics==8.2.0 (YOLO models)
- filterpy (Kalman filters)
- numba, pandas, numpy<2.0
- opencv, scipy

## Nodes and Topics Table

| Node Name | Script File | Subscribed Topics | Published Topics | Function Description |
|-----------|-------------|-------------------|------------------|----------------------|
| **aruco_detect** | `aruco_detect.py` | `~sub_image` (Image)<br/>`camera_info` (CameraInfo) | `~pub_aruco_pose` (Marker)<br/>TF Transform | ArUco marker detection and 3D pose estimation |
| **object_detect** | `object_detect.py` | `/zed/zed_node/rgb/image_rect_color` (Image)<br/>`/zed/zed_node/rgb/camera_info` (CameraInfo) | `objects` (StampedObjectArray)<br/>`bbox_image` (Image, optional) | YOLO-based object detection, outputs 2D detection results |
| **object_pose** | `object_pose.py` | `objects` (StampedObjectArray)<br/>`/zed/zed_node/depth/depth_registered` (Image)<br/>`/zed/zed_node/depth/camera_info` (CameraInfo) | `objectposes` (StampedObjectPoseArray)<br/>`objectmarkers` (Marker) | Convert 2D detection results to 3D poses |
| **detection_splitter** | `detection_splitter.py` | `/objectposes` (StampedObjectPoseArray) | `~persons` (StampedObjectPoseArray)<br/>`~vehicles` (StampedObjectPoseArray)<br/>`~other` (StampedObjectPoseArray) | Classify detection results by object type |
| **person_state_estimation** | `person_state_estimation.py` | `/detection_splitter/persons` (StampedObjectPoseArray) | `~person_states_kf` (PersonStateArray) | Kalman filter-based pedestrian state estimation and trajectory prediction |
| **pedestrian_flow_estimate** | `pedestrian_flow_estimate.py` | `/detection_splitter/persons` (StampedObjectPoseArray) | `~float_1` (Float64)<br/>`~float_2` (Float64)<br/>`~pedestrian_flow_estimate` (PersonStateArray) | Pedestrian flow and velocity estimation |
| **segment_anything** | `segment_anything.py` | `/zed/zed_node/rgb/image_rect_color` (Image)<br/>`/zed/zed_node/point_cloud/cloud_registered` (PointCloud2, optional) | `sidewalk_mask` (Image)<br/>`sidewalk_image` (Image)<br/>`sidewalk_pointcloud` (PointCloud2, optional) | FastSAM-based sidewalk segmentation |
| **sidewalk_mapper** | `sidewalk_mapper.py` | `/zed/zed_node/point_cloud/cloud_registered` (PointCloud2)<br/>`sidewalk_mask` (Image)<br/>`/zed/zed_node/pose` (PoseStamped) | `sidewalk_occupancy_grid` (OccupancyGrid) | Sidewalk occupancy grid mapping |
| **static_image_publisher** | `static_image_publisher.py` | None | `static_image` (Image) | Publish static images for testing purposes |

## Package Structure

```
svea_vision/
├── svea_vision/
│   ├── nodes/
│   │   ├── aruco/          # ArUco marker detection
│   │   │   └── aruco_detect.py
│   │   ├── image/          # Image processing utilities
│   │   │   └── static_image_publisher.py
│   │   ├── object/         # Object detection and pose estimation
│   │   │   ├── object_detect.py
│   │   │   ├── object_pose.py
│   │   │   └── detection_splitter.py
│   │   ├── person/         # Pedestrian state analysis
│   │   │   ├── person_state_estimation.py
│   │   │   └── pedestrian_flow_estimate.py
│   │   └── sidewalk/       # Sidewalk segmentation and mapping
│   │       ├── segment_anything.py
│   │       └── sidewalk_mapper.py
│   └── utils/              # Utility functions and classes
│       ├── kalman_filter.py
│       ├── sort.py
│       └── extract_csvs.py
├── launch/                 # Launch files
├── test/                   # Test files
├── package.xml
└── setup.py
```

## Main Functional Modules

### 1. Object Detection Pipeline
- **YOLO Detection**: Uses Ultralytics YOLO v8 for 2D object detection
- **3D Pose Estimation**: Combines depth information to convert 2D detections to 3D poses
- **Object Classification**: Classifies detection results by type (persons, vehicles, others)

### 2. Pedestrian State Estimation
- **Trajectory Tracking**: Uses Kalman filters for pedestrian position and velocity prediction
- **State Estimation**: Estimates pedestrian position, velocity, and orientation
- **Flow Analysis**: Analyzes pedestrian flow and movement patterns

### 3. Sidewalk Segmentation and Mapping
- **Semantic Segmentation**: Uses FastSAM for sidewalk region segmentation
- **Occupancy Grid**: Generates sidewalk occupancy grid maps
- **3D Mapping**: Combines point cloud data for three-dimensional mapping

### 4. ArUco Marker Detection
- **Marker Detection**: Detects ArUco markers and estimates their 3D poses
- **Coordinate Transformation**: Publishes TF transforms for localization

## Launch Files

| Launch File | Function Description |
|----------|----------|
| `zed_main.launch.py` | Main launch file that integrates all vision functionalities |
| `object_detect.launch.py` | Launch only object detection functionality |
| `object_pose.launch.py` | Object detection and pose estimation |
| `sidewalk_segmentation.launch.py` | Sidewalk segmentation functionality |
| `sidewalk_mapper.launch.py` | Sidewalk mapping functionality |
| `aruco_detect.launch.py` | ArUco marker detection |
| `zed_eval.launch.py` | Evaluation and testing configuration |

## Configuration Parameters

### Main Parameters
- `camera_name`: Camera name (default: 'zed')
- `camera_model`: Camera model (default: 'zed')
- `use_cuda`: Whether to use CUDA acceleration (default: true)
- `enable_bbox_image`: Whether to enable bounding box image output
- `enable_aruco`: Whether to enable ArUco detection
- `enable_state_estimation`: Whether to enable state estimation
- `enable_sidewalk_segmentation`: Whether to enable sidewalk segmentation

### Pedestrian Tracking Parameters
- `max_time_missing`: Maximum missing time for pedestrian tracking (default: 1.0s)
- `vel_filter_window`: Velocity filter window size (default: 15)
- `discard_id_threshold`: ID discard threshold (default: 0.5)

## Data Flow Relationships

The system's data flow follows these patterns:

### 1. Main Detection Chain
```
ZED Camera → object_detect → object_pose → detection_splitter → type-specific processing nodes
```

### 2. Sidewalk Mapping Chain
```
ZED Camera → sidewalk_segmentation → sidewalk_mapper → occupancy grid
```

### 3. Pedestrian Tracking Chain
```
detection_splitter/persons → person_state_estimation (Kalman filtering)
                          → pedestrian_flow_estimate (flow analysis)
```

## Usage Instructions

### Basic Launch
```bash
# Launch complete vision system
ros2 launch svea_vision zed_main.launch.py

# Launch only object detection
ros2 launch svea_vision object_detect.launch.py

# Launch sidewalk segmentation
ros2 launch svea_vision sidewalk_segmentation.launch.py
```

### Custom Parameters
```bash
# Enable all functionalities
ros2 launch svea_vision zed_main.launch.py \
    enable_aruco:=true \
    enable_state_estimation:=true \
    enable_sidewalk_segmentation:=true \
    use_cuda:=true
```

## Utility Classes

### Kalman Filter (`utils/kalman_filter.py`)
Kalman filter implementation for pedestrian state estimation.

### SORT (`utils/sort.py`)
Simple Online and Realtime Tracking algorithm implementation.

### CSV Extraction (`utils/extract_csvs.py`)
Utility functions for data extraction and analysis.

## Testing

```bash
# Run tests
colcon test --packages-select svea_vision

# Kalman filter test
python3 test/kalman_filter_test.py
```

## Important Notes

- Most nodes use `TimeSynchronizer` to synchronize multiple input topics
- Topic names can be remapped through launch file parameters
- The `~` prefix indicates private topics, where the actual topic name will include the node namespace
- The system is optimized for ZED cameras but can be adapted for other RGB-D cameras
- YOLO models need to be downloaded separately, typically placed as `yolov8n.pt` in the package root directory

## Development and Contributing

This package is part of the SVEA project, primarily used for visual perception in autonomous navigation vehicles. For code contributions or issue reporting, please contact the maintainer.

## Data Flow Relationships

The system's data flow follows these patterns:

1. **ZED Camera** → `object_detect` → `object_pose` → `detection_splitter` → type-specific processing nodes
2. **ZED Camera** → `segment_anything` → `sidewalk_mapper` → occupancy grid
3. **Pedestrian tracking chain**: `detection_splitter/persons` → `person_state_estimation` and `pedestrian_flow_estimate`

## Important Notes

- Most nodes use `TimeSynchronizer` to synchronize multiple input topics
- Topic names can be remapped through launch file parameters
- The `~` prefix indicates private topics, where the actual topic name will include the node namespace
- This package is designed for the visual perception needs of the SVEA autonomous vehicle project