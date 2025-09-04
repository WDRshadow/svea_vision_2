## Nodes and Topics Table

| Node Name | Script File | Subscribed Topics | Published Topics | Function Description |
|-----------|-------------|-------------------|------------------|----------------------|
| **aruco_detect** | `aruco_detect.py` | `~sub_image` (Image)<br/>`camera_info` (CameraInfo) | `~pub_aruco_pose` (Marker)<br/>TF Transform | ArUco marker detection and 3D pose estimation |
| **object_detect** | `object_detect.py` | `/zed/zed_node/rgb/image_rect_color` (Image)<br/>`/zed/zed_node/rgb/camera_info` (CameraInfo) | `objects` (StampedObjectArray)<br/>`bbox_image` (Image, optional) | YOLO object detection, outputs 2D detection results |
| **object_pose** | `object_pose.py` | `objects` (StampedObjectArray)<br/>`/zed/zed_node/depth/depth_registered` (Image)<br/>`/zed/zed_node/depth/camera_info` (CameraInfo) | `objectposes` (StampedObjectPoseArray)<br/>`objectmarkers` (Marker) | Convert 2D detections to 3D poses |
| **detection_splitter** | `detection_splitter.py` | `/objectposes` (StampedObjectPoseArray) | `~persons` (StampedObjectPoseArray)<br/>`~vehicles` (StampedObjectPoseArray)<br/>`~other` (StampedObjectPoseArray) | Classify detection results by object type |
| **person_state_estimation** | `person_state_estimation.py` | `/detection_splitter/persons` (StampedObjectPoseArray) | `~person_states_kf` (PersonStateArray) | Kalman filter pedestrian state estimation |
| **pedestrian_flow_estimate** | `pedestrian_flow_estimate.py` | `/detection_splitter/persons` (StampedObjectPoseArray) | `~float_1` (Float64)<br/>`~float_2` (Float64)<br/>`~pedestrian_flow_estimate` (PersonStateArray) | Pedestrian flow and velocity estimation |
| **sidewalk_segmentation** | `segment_anything.py` | `/zed/zed_node/rgb/image_rect_color` (Image)<br/>`/zed/zed_node/point_cloud/cloud_registered` (PointCloud2, optional) | `sidewalk_mask` (Image)<br/>`sidewalk_image` (Image)<br/>`sidewalk_pointcloud` (PointCloud2, optional) | FastSAM sidewalk segmentation |
| **sidewalk_mapper** | `sidewalk_mapper.py` | `/zed/zed_node/point_cloud/cloud_registered` (PointCloud2)<br/>`sidewalk_mask` (Image)<br/>`/zed/zed_node/pose` (PoseStamped) | `sidewalk_occupancy_grid` (OccupancyGrid) | Sidewalk occupancy grid mapping |
| **static_image_publisher** | `static_image_publisher.py` | None | `static_image` (Image) | Publish static images for testing |

## Data Flow Relationships

The system's data flow follows these patterns:

1. **ZED Camera** → `object_detect` → `object_pose` → `detection_splitter` → type-specific processing nodes
2. **ZED Camera** → `sidewalk_segmentation` → `sidewalk_mapper` → occupancy grid
3. **Pedestrian tracking chain**: `detection_splitter/persons` → `person_state_estimation` and `pedestrian_flow_estimate`

## Notes

- Most nodes use `TimeSynchronizer` to synchronize multiple input topics
- Topic names can be remapped through launch file parameters
- The `~` prefix indicates private topics, where the actual topic name will include the node namespace