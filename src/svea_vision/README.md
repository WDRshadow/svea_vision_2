## 节点和话题表格

| 节点名称 | 脚本文件 | 订阅话题 | 发布话题 | 功能描述 |
|---------|---------|---------|---------|---------|
| **aruco_detect** | `aruco_detect.py` | `~sub_image` (Image)<br/>`camera_info` (CameraInfo) | `~pub_aruco_pose` (Marker)<br/>TF变换 | ArUco标记检测和3D姿态估计 |
| **object_detect** | `object_detect.py` | `/zed/zed_node/rgb/image_rect_color` (Image)<br/>`/zed/zed_node/rgb/camera_info` (CameraInfo) | `objects` (StampedObjectArray)<br/>`bbox_image` (Image, 可选) | YOLO物体检测，输出2D检测结果 |
| **object_pose** | `object_pose.py` | `objects` (StampedObjectArray)<br/>`/zed/zed_node/depth/depth_registered` (Image)<br/>`/zed/zed_node/depth/camera_info` (CameraInfo) | `objectposes` (StampedObjectPoseArray)<br/>`objectmarkers` (Marker) | 将2D检测转换为3D姿态 |
| **detection_splitter** | `detection_splitter.py` | `/objectposes` (StampedObjectPoseArray) | `~persons` (StampedObjectPoseArray)<br/>`~vehicles` (StampedObjectPoseArray)<br/>`~other` (StampedObjectPoseArray) | 按物体类型分类检测结果 |
| **person_state_estimation** | `person_state_estimation.py` | `/detection_splitter/persons` (StampedObjectPoseArray) | `~person_states_kf` (PersonStateArray) | 卡尔曼滤波行人状态估计 |
| **pedestrian_flow_estimate** | `pedestrian_flow_estimate.py` | `/detection_splitter/persons` (StampedObjectPoseArray) | `~float_1` (Float64)<br/>`~float_2` (Float64)<br/>`~pedestrian_flow_estimate` (PersonStateArray) | 行人流量和速度估计 |
| **sidewalk_segmentation** | `segment_anything.py` | `/zed/zed_node/rgb/image_rect_color` (Image)<br/>`/zed/zed_node/point_cloud/cloud_registered` (PointCloud2, 可选) | `sidewalk_mask` (Image)<br/>`sidewalk_image` (Image)<br/>`sidewalk_pointcloud` (PointCloud2, 可选) | FastSAM人行道分割 |
| **sidewalk_mapper** | `sidewalk_mapper.py` | `/zed/zed_node/point_cloud/cloud_registered` (PointCloud2)<br/>`sidewalk_mask` (Image)<br/>`/zed/zed_node/pose` (PoseStamped) | `sidewalk_occupancy_grid` (OccupancyGrid) | 人行道占用栅格映射 |
| **static_image_publisher** | `static_image_publisher.py` | 无 | `static_image` (Image) | 发布静态图像用于测试 |

## 数据流关系

系统的数据流遵循以下模式：<cite/>

1. **ZED相机** → `object_detect` → `object_pose` → `detection_splitter` → 各类型特定处理节点
2. **ZED相机** → `sidewalk_segmentation` → `sidewalk_mapper` → 占用栅格
3. **行人跟踪链**：`detection_splitter/persons` → `person_state_estimation` 和 `pedestrian_flow_estimate`

## Notes

- 大部分节点使用 `TimeSynchronizer` 来同步多个输入话题
- 话题名称可通过launch文件参数进行重映射
- `~` 前缀表示私有话题，实际话题名会包含节点名称空间