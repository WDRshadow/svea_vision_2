# svea_vision
This package contains the vision system for SVEA. It contains nodes for object detection, pose estimation, aruco markers and sidewalk segmentation. 

### Dependences

- ROS2
- ZED SDK
- CUDA
- PyTorch (GPU)
- Torchvision
- RVIZ2
- Others following [src/svea_vision/README.md](src/svea_vision/README.md)

##### Packages for the ZED object detection:
- zed-ros-wrapper

    Note: Please follow the document of `https://github.com/stereolabs/zed-ros2-wrapper` for installing the dependence of `ZED sdk`.

### Basic use of Jetson and Zed Camera
- Plug in Nvidia Jetson and Zed camera accordingly and boot up the Jetson.

On your own machine:

```bash
colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release --parallel-workers $(nproc) # build the workspace
source install/setup.bash
```

Then you can use RVIZ2 (on another terminal) to visualize the ROS2 message"

```bash
rviz2
```