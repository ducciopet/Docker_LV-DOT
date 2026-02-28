# LV-DOT: Light-Weight Visual Dynamic Obstacle Tracker (ROS2 Port)

## Introduction
This repository contains the **ROS2 Humble** port of the LV-DOT dynamic obstacle detection and tracking system. The original ROS1 implementation can be found in the `src_ROS1/` directory.

LV-DOT integrates:
- **YOLO v11** person detection
- **LiDAR-based dynamic obstacle detection**
- **Kalman filter tracking**
- **Real-time visualization in RViz2**

This setup runs in a **Docker container** with NVIDIA GPU support for YOLO inference.

## Prerequisites

- **Docker Engine:** Follow the [Docker Engine installation guide](https://docs.docker.com/engine/install/ubuntu/), then enable Docker usage as a non-root user as described [here](https://docs.docker.com/engine/install/linux-postinstall/).

- **NVIDIA Driver:** Follow the [NVIDIA Display Driver installation guide](https://github.com/oddmario/NVIDIA-Ubuntu-Driver-Guide).

- **NVIDIA Container Toolkit:** Install the NVIDIA Container Toolkit by following [this guide](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html).

## Repository Structure

```
LV-DOT_ROS2/
├── src/                          # ROS2 workspace source
│   └── onboard_detector/        # Main detector package
├── src_ROS1/                    # Original ROS1 reference code
├── bags/                        # ROS2 bag files
│   └── corridor_demo_mcap/     # MCAP format bag
├── DockerFiles/                 # Docker configuration
│   └── Dockerfile
├── compose.yaml                 # Docker Compose configuration
└── entrypoint.sh               # Container startup script
```

## Setup and Build

### 1. Clone the Repository

```bash
git clone <repository-url>
cd LV-DOT_ROS2
```

### 2. Build the Docker Image

```bash
./compose_build.bash
```

Or manually:
```bash
docker compose build
```

### 3. Start the Container

```bash
docker compose up -d
```

### 4. Enter the Container

```bash
docker compose exec ros2_humble_sim bash
```

## Verify GPU Support

Inside the container, verify NVIDIA GPU is accessible:

```bash
nvidia-smi
```

## Building the Workspace

Inside the container:

```bash
cd ~/ros2_ws
colcon build --packages-select onboard_detector
source install/setup.bash
```

## Running the System

### Terminal 1: Launch the Detector

```bash
ros2 launch onboard_detector run_detector.launch.py
```

This launches:
- `detector_node` - Main C++ detector with LiDAR processing
- `yolov11_detector_node` - Python YOLO detector
- `rviz2` - Visualization with detector_working.rviz config

### Terminal 2: Play the Bag File

First, reindex the MCAP bag (one-time operation):
```bash
cd ~/ros2_ws/bags
ros2 bag reindex corridor_demo_mcap -s mcap
```

Then play the bag with clock simulation:
```bash
ros2 bag play corridor_demo_mcap -s mcap --loop --clock
```

**Important:** The `--clock` flag publishes `/clock` topic required for `use_sim_time:True` nodes.

## ROS2 Command Reference

Common ROS2 commands (updated from ROS1):

### Topics
```bash
# List all topics
ros2 topic list

# Echo a topic
ros2 topic echo /onboard_detector/dynamic_bboxes

# Get topic info
ros2 topic info /pointcloud

# Show topic Hz
ros2 topic hz /camera/color/image_raw
```

### Nodes
```bash
# List running nodes
ros2 node list

# Get node info
ros2 node info /detector_node

# View node parameters
ros2 param list /detector_node
```

### Bags
```bash
# Get bag info
ros2 bag info corridor_demo_mcap

# Play bag
ros2 bag play corridor_demo_mcap -s mcap --loop --clock

# Record new bag
ros2 bag record -o my_bag /topic1 /topic2
```

### Launch Files
```bash
# Launch with default parameters
ros2 launch onboard_detector run_detector.launch.py

# Launch with custom RViz config
ros2 launch onboard_detector run_detector.launch.py rviz_config:=detector_lv.rviz
```

## Key Topics

### Published by Detector
- `/onboard_detector/dynamic_bboxes` - Dynamic obstacle bounding boxes (MarkerArray)
- `/onboard_detector/tracked_bboxes` - Tracked obstacles (MarkerArray)
- `/onboard_detector/filtered_bboxes` - Filtered detections (MarkerArray)
- `/onboard_detector/raw_lidar_point_cloud` - Raw LiDAR pointcloud
- `/onboard_detector/dynamic_point_cloud` - Dynamic obstacle points
- `/onboard_detector/detected_color_image` - Annotated camera image

### Published by YOLO
- `/yolo_detector/detected_bounding_boxes` - YOLO detections (Detection2DArray)
- `/yolo_detector/detected_image` - Annotated YOLO image
- `/yolo_detector/yolo_time` - Inference time statistics

### Subscribed Topics
- `/pointcloud` - Input LiDAR pointcloud
- `/mavros/local_position/pose` - Robot pose
- `/camera/color/image_raw` - Input camera image
- `/camera/depth/image_rect_raw` - Depth image

## Configuration

Edit detector parameters in:
```bash
src/onboard_detector/cfg/detector_param.yaml
```

Key parameters:
- `use_sim_time: true` - Required for bag playback
- `detection_rate: 10.0` - Detection frequency (Hz)
- `tracking_dt: 0.1` - Tracking time step

## Troubleshooting

### YOLO not detecting
- Check `/clock` topic is being published: `ros2 topic hz /clock`
- Verify camera images: `ros2 topic echo /camera/color/image_raw`
- Check YOLO timing: `ros2 topic echo /yolo_detector/yolo_time`

### Empty LiDAR pointcloud
- Ensure `--clock` flag is used during bag playback
- Verify `use_sim_time: true` is set in launch file
- Check message_filters synchronization

### RViz not showing markers
- Verify topics match in RViz config (no `_markers` suffix)
- Check Fixed Frame is set correctly (usually `map` or `local_origin`)
- Enable markers in Displays panel

## Differences from ROS1

Key changes in the ROS2 port:

1. **Publishers/Subscribers:** 
   - ROS1: `ros::Publisher`, `nh_.advertise()`
   - ROS2: `rclcpp::Publisher<>::SharedPtr`, `create_publisher<>()`

2. **Launch Files:**
   - ROS1: `.launch` XML format
   - ROS2: `.launch.py` Python format

3. **Bag Files:**
   - ROS1: `.bag` format
   - ROS2: `.db3` or `.mcap` format with metadata.yaml

4. **Image Transport:**
   - Removed `image_transport` to eliminate compressed topic duplicates
   - Using standard `sensor_msgs::msg::Image` publishers

5. **Time:**
   - ROS1: `ros::Time::now()`
   - ROS2: `this->now()` or `this->get_clock()->now()`

## Citation

If you use this work, please cite the original LV-DOT paper:
```bibtex
@article{xu2022lvdot,
  title={LV-DOT: Light-Weight Visual Dynamic Obstacle Tracker},
  author={Xu, Zhefan and others},
  journal={IEEE Robotics and Automation Letters},
  year={2022}
}
```

## License

See LICENSE file for details.

## References

- Original ROS1 code: `src_ROS1/`
- ROS2 Migration Guide: https://docs.ros.org/en/humble/
- Ultralytics YOLO: https://docs.ultralytics.com/
