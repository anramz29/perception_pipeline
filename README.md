# perception_pipeline

ROS 2 Python package for 6-DoF object pose estimation using the T-LESS BOP dataset. Built for benchmarking perception pipelines for robotic manipulation.

## Requirements

- Ubuntu 22.04, ROS 2 Humble, Python 3.10

## Installation

### 1. Clone

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/anramz29/perception_pipeline.git
```

### 2. ROS 2 dependencies

```bash
sudo apt update && sudo apt install \
  ros-humble-cv-bridge \
  ros-humble-sensor-msgs \
  ros-humble-geometry-msgs \
  ros-humble-vision-msgs \
  ros-humble-rosbag2-py \
  python3-pip
```

### 3. Python dependencies

```bash
pip3 install \
  "numpy<2" \
  "opencv-python<4.9" \
  scipy \
  huggingface_hub \
  ultralytics \
  --break-system-packages
```

> **Compatibility:** `numpy<2` required for ROS 2 Humble's `cv_bridge`; `opencv-python<4.9` required because newer versions depend on NumPy 2.

### 4. Build

```bash
cd ~/ros2_ws
colcon build --packages-select perception_pipeline
source install/setup.bash
```

## Dataset Setup

```bash
# Download and extract T-LESS into data/tless/
python3 scripts/download_bop.py

# Convert scene to ROS 2 bag at data/bags/tless_scene1
python3 scripts/bop_to_rosbag.py
```

## Running

### Launch (bag + detector)

```bash
ros2 launch perception_pipeline detect.launch.py
```

Override defaults:

```bash
ros2 launch perception_pipeline detect.launch.py \
  bag_path:=/path/to/bag \
  model_path:=/path/to/model.pt \
  confidence_threshold:=0.5
```

### Manual

```bash
ros2 bag play ~/ros2_ws/src/perception_pipeline/data/bags/tless_scene1 --loop
ros2 run perception_pipeline detector_node --ros-args -p model_path:=models/yolo11n.pt
rviz2 -d config/tless_viz.rviz
```

### Topics

| Topic | Type | Description |
|---|---|---|
| `/camera/rgb/image_raw` | `sensor_msgs/Image` | RGB frames |
| `/camera/depth/image_raw` | `sensor_msgs/Image` | Depth frames |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | Camera intrinsics |
| `/gt_pose/obj_000002` | `geometry_msgs/PoseStamped` | Ground truth pose (obj 2) |
| `/gt_pose/obj_000025` | `geometry_msgs/PoseStamped` | Ground truth pose (obj 25) |
| `/gt_pose/obj_000029` | `geometry_msgs/PoseStamped` | Ground truth pose (obj 29) |
| `/gt_pose/obj_000030` | `geometry_msgs/PoseStamped` | Ground truth pose (obj 30) |
| `/detections` | `vision_msgs/Detection2DArray` | YOLOv8 detections |
| `/detections/debug_image` | `sensor_msgs/Image` | Annotated debug image |

> Scene 1 contains objects 2, 25, and 30.

## Project Structure

```
perception_pipeline/
├── config/
│   ├── detector_params.yaml
│   └── tless_viz.rviz
├── data/                        # gitignored
│   ├── bags/
│   └── tless/
├── launch/
│   └── detect.launch.py
├── models/
│   └── yolo11n.pt               # gitignored
├── perception_pipeline/
│   └── detector_node.py
└── scripts/
    ├── download_bop.py
    └── bop_to_rosbag.py
```

## Dataset

[T-LESS](https://bop.felk.cvut.cz/datasets/) from the BOP Benchmark.
Hodaň et al.: *T-LESS: An RGB-D Dataset for 6D Pose Estimation of Texture-less Objects*, WACV 2017. License: CC BY 4.0.
Not included — run `scripts/download_bop.py` to fetch from HuggingFace.
