# perception_pipeline

ROS 2 package for 6-DoF object pose estimation using the T-LESS BOP dataset, targeting robotic manipulation perception.

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

> `numpy<2` and `opencv-python<4.9` required for ROS 2 Humble cv_bridge compatibility.

### 4. Build

```bash
cd ~/ros2_ws && colcon build --packages-select perception_pipeline
source install/setup.bash
```

## Dataset Setup

Downloads ~860 MB of [T-LESS](https://bop.felk.cvut.cz/datasets/) from HuggingFace (CC BY 4.0).

```bash
python3 scripts/download_bop.py   # download + extract T-LESS
python3 scripts/bop_to_rosbag.py  # convert to ROS 2 bag
```

## Running

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

## Topics

| Topic | Type | Description |
|---|---|---|
| `/camera/rgb/image_raw` | `sensor_msgs/Image` | RGB frames |
| `/camera/depth/image_raw` | `sensor_msgs/Image` | Depth frames |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | Camera intrinsics |
| `/detections` | `vision_msgs/Detection2DArray` | YOLO detections |
| `/detections/debug_image` | `sensor_msgs/Image` | Annotated debug image |
| `/gt_pose/obj_XXXXXX` | `geometry_msgs/PoseStamped` | Ground truth poses |


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