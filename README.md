# perception_pipeline

ROS 2 package for object detection using YOLO11 in C++ via ONNX Runtime, targeting robotic manipulation perception with the T-LESS BOP dataset.

## Status

- [x] C++ detector node (`src/detector_node.cpp`) — YOLO11 via ONNX Runtime
- [x] T-LESS BOP dataset download + ROS 2 bag conversion scripts
- [x] RViz visualization config
- [ ] Pose estimation node (in progress)
- [ ] TensorRT optimization

## Requirements

- Ubuntu 22.04, ROS 2 Humble
- C++17
- ONNX Runtime 1.20.1 (aarch64)

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
  ros-humble-vision-msgs
```

### 3. ONNX Runtime

Download `onnxruntime-linux-aarch64-1.20.1.tgz` from the [ONNX Runtime releases](https://github.com/microsoft/onnxruntime/releases) and install to `/usr/local`:

```bash
tar xzf onnxruntime-linux-aarch64-1.20.1.tgz
sudo cp -r onnxruntime-linux-aarch64-1.20.1/include/* /usr/local/include/
sudo cp -r onnxruntime-linux-aarch64-1.20.1/lib/*     /usr/local/lib/
sudo ldconfig
```

### 4. Build

```bash
cd ~/ros2_ws && colcon build --packages-select perception_pipeline
source install/setup.bash
```

## Dataset Setup

Utility scripts require Python 3 with `huggingface_hub`. Downloads ~860 MB of [T-LESS](https://bop.felk.cvut.cz/datasets/) (CC BY 4.0).

```bash
pip3 install huggingface_hub rosbags
python3 scripts/download_bop.py   # download + extract T-LESS
python3 scripts/bop_to_rosbag.py  # convert to ROS 2 bag
```

## Export YOLO model to ONNX

```bash
pip3 install ultralytics
python3 -c "from ultralytics import YOLO; YOLO('yolo11n.pt').export(format='onnx')"
# outputs yolo11n.onnx — copy to models/
```

## Running

```bash
ros2 launch perception_pipeline detect_cpp.launch.py
```

Override defaults:

```bash
ros2 launch perception_pipeline detect_cpp.launch.py \
  bag_path:=/path/to/bag \
  model_path:=/path/to/yolo11n.onnx \
  confidence_threshold:=0.25 \
  nms_threshold:=0.45
```

## Topics

| Topic | Type | Description |
|---|---|---|
| `/camera/rgb/image_raw` | `sensor_msgs/Image` | Input RGB frames |
| `/detections` | `vision_msgs/Detection2DArray` | YOLO bounding boxes |
| `/detections/debug_image` | `sensor_msgs/Image` | Annotated debug image |

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
│   └── detect_cpp.launch.py
├── models/
│   └── yolo11n.onnx             # gitignored
├── scripts/
│   ├── download_bop.py
│   └── bop_to_rosbag.py
└── src/
    └── detector_node.cpp
```
