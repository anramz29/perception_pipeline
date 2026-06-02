# perception_pipeline

ROS 2 package for object detection using YOLO11 in C++ via ONNX Runtime, targeting robotic manipulation perception with the T-LESS BOP dataset.

## Status

- [x] C++ detector node (`src/detector_node.cpp`) — YOLO11 via ONNX Runtime
- [x] T-LESS BOP dataset download + ROS 2 bag conversion (`scripts/bop_to_rosbag.py`), including `/gt_detections` ground-truth bounding boxes
- [x] Ground-truth detection viz node (`src/bbox_viz_node.cpp`) — validation scaffold for pose estimation testing
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
  ros-humble-vision-msgs \
  ros-humble-message-filters
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

`bop_to_rosbag.py` converts scene `test_primesense/000001` — which contains T-LESS objects **2, 25, 29, and 30** — into a ROS 2 bag. In addition to RGB, depth, camera info, and per-object ground-truth pose topics, it also publishes a `/gt_detections` topic (`vision_msgs/Detection2DArray`).

`/gt_detections` is a **temporary validation scaffold**: it provides perfect 2D bounding boxes so pose estimation can be developed and evaluated before the YOLO detector is fine-tuned on T-LESS. Once the detector is trained, `/gt_detections` will be replaced by live YOLO output.

Each detection is built by pairing entries from two parallel JSON files in the scene directory:

| File | Content used |
|---|---|
| `scene_gt_info.json` | `bbox_visib` — the visible bounding box `[x, y, w, h]` |
| `scene_gt.json` | `obj_id` — the T-LESS object identity |

Entry *N* in each file refers to the same object instance, so the script zips them together. Boxes are stored as center + half-size (the `vision_msgs` convention), with the T-LESS object ID in `hypothesis.class_id`.

## Export YOLO model to ONNX

```bash
pip3 install ultralytics
python3 -c "from ultralytics import YOLO; YOLO('yolo11n.pt').export(format='onnx')"
# outputs yolo11n.onnx — copy to models/
```

## Running

### Full detector pipeline

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

### Ground-truth detection visualizer

Plays the bag in a loop and runs the bbox viz node, which overlays ground-truth bounding boxes and class-id labels on the RGB stream:

```bash
ros2 launch perception_pipeline detect_viz.launch.py
```

Override the bag path:

```bash
ros2 launch perception_pipeline detect_viz.launch.py \
  bag_path:=/path/to/bag
```

Monitor the annotated output in RViz or with `ros2 topic echo /gt_detections_debug`.

## Topics

| Topic | Type | Description |
|---|---|---|
| `/camera/rgb/image_raw` | `sensor_msgs/Image` | Input RGB frames |
| `/camera/depth/image_raw` | `sensor_msgs/Image` | Input depth frames |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | Camera intrinsics |
| `/gt_pose/obj_<id>` | `geometry_msgs/PoseStamped` | Ground-truth pose per object (from `scene_gt.json`) |
| `/gt_detections` | `vision_msgs/Detection2DArray` | Ground-truth 2D bounding boxes (from `scene_gt_info.json` + `scene_gt.json`) — validation scaffold |
| `/gt_detections_debug` | `sensor_msgs/Image` | RGB image annotated with ground-truth bounding boxes and class-id labels |
| `/detections` | `vision_msgs/Detection2DArray` | YOLO bounding boxes (live detector) |
| `/detections/debug_image` | `sensor_msgs/Image` | Annotated debug image from YOLO detector |

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
│   ├── detect_cpp.launch.py     # YOLO detector + bag playback
│   └── detect_viz.launch.py     # ground-truth bbox viz + bag playback
├── models/
│   └── yolo11n.onnx             # gitignored
├── scripts/
│   ├── download_bop.py
│   └── bop_to_rosbag.py
└── src/
    ├── detector_node.cpp        # YOLO11 via ONNX Runtime
    └── bbox_viz_node.cpp        # ground-truth bbox overlay (message_filters ApproximateTime)
```
