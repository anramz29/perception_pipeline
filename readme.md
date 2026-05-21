# perception_pipeline

A ROS 2 C++ package for 6-DoF object pose estimation using the T-LESS BOP dataset. Built for benchmarking and developing perception pipelines for robotic manipulation.

---

## Requirements

- Ubuntu 22.04
- ROS 2 Humble
- Python 3.10
- Git

---

## Installation

### 1. Clone the repo

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/anramz29/perception_pipeline.git
cd perception_pipeline
```

### 2. Install ROS 2 dependencies

```bash
sudo apt update
sudo apt install \
  ros-humble-cv-bridge \
  ros-humble-sensor-msgs \
  ros-humble-geometry-msgs \
  python3-pip
```

### 3. Install Python dependencies

```bash
pip3 install \
  "numpy<2" \
  opencv-python \
  scipy \
  huggingface_hub \
  --break-system-packages
```

> **Note:** `numpy<2` is required due to a compatibility issue with ROS 2 Humble's `cv_bridge`.

### 4. Build the package

```bash
cd ~/ros2_ws
colcon build --packages-select perception_pipeline
source install/setup.bash
```

---

## Dataset Setup

### 1. Download and extract T-LESS

```bash
python3 scripts/download_bop.py
```

This will download the following files into `data/tless/` and extract them:

- `tless_base.zip` — base archive
- `tless_models.zip` — 3D object meshes (PLY)
- `tless_test_primesense_bop19.zip` — test images (~825 MB)

### 2. Convert scene to ROS 2 bag

```bash
python3 scripts/bop_to_rosbag.py
```

The bag will be saved to `data/bags/tless_scene1`.

---

## Visualization

### 1. Play the bag

```bash
ros2 bag play data/bags/tless_scene1 --loop
```

### 2. Launch RViz

```bash
rviz2 -d config/tless_viz.rviz
```

Topics available:

| Topic | Type | Description |
|---|---|---|
| `/camera/rgb/image_raw` | `sensor_msgs/Image` | RGB frames |
| `/camera/depth/image_raw` | `sensor_msgs/Image` | Depth frames |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | Camera intrinsics |
| `/gt_pose/obj_000002` | `geometry_msgs/PoseStamped` | Ground truth pose (object 2) |
| `/gt_pose/obj_000025` | `geometry_msgs/PoseStamped` | Ground truth pose (object 25) |
| `/gt_pose/obj_000029` | `geometry_msgs/PoseStamped` | Ground truth pose (object 29) |
| `/gt_pose/obj_000030` | `geometry_msgs/PoseStamped` | Ground truth pose (object 30) |

> Scene 1 contains objects 2, 25, 29, and 30.

---

## Project Structure

```
perception_pipeline/
├── config/
│   └── tless_viz.rviz          # RViz config
├── data/                        # gitignored
│   ├── bags/                   # ROS 2 bag files
│   └── tless/                  # T-LESS dataset
├── include/perception_pipeline/ # C++ headers
├── launch/                      # Launch files
├── scripts/
│   ├── download_bop.py         # Download + extract T-LESS
│   └── bop_to_rosbag.py        # Convert dataset to ROS 2 bag
├── src/                         # C++ source files
├── test/                        # Unit tests
├── CMakeLists.txt
└── package.xml
```

---

## Dataset

This project uses the [T-LESS](https://bop.felk.cvut.cz/datasets/) dataset from the BOP Benchmark.

> Hodaň et al.: T-LESS: An RGB-D Dataset for 6D Pose Estimation of Texture-less Objects, WACV 2017. License: CC BY 4.0.

The dataset is not included in this repo. Run `scripts/download_bop.py` to fetch it from HuggingFace.