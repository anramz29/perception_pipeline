# CLAUDE.md

## Project Overview
ROS 2 mixed C++/Python package for 6-DoF object pose estimation using the T-LESS BOP dataset.
Target use case: robotic manipulation perception pipeline for industrial objects.

## Build System
- ROS 2 Humble + ament_cmake_python (mixed C++ and Python)
- C++ nodes in src/, headers in include/perception_pipeline/
- Python ROS nodes in perception_pipeline/
- Python utility scripts in scripts/ (not ROS nodes)

```bash
cd ~/ros2_ws
colcon build --packages-select perception_pipeline
source install/setup.bash
```

## Key Commands
```bash
# play dataset bag
ros2 bag play ~/ros2_ws/src/perception_pipeline/data/bags/tless_scene1 --loop

# run detector
ros2 run perception_pipeline detector_node \
  --ros-args -p model_path:=/home/adrian/ros2_ws/src/perception_pipeline/models/yolo11n.pt

# run pose estimator
ros2 run perception_pipeline pose_estimation_node \
  --ros-args --params-file config/pose_estimation_params.yaml

# visualize
rviz2 -d config/tless_viz.rviz

# download + extract dataset
python3 scripts/download_bop.py

# convert dataset to bag
python3 scripts/bop_to_rosbag.py
```

## Important Notes
- numpy must be <2 due to cv_bridge compatibility: `pip3 install "numpy<2" --break-system-packages`
- data/ and models/ are gitignored — must be generated locally
- T-LESS scene 1 contains objects 2, 25, 29, 30
- Time in ROS 2 bags is in nanoseconds (not seconds like ROS 1)
- Depth values in T-LESS are in mm, multiply by depth_scale then divide by 1000 for meters
- Python inference uses ultralytics directly (not OpenCV DNN — too old at 4.5.4)
- YOLO model accepts .pt files directly, no ONNX export needed until TensorRT optimization

## Package Structure
- src/                    C++ source files
- include/                C++ headers
- perception_pipeline/    Python ROS nodes
- scripts/                Python utilities (download, bag conversion)
- models/                 YOLO model weights (gitignored)
- data/tless/             T-LESS dataset (gitignored)
- data/bags/              ROS 2 bag files (gitignored)
- config/                 RViz configs, parameter files
- launch/                 ROS 2 launch files

## Current Status
- [x] ROS 2 package structure
- [x] T-LESS BOP dataset downloaded and converted to ROS 2 bag
- [x] RViz visualization config
- [x] YOLOv8/YOLO11 Python detection node
- [x] Pose estimation node (FoundationPose)
- [ ] Evaluation against ground truth
- [ ] TensorRT optimization
- [ ] Launch file wiring full pipeline
## Next Task: Pose Estimation Node

Create `perception_pipeline/pose_estimation_node.py`:

### Subscribes to
- `/camera/rgb/image_raw` (sensor_msgs/Image)
- `/camera/depth/image_raw` (sensor_msgs/Image)
- `/camera/camera_info` (sensor_msgs/CameraInfo)
- `/detections` (vision_msgs/Detection2DArray)

### For each detection
- Crop RGB and depth using the bounding box
- Load the T-LESS object mesh from `data/tless/models/obj_XXXXXX.ply`
- Run FoundationPose inference in CPU mode
- Publish estimated pose to `/pose_estimation/obj_XXXXXX` (geometry_msgs/PoseStamped)

### Parameters
- `mesh_dir`: path to T-LESS models folder (default: `data/tless/models`)
- `foundationpose_path`: path to FoundationPose repo (default: `~/ros2_ws/src/FoundationPose`)

### Important
- FoundationPose is cloned at `~/ros2_ws/src/FoundationPose`
- Add FoundationPose to `sys.path` at startup
- CPU mode only — no CUDA
- T-LESS meshes are PLY files: `obj_000001.ply`, `obj_000002.ply` etc.
- Object ID comes from the detection `class_id` field
- Use `trimesh` to load PLY meshes
