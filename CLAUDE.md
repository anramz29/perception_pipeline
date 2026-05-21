# CLAUDE.md

## Project Overview
ROS 2 C++ package for 6-DoF object pose estimation using the T-LESS BOP dataset.
Target use case: robotic manipulation perception pipeline.

## Build System
- ROS 2 Humble + ament_cmake
- C++ nodes in src/, headers in include/perception_pipeline/
- Python utility scripts in scripts/ (not ROS nodes)

```bash
cd ~/ros2_ws
colcon build --packages-select perception_pipeline
source install/setup.bash
```

## Key Commands
```bash
# play dataset bag
ros2 bag play data/bags/tless_scene1 --loop

# visualize
rviz2 -d config/tless_viz.rviz

# download + extract dataset
python3 scripts/download_bop.py

# convert dataset to bag
python3 scripts/bop_to_rosbag.py
```

## Important Notes
- numpy must be <2 due to cv_bridge compatibility: `pip3 install "numpy<2" --break-system-packages`
- data/ is gitignored — dataset must be downloaded locally
- T-LESS scene 1 contains objects 2, 25, 29, 30
- Time in ROS 2 bags is in nanoseconds (not seconds like ROS 1)
- Depth values in T-LESS are in mm, multiply by depth_scale then divide by 1000 for meters

## Package Structure
- src/          C++ source files (perception nodes go here)
- include/      C++ headers
- scripts/      Python utilities (download, bag conversion)
- data/tless/   T-LESS dataset (gitignored)
- data/bags/    ROS 2 bag files (gitignored)
- config/       RViz configs, parameter files
- launch/       ROS 2 launch files
