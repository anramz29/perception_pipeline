import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bag_path_arg = DeclareLaunchArgument(
        "bag_path",
        default_value=os.path.expanduser(
            "~/ros2_ws/src/perception_pipeline/data/bags/tless_scene1"
        ),
        description="Path to the ROS 2 bag directory",
    )
    model_path_arg = DeclareLaunchArgument(
        "model_path",
        default_value=os.path.expanduser(
            "~/ros2_ws/src/perception_pipeline/models/yolo11n.onnx"
        ),
        description="Path to the YOLO ONNX model file",
    )
    conf_thresh_arg = DeclareLaunchArgument(
        "confidence_threshold",
        default_value="0.25",
        description="Minimum confidence score to keep a detection",
    )
    nms_thresh_arg = DeclareLaunchArgument(
        "nms_threshold",
        default_value="0.45",
        description="IoU threshold for non-maximum suppression",
    )

    bag_play = ExecuteProcess(
        cmd=["ros2", "bag", "play", LaunchConfiguration("bag_path"), "--loop"],
        output="screen",
    )

    detector_node = Node(
        package="perception_pipeline",
        executable="detector_node",
        name="detector",
        output="screen",
        parameters=[
            {
                "model_path": LaunchConfiguration("model_path"),
                "confidence_threshold": LaunchConfiguration("confidence_threshold"),
                "nms_threshold": LaunchConfiguration("nms_threshold"),
            }
        ],
    )

    return LaunchDescription(
        [
            bag_path_arg,
            model_path_arg,
            conf_thresh_arg,
            nms_thresh_arg,
            bag_play,
            detector_node,
        ]
    )
