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

    bag_play = ExecuteProcess(
        cmd=["ros2", "bag", "play", LaunchConfiguration("bag_path"), "--loop"],
        output="screen",
    )
    bbox_viz_node = Node(
        package="perception_pipeline",
        executable="bbox_viz_node",
        name="bbox_viz",
        output="screen",
    )

    point_localization_node = Node(
        package="perception_pipeline",
        executable="point_localization_node",
        name="point_localization",
        output="screen",
    )

    return LaunchDescription(
            [
                bag_path_arg,
                bag_play,
                bbox_viz_node,
                point_localization_node,
            ]
    )
