import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bag_path_arg = DeclareLaunchArgument(
        "bag_path",
        default_value=os.path.expanduser(
            "~/ros2_ws/src/perception_pipeline/data/bags/tless_scene8"
        ),
        description="Path to the ROS 2 bag directory",
    )

    model_path_arg = DeclareLaunchArgument(
        "model_path",
        default_value=os.path.expanduser(
            "~/ros2_ws/src/perception_pipeline/data/tless/models_cad/obj_000025.ply"
        ),
        description="Path to the object CAD model PLY file",
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

    icp_node = Node(
        package="perception_pipeline",
        executable="icp_node",
        name="icp",
        output="screen",
        parameters=[{"model_path": LaunchConfiguration("model_path")}],
    )

    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_camera",
        arguments=["0", "0", "0", "0", "0", "0", "world", "camera_link"],
    )


    return LaunchDescription([
        bag_path_arg,
        model_path_arg,
        bbox_viz_node,
        bag_play,
        point_localization_node,
        icp_node,
        static_tf
])