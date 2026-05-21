import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('perception_pipeline')

    bag_path_arg = DeclareLaunchArgument(
        'bag_path',
        default_value=os.path.join(
            os.path.expanduser('~'), 'ros2_ws', 'src',
            'perception_pipeline', 'data', 'bags', 'tless_scene1'
        ),
        description='Path to the ROS 2 bag directory'
    )

    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value=os.path.join(pkg_share, 'models', 'yolo11n.pt'),
        description='Path to the YOLO model (.pt) file'
    )

    confidence_arg = DeclareLaunchArgument(
        'confidence_threshold',
        default_value='0.25',
        description='Detection confidence threshold (0.0–1.0)'
    )

    play_bag = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', LaunchConfiguration('bag_path'), '--loop'],
        output='screen'
    )

    detector_node = Node(
        package='perception_pipeline',
        executable='detector_node',
        name='detector',
        parameters=[{
            'model_path': LaunchConfiguration('model_path'),
            'confidence_threshold': LaunchConfiguration('confidence_threshold'),
        }],
        output='screen'
    )

    return LaunchDescription([
        bag_path_arg,
        model_path_arg,
        confidence_arg,
        play_bag,
        detector_node,
    ])
