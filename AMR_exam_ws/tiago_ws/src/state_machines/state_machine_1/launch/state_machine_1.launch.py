from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
import os


def generate_launch_description():
    safe_position_launch_path = os.path.join(
        get_package_share_directory('tiago_safe_position'),
        'launch',
        'safe_position_launch.py'
    )

    explore_lite_launch_path = os.path.join(
        get_package_share_directory('explore_lite'),
        'launch',
        'explore.launch.py'
    )

    safe_position_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(safe_position_launch_path)
    )

    explore_lite_launch = IncludeLaunchDescription(
         PythonLaunchDescriptionSource(explore_lite_launch_path)
    )  

    # Combine nodes/actions into a single launch description
    return LaunchDescription([
        safe_position_launch,
        explore_lite_launch,
    ])