from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

import os

def generate_launch_description():
    # Declare launch arguments for marker detection
    marker_id_arg = DeclareLaunchArgument('marker_id', default_value='582')
    marker_size_arg = DeclareLaunchArgument('marker_size', default_value='0.04667')
    marker_frame_arg = DeclareLaunchArgument('marker_frame', default_value='aruco_marker_frame')
    reference_frame_arg = DeclareLaunchArgument('reference_frame', default_value='')
    corner_refinement_arg = DeclareLaunchArgument('corner_refinement', default_value='LINES')

    # Node for ArUco marker detection (aruco_ros/single)
    aruco_node = Node(
        package='aruco_ros',
        executable='single',
        name='aruco_single',
        output='screen',
        parameters=[{
            'image_is_rectified': True,
            'marker_size': LaunchConfiguration('marker_size'),
            'marker_id': LaunchConfiguration('marker_id'),
            'reference_frame': LaunchConfiguration('reference_frame'),
            'camera_frame': 'head_front_camera_rgb_optical_frame',
            'marker_frame': LaunchConfiguration('marker_frame'),
            'corner_refinement': LaunchConfiguration('corner_refinement'),
            'use_sensor_data_qos': True
        }],
        remappings=[
            ('camera_info', '/head_front_camera/rgb/camera_info'),
            ('image', '/head_front_camera/rgb/image_raw'),
        ]
    )

    # Path to the safe position launch file
    safe_position_launch_path = os.path.join(
        get_package_share_directory('tiago_safe_position'),
        'launch',
        'safe_position_launch.py'
    )

    # Node for autonomous localization
    autonomous_localization_node = Node(
        package='autonomous_localization',
        executable='autonomous_localization',
        name='autonomous_localization_node',
        output='screen'
    )

    # Node for navigation to the pick location
    navigation_to_pick_node = Node(
        package='navigation_from_text',
        executable='navigation_to_pick',
        name='navigation_to_pick_node',
        output='screen'
    )

    # Node for broadcasting the ArUco grasp pose
    aruco_grasp_node = Node(
        package='lab3',
        executable='2_aruco_grasp_pose_broadcaster',
        name='aruco_grasp_node',
        output='screen'
    )

    # Node for controlling the arm during pick operation
    pick_arm_node = Node(
        package='lab3',
        executable='pick_arm',
        name='pick_arm_node',
        output='screen',
        parameters=[{'cube_id': '582'}]  # Uncomment this parameter if needed
    )

    # Node for controlling the torso during pick operation
    pick_torso_node = Node(
        package='lab3',
        executable='pick_torso',
        name='pick_torso_node',
        output='screen',
        parameters=[{'cube_id': '582'}]  # Uncomment this parameter if needed
    )

    # Node for controlling the arm during place operation
    place_arm_node = Node(
        package='lab3',
        executable='place_arm',
        name='place_arm_node',
        output='screen',
        parameters=[{'cube_id': '582'}]  # Uncomment this parameter if needed
    )

    # Node for moving to the transport position
    transport_position_node = Node(
        package='lab3',
        executable='transport_position',
        name='transport_position_node',
        output='screen'
    )

    # Node for navigation to the place location
    navigation_to_place_node = Node(
        package='navigation_from_text',
        executable='navigation_to_place',
        name='navigation_to_place_node',
        output='screen'
    )

    # Include the safe position launch file
    safe_position_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(safe_position_launch_path)
    )

    # Return the complete launch description with all nodes and arguments
    return LaunchDescription([
        marker_id_arg,
        marker_size_arg,
        marker_frame_arg,
        reference_frame_arg,
        corner_refinement_arg,

        safe_position_launch,
        autonomous_localization_node,
        # aruco_node,  # Uncomment to enable ArUco node
        navigation_to_pick_node,
        aruco_grasp_node,
        pick_arm_node,
        pick_torso_node,
        place_arm_node,
        transport_position_node,
        navigation_to_place_node
    ])
