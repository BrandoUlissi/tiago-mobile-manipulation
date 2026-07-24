from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    ld = LaunchDescription()

    ########################################################################################
    # Declare Launch Arguments #############################################################
    ########################################################################################
    ld.add_action(DeclareLaunchArgument('group_number', default_value='9'))
    ld.add_action(DeclareLaunchArgument('moveit', default_value='true'))
    ld.add_action(DeclareLaunchArgument('is_public_sim', default_value='false'))
    ld.add_action(DeclareLaunchArgument('rviz', default_value='True'))
    ld.add_action(DeclareLaunchArgument('slam', default_value='True'))
    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation/Gazebo clock'))
    ld.add_action(DeclareLaunchArgument('namespace', default_value='', description='Namespace for the explore node'))

    use_sim_time = LaunchConfiguration('use_sim_time')
    namespace = LaunchConfiguration('namespace')

    ########################################################################################
    # TIAGO Simulation #####################################################################
    ########################################################################################
    tiago_gazebo_launch_file = os.path.join(
        get_package_share_directory('tiago_gazebo'), 'launch', 'tiago_gazebo.launch.py'
    )
    tiago_gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(tiago_gazebo_launch_file),
        launch_arguments={
            'moveit': 'true',
            'use_sim_time': use_sim_time  # Pass use_sim_time to tiago_gazebo
        }.items()
    )

    tiago_nav_bringup_launch_file = os.path.join(
        get_package_share_directory('tiago_2dnav'), 'launch', 'tiago_nav_bringup.launch.py'
    )
    tiago_nav_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(tiago_nav_bringup_launch_file),
        launch_arguments={
            'is_public_sim': 'false',
            'rviz': 'True',
            'slam': 'True',
            'use_sim_time': use_sim_time  # Pass use_sim_time to tiago_nav_bringup
        }.items()
    )

    ########################################################################################
    # TASK Nodes ###########################################################################
    ########################################################################################
    sm_mission_manager = Node(
        package='sm_mission_manager',
        executable='sm_mission_manager',
        name='sm_mission_manager',
        output='screen',
        namespace=namespace  # Pass namespace to sm_task_1
    )

    sm_arm_in_safe_position_node = Node(
        package='arm_in_safe_pos',
        executable='sm_arm_in_safe_pos',
        name='sm_arm_in_safe_pos',
        output='screen',
        namespace=namespace  # Pass namespace to sm_arm_in_safe_pos
    )

    yolo_node = Node(
        package='yolo_tiago',
        executable='yolo_tiago_node',
        name='yolo_tiago_node',
        output='screen',
        namespace=namespace  # Pass namespace to sm_arm_in_safe_pos
    )

    tracking_heading_node = Node(
        package='tracking_tiago',
        executable='tracking_heading_node',
        name='tracking_heading_node',
        output='screen',
        namespace=namespace  # Pass namespace to sm_arm_in_safe_pos
    )

    distance_detection_node = Node(
        package='approach_target_tiago',
        executable='distance_detection_node',
        name='distance_detection_node',
        output='screen',
        namespace=namespace  # Pass namespace to sm_arm_in_safe_pos
    )

    approach_target_tiago_node = Node(
        package='approach_target_tiago',
        executable='approach_target_node',
        name='approach_target_node',
        output='screen',
        namespace=namespace  # Pass namespace to sm_arm_in_safe_pos
    )

    ########################################################################################
    # Explore Lite Launch ##################################################################
    ########################################################################################
    config = os.path.join(get_package_share_directory('explore_lite'), 'config', 'params.yaml')

    explore_lite_node = Node(
        package='explore_lite',
        executable='explore',
        name='explore_node',
        namespace=namespace,
        parameters=[config, {'use_sim_time': use_sim_time}],
        output='screen',
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]  # Rimuovi se non necessario
    )

    ########################################################################################
    # Publish Initial /explore/resume Message ##############################################
    ########################################################################################
    publish_resume_false = ExecuteProcess(
        cmd=['ros2', 'topic', 'pub', '/explore/resume', 'std_msgs/msg/Bool', '{data: false}', '--once'],
        output='screen'
    )

    ########################################################################################
    # Add Actions ##########################################################################
    ########################################################################################
    ld.add_action(tiago_gazebo_launch)
    #ld.add_action(tiago_nav_bringup_launch)
    ld.add_action(sm_mission_manager)
    ld.add_action(sm_arm_in_safe_position_node)
    ld.add_action(publish_resume_false)  # Publish the initial message
    ld.add_action(explore_lite_node)
    ld.add_action(yolo_node)
    ld.add_action(tracking_heading_node)
    ld.add_action(approach_target_tiago_node)
    ld.add_action(distance_detection_node)
    
    return ld