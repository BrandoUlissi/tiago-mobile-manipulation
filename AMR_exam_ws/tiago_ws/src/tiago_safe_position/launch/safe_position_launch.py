from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    Generate a launch description for the TIAGo safe position package.
    
    This launch file starts two nodes simultaneously:
    1. Arm position control node - manages TIAGo robot arm positioning
    2. Head position control node - manages TIAGo robot head positioning
    
    Both nodes work together to move the robot to a safe configuration,
    typically used during startup, shutdown, or when transitioning between tasks.
    
    Returns:
        LaunchDescription: Complete launch configuration with both nodes
    """
    # Node for arm position control
    # This node handles moving the TIAGo robot's arm to a predefined safe position
    arm_position_node = Node(
        package='tiago_safe_position',           # Package containing the arm control executable
        executable='tiago_arm_position',         # Executable name for arm position control
        name='tiago_arm_position',              # Unique node name in the ROS2 graph
        output='screen',                        # Display node output in the terminal for debugging
    )

    # Node for head position control
    # This node handles moving the TIAGo robot's head to a predefined safe position
    head_position_node = Node(
        package='tiago_safe_position',           # Package containing the head control executable
        executable='tiago_head_position',        # Executable name for head position control
        name='tiago_head_position',             # Unique node name in the ROS2 graph
        output='screen',                        # Display node output in the terminal for debugging
    )

    # Return the complete launch description containing both nodes
    # The nodes will be launched simultaneously and run in parallel
    # This ensures coordinated movement of both arm and head to safe positions
    return LaunchDescription([
        arm_position_node,    # Start arm positioning node
        head_position_node    # Start head positioning node
    ])
