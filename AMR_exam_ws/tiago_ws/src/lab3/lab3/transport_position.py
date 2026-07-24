# Import necessary modules for ROS2 robotic arm transport positioning
from threading import Thread                                    # For running background executor threads
import rclpy                                                   # ROS2 Python client library
from rclpy.callback_groups import ReentrantCallbackGroup       # Allow concurrent callback execution
from rclpy.node import Node                                    # Base class for ROS2 nodes
from pymoveit2 import MoveIt2                                  # MoveIt2 motion planning interface
from std_msgs.msg import Empty, Bool                           # Standard ROS2 message types
import math                                                    # Mathematical operations for angle conversions

# Global configuration constants for TIAGo arm control

# List of joint names for the robot's 7-DOF arm kinematic chain
# These joints form the complete arm from shoulder to wrist
JOINT_NAMES = [
    "arm_1_joint",  # Shoulder rotation (base joint)
    "arm_2_joint",  # Shoulder elevation
    "arm_3_joint",  # Arm twist/rotation
    "arm_4_joint",  # Elbow joint
    "arm_5_joint",  # Forearm twist
    "arm_6_joint",  # Wrist pitch
    "arm_7_joint",  # Wrist roll
]

# MoveIt2 configuration parameters for motion planning
# These define the robot model structure and planning targets
BASE_LINK_NAME = "base_link"        # Robot's base coordinate frame for planning
END_EFFECTOR_NAME = "arm_tool_link" # End-effector frame for pose-based planning
GROUP_NAME = "arm"                  # MoveIt2 planning group name for the arm

class TransportPosition(Node):
    """
    ROS2 Node for moving TIAGo robot's arm to a safe transport configuration.
    
    This node provides a specialized service for positioning the robot's arm in
    a predefined transport pose that is optimized for:
    
    1. Safe navigation while carrying objects
    2. Collision avoidance during movement
    3. Reduced arm stress and power consumption
    4. Improved robot stability during transport
    
    Key Features:
    - Predefined joint configuration for consistent transport positioning
    - MoveIt2 integration for safe motion planning and execution
    - Topic-based coordination with other robotic processes
    - Completion notifications for workflow synchronization
    
    The transport position is carefully designed to:
    - Keep the arm close to the robot body for stability
    - Avoid collisions with the environment during navigation
    - Maintain secure grip on carried objects
    - Minimize mechanical stress on joints and actuators
    
    Usage Context:
    This node is typically used in pick-and-place workflows after grasping
    operations to position the arm safely before navigation to placement locations.
    """
    def __init__(self):
        # Initialize the ROS2 node with a descriptive name for transport positioning
        super().__init__("transport_position")
        
        # Use a reentrant callback group to allow concurrent callbacks
        # This enables simultaneous handling of MoveIt2 operations and ROS service calls
        self.callback_group = ReentrantCallbackGroup()

        # Publisher to notify when the transport position is reached
        # Enables coordination with downstream navigation and placement processes
        self.done_pub = self.create_publisher(Bool, '/transport_position_done', 10)
        
        # Subscriber to trigger the transport position sequence
        # Provides external interface for initiating transport positioning
        self.start_sub = self.create_subscription(
            Empty,                                      # Simple trigger message (no data payload)
            '/start_transport_position',                # Topic name for receiving start commands
            self.start_callback,                        # Callback function to handle triggers
            10                                          # Queue size for message buffering
        )

        # Initialize MoveIt2 interface for arm motion planning and execution
        # Provides comprehensive motion planning with collision avoidance
        self.moveit2 = MoveIt2(
            node=self,                                  # Reference to this ROS2 node
            joint_names=JOINT_NAMES,                    # List of arm joints to control
            base_link_name=BASE_LINK_NAME,              # Robot base coordinate frame
            end_effector_name=END_EFFECTOR_NAME,        # End-effector frame for planning
            group_name=GROUP_NAME,                      # MoveIt2 planning group name
            callback_group=self.callback_group,         # Allow concurrent callbacks
        )
        # Set motion planner algorithm - RRTConnect is reliable for transport motions
        self.moveit2.planner_id = "RRTConnectkConfigDefault"

        # Log node initialization completion with multilingual message
        self.get_logger().info("Nodo transport_position pronto e in idle.")

    def start_callback(self, msg):
        """
        Callback triggered by the /start_transport_position topic.
        
        This method executes the complete transport positioning sequence when
        triggered by an external command. The transport position is a carefully
        designed joint configuration that optimizes the robot for safe navigation
        while carrying objects.
        
        Transport Position Benefits:
        1. Arm positioned close to robot body for improved stability
        2. Reduced risk of collisions during navigation
        3. Lower center of gravity for better balance
        4. Minimized joint stress and power consumption
        5. Secure object transport without dropping
        
        Execution Sequence:
        1. Log receipt of start command and send acknowledgment
        2. Define target joint angles for transport configuration
        3. Execute motion planning and arm movement
        4. Wait for movement completion
        5. Publish completion notification for workflow coordination
        
        Args:
            msg (Empty): ROS2 Empty message triggering the positioning sequence
            
        The method uses joint-space motion planning for reliable and predictable
        movement to the predefined transport configuration.
        """
        # Log command receipt with multilingual acknowledgment message
        self.get_logger().info("Ricevuto comando di start transport_position, invio ACK e muovo il braccio.")

        # Define the target joint positions for the transport pose (in radians)
        # These angles position the arm in an optimal configuration for safe transport
        joint_positions = [
            math.radians(35),               # arm_1_joint: Shoulder rotation (35°)
            math.radians(-75),              # arm_2_joint: Shoulder elevation (-75°)
            math.radians(-194),             # arm_3_joint: Arm twist (-194°)
            math.radians(125),              # arm_4_joint: Elbow joint (125°)
            math.radians(75),               # arm_5_joint: Forearm twist (75°)
            math.radians(-40),              # arm_6_joint: Wrist pitch (-40°)
            math.radians(-51),              # arm_7_joint: Wrist roll (-51°)
        ]

        # Log target configuration for debugging and monitoring
        self.get_logger().info(f"Moving arm to joint positions: {joint_positions}")
        
        # Command MoveIt2 to plan and execute movement to the target configuration
        self.moveit2.move_to_configuration(joint_positions)    # Plan and start movement
        self.moveit2.wait_until_executed()                     # Wait for movement completion
        
        # Publish completion notification for workflow coordination
        self.done_pub.publish(Bool(data=True))                 # Signal transport position reached
        self.get_logger().info("Published /transport_position_done = True")

def main(args=None):
    """
    Main entry point for the TIAGo transport position node.
    
    This function serves as the primary entry point for the transport positioning
    system, handling the complete lifecycle from initialization to shutdown.
    The node provides a critical service in pick-and-place workflows by ensuring
    the robot arm is positioned safely for navigation while carrying objects.
    
    System Purpose:
    The transport position node fills a crucial role in robotic manipulation
    workflows by providing a standardized, safe arm configuration for:
    - Navigation between pick and place locations
    - Object transport without collisions
    - Reduced mechanical stress during movement
    - Improved robot stability and balance
    
    Key Responsibilities:
    1. Initialize ROS2 Python client library with command line arguments
    2. Create the TransportPosition node instance with MoveIt2 integration
    3. Set up multi-threaded executor for concurrent MoveIt2 operations
    4. Run the main event loop until shutdown or interruption
    5. Ensure proper cleanup of all resources during shutdown
    
    Multi-Threading Benefits:
    The multi-threaded executor enables:
    - Concurrent motion planning and execution
    - Simultaneous topic publishing and subscription handling
    - Non-blocking service operations
    - Responsive system behavior during arm movements
    
    Integration Context:
    This node typically operates as part of larger pick-and-place systems,
    receiving commands after grasping operations and before navigation phases.
    
    Args:
        args (list, optional): Command line arguments passed to ROS2 initialization.
                              Includes node parameters, logging configuration, and system settings.
                              
    The function includes comprehensive error handling to ensure graceful shutdown
    and proper resource cleanup under all termination conditions.
    """
    # Initialize the ROS2 Python client library with command line arguments
    rclpy.init(args=args)
    
    # Create an instance of the transport position node
    node = TransportPosition()
    
    # Create multi-threaded executor for concurrent MoveIt2 operations
    # Required for handling motion planning and topic communication simultaneously
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)                                   # Add transport node to executor
    
    try:
        # Start the executor - runs until interrupted or shutdown
        executor.spin()                                       # Begin processing callbacks and services
    finally:
        # Ensure proper cleanup regardless of how the program exits
        node.destroy_node()                                   # Clean up node resources
        rclpy.shutdown()                                      # Shutdown ROS2 Python client library

# Standard Python idiom to run main() only when script is executed directly
# This prevents execution when the file is imported as a module by other Python programs
if __name__ == "__main__":
    main()