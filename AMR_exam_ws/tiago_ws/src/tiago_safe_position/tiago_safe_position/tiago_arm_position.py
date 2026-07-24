import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from pymoveit2 import MoveIt2, GripperInterface
import math

# TIAGo robot joint configuration - defines all joints involved in arm and torso movement
# These joints form a kinematic chain from the torso base to the end effector
JOINT_NAMES = [
    "torso_lift_joint",    # Controls vertical torso movement (up/down)
    "arm_1_joint",         # Shoulder rotation (base joint of arm)
    "arm_2_joint",         # Shoulder elevation (second joint)
    "arm_3_joint",         # Arm twist/rotation
    "arm_4_joint",         # Elbow joint
    "arm_5_joint",         # Forearm twist
    "arm_6_joint",         # Wrist pitch
    "arm_7_joint",         # Wrist roll
    "arm_tool_joint",      # Tool interface joint
]

# Robot kinematic chain configuration for MoveIt2 planning
BASE_LINK_NAME = "base_link"        # Root link of the robot coordinate system
END_EFFECTOR_NAME = "arm_tool_link" # Final link where tools/grippers are attached
GROUP_NAME = "arm_torso"            # MoveIt2 planning group that includes both arm and torso

# TIAGo gripper configuration - two-finger parallel gripper setup
GRIPPER_JOINT_NAMES = [
    "gripper_left_finger_joint",   # Left finger actuator joint
    "gripper_right_finger_joint",  # Right finger actuator joint
]

# Gripper position presets (in meters)
OPEN_GRIPPER_JOINT_POSITIONS = [0.04, 0.04]    # Fully open position (4cm separation)
CLOSED_GRIPPER_JOINT_POSITIONS = [0.0, 0.0]    # Fully closed position (fingers touching)

# Gripper control configuration for MoveIt2
GRIPPER_GROUP_NAME = "gripper"                           # MoveIt2 planning group for gripper
GRIPPER_COMMAND_ACTION_NAME = "gripper_controller/joint_trajectory"  # Action server for gripper control

class ArmSafePositionNode(Node):
    """
    ROS2 node for controlling TIAGo robot arm movement to a predefined safe position.
    
    This node provides a service-like interface that:
    1. Listens for commands on /arm_to_safe topic
    2. Moves the arm and torso to a safe configuration using MoveIt2
    3. Sets the gripper to a safe open position
    4. Publishes completion status on /arm_in_safe topic
    
    The safe position is designed to:
    - Avoid collisions with the environment
    - Provide clearance for robot movement
    - Position the arm in a compact, stable configuration
    """

    def __init__(self):
        # Initialize the ROS2 node with a descriptive name
        super().__init__("arm_safe_position_node")

        # Callback group for MoveIt2 and Gripper operations
        # ReentrantCallbackGroup allows multiple callbacks to execute simultaneously
        # This is essential for MoveIt2 which uses multiple internal callbacks
        self.callback_group = ReentrantCallbackGroup()

        # MoveIt2 interface for motion planning and execution
        # Handles trajectory planning, collision avoidance, and joint control
        self.moveit2 = MoveIt2(
            node=self,                          # Reference to this ROS2 node
            joint_names=JOINT_NAMES,            # List of joints to control
            base_link_name=BASE_LINK_NAME,      # Robot base coordinate frame
            end_effector_name=END_EFFECTOR_NAME, # Target end effector frame
            group_name=GROUP_NAME,              # MoveIt2 planning group name
            callback_group=self.callback_group,  # Allow concurrent callbacks
        )
        # Set motion planner algorithm - RRTConnect is reliable for arm movements
        self.moveit2.planner_id = "RRTConnectkConfigDefault"

        # Gripper interface for controlling the two-finger parallel gripper
        # Provides high-level commands for opening, closing, and positioning
        self.gripper_interface = GripperInterface(
            node=self,                                      # Reference to this ROS2 node
            gripper_joint_names=GRIPPER_JOINT_NAMES,        # List of gripper joints to control
            open_gripper_joint_positions=OPEN_GRIPPER_JOINT_POSITIONS,    # Predefined open position
            closed_gripper_joint_positions=CLOSED_GRIPPER_JOINT_POSITIONS, # Predefined closed position
            gripper_group_name=GRIPPER_GROUP_NAME,          # MoveIt2 group for gripper planning
            callback_group=self.callback_group,             # Allow concurrent callbacks
            gripper_command_action_name=GRIPPER_COMMAND_ACTION_NAME, # Action server for gripper control
        )

        # Subscriber to receive state machine commands
        # Listens for trigger messages to initiate safe position movement
        self.subscription = self.create_subscription(
            String,                    # Message type for simple text commands
            "/arm_to_safe",           # Topic name for receiving movement commands
            self.command_callback,     # Callback function to handle incoming messages
            10                        # Queue size for message buffering
        )

        # Publisher to notify when safe position movement is complete
        # Allows other nodes to know when the arm has finished moving
        self.done_publisher = self.create_publisher(
            String,                   # Message type for completion notification
            "/arm_in_safe",          # Topic name for publishing completion status
            10                       # Queue size for message buffering
        )

        # Log successful initialization and ready state
        self.get_logger().info("Arm node ready and idle, waiting for /arm_to_safe message.")

    def command_callback(self, msg):
        """
        Callback function for processing incoming commands on /arm_to_safe topic.
        
        Args:
            msg (String): ROS2 String message containing the command
        
        Currently supported commands:
            - "go": Initiates movement to safe position
        """
        command = msg.data
        self.get_logger().info(f"Received command: {command}")

        # Process the command and trigger appropriate action
        if command == "go":
            self.move_to_safe_position()

    def move_to_safe_position(self):
        """
        Moves the TIAGo robot arm and torso to a predefined safe configuration.
        
        The safe position is carefully designed to:
        1. Keep the arm close to the body to minimize collision risk
        2. Position joints in mechanically stable configurations
        3. Provide clearance for base movement and navigation
        4. Set the gripper to a safe open position
        
        Joint angles are specified in radians (converted from degrees for readability)
        """
        # Define target joint positions for safe configuration
        # Values are carefully chosen to avoid self-collisions and provide stability
        joint_positions = [
            0.25,                           # torso_lift_joint: Raise torso for better clearance
            math.radians(35),               # arm_1_joint: Shoulder rotation (35°)
            math.radians(-75),              # arm_2_joint: Shoulder elevation (-75°, brings arm inward)
            math.radians(-194),             # arm_3_joint: Arm twist (-194°)
            math.radians(125),              # arm_4_joint: Elbow bend (125°, folded configuration)
            math.radians(75),               # arm_5_joint: Forearm twist (75°)
            math.radians(-40),              # arm_6_joint: Wrist pitch (-40°)
            math.radians(-51),              # arm_7_joint: Wrist roll (-51°)
            0.0                             # arm_tool_joint: Tool interface neutral position
        ]

        # Execute the arm movement using MoveIt2 motion planning
        self.get_logger().info(f"Moving arm to joint positions: {joint_positions}")
        self.moveit2.move_to_configuration(joint_positions)  # Plan and execute trajectory
        self.moveit2.wait_until_executed()                   # Block until movement completes

        # Move gripper to specific safe position (slightly open for safety)
        # 0.05m provides enough opening to avoid damage while being compact
        self.get_logger().info("Moving gripper to 0.05 position")
        self.gripper_interface.move_to_position(0.05)        # Move to 5cm opening
        self.gripper_interface.wait_until_executed()         # Block until gripper movement completes

        # Notify other nodes that the safe position has been achieved
        done_msg = String()
        done_msg.data = "done"               # Simple completion signal
        self.done_publisher.publish(done_msg)
        self.get_logger().info("Arm in safe position. Published done.")

def main():
    """
    Main entry point for the TIAGo arm safe position node.
    
    Sets up a multi-threaded executor to handle concurrent operations required by:
    - MoveIt2 motion planning (uses multiple internal threads)
    - Gripper control (action client requires separate thread)
    - ROS2 callback processing (subscription and publishing)
    
    The multi-threaded approach ensures responsive operation and prevents deadlocks
    that could occur with single-threaded execution of complex motion planning.
    """
    # Initialize the ROS2 Python client library
    rclpy.init()
    
    # Create the arm safe position node instance
    node = ArmSafePositionNode()

    # Create multi-threaded executor for concurrent callback processing
    # This is essential for MoveIt2 which requires multiple threads for:
    # - Planning scene updates
    # - Trajectory execution monitoring  
    # - Action client/server communication
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        # Start the executor - this will run until interrupted
        # The executor handles all callbacks and maintains node operation
        executor.spin()
    except KeyboardInterrupt:
        # Handle Ctrl+C gracefully
        node.get_logger().info("Shutting down arm node.")
    finally:
        # Ensure proper cleanup regardless of how the program exits
        node.destroy_node()  # Clean up node resources and stop any ongoing operations
        rclpy.shutdown()     # Shutdown ROS2 Python client library

# Standard Python idiom to run main() only when script is executed directly
if __name__ == "__main__":
    main()
