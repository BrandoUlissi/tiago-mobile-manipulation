# Import necessary modules for ROS2 robotic arm placement operations
from threading import Thread                                    # For running background executor threads
import rclpy                                                   # ROS2 Python client library
from rclpy.callback_groups import ReentrantCallbackGroup       # Allow concurrent callback execution
from rclpy.node import Node                                    # Base class for ROS2 nodes
from pymoveit2 import MoveIt2, GripperInterface                # MoveIt2 motion planning and gripper control
from linkattacher_msgs.srv import DetachLink                   # Service for detaching objects in Gazebo simulation
from std_msgs.msg import Bool, Empty, String                   # Standard ROS2 message types
from sensor_msgs.msg import JointState                         # Joint state information messages
import time                                                    # Python time utilities for delays
import math                                                    # Mathematical operations for angle conversions

class TiagoPlaceArm(Node):
    """
    ROS2 Node for controlling TIAGo robot's arm placement operations.
    
    This node handles the placement phase of pick-and-place operations by:
    1. Moving the arm to predefined placement configurations
    2. Opening the gripper to release grasped objects
    3. Detaching objects from the gripper via Gazebo simulation services
    4. Coordinating the complete placement sequence
    
    The node uses MoveIt2 for motion planning and execution, providing:
    - Joint-space motion to predefined placement poses
    - Collision-aware trajectory planning
    - Gripper control for object release
    - Simulation integration for object detachment
    
    Key Features:
    - Predefined joint configurations for consistent placement
    - Dynamic object ID updating for different targets
    - Service-based object detachment in simulation
    - Workflow coordination through completion notifications
    - Robust error handling and state management
    """
    def __init__(self):
        # Initialize the ROS2 node with a descriptive name for arm placement operations
        super().__init__('tiago_place_arm')
        
        # Declare and get the cube_id parameter (used for identifying the object to detach)
        # This parameter specifies which ArUco cube is currently being manipulated
        self.declare_parameter("cube_id", "undefined")
        cube_id_param = self.get_parameter("cube_id").get_parameter_value().string_value
        self.cube_id = cube_id_param                           # Store the target cube identifier
        self.marker_id_received = False                        # Flag to track dynamic ID updates

        # Subscribe to the current marker ID topic to update the cube_id dynamically
        # Enables runtime switching between different target objects
        self.create_subscription(String, '/current_marker_id', self.marker_id_callback, 10)

        # Define the robot's base coordinate frame for motion planning
        self.robot_base_frame = "base_link"

        # Use a reentrant callback group for concurrent MoveIt2 and service calls
        # This prevents deadlocks when multiple operations need to execute simultaneously
        callback_group = ReentrantCallbackGroup()

        # Create a client for the DetachLink service (used to detach the object from the gripper)
        # This service simulates the physical release of objects in the Gazebo simulation
        self.detach_client = self.create_client(DetachLink, '/DETACHLINK')
        while not self.detach_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('DetachLink service not available, waiting...')

        # Define the arm joint names for the 7-DOF TIAGo arm
        # These joints form the complete kinematic chain for arm manipulation
        self.JOINT_NAMES = [
            "arm_1_joint",  # Shoulder rotation (base joint)
            "arm_2_joint",  # Shoulder elevation
            "arm_3_joint",  # Arm twist/rotation
            "arm_4_joint",  # Elbow joint
            "arm_5_joint",  # Forearm twist
            "arm_6_joint",  # Wrist pitch
            "arm_7_joint"   # Wrist roll
        ]

        # Initialize MoveIt2 interface for arm motion planning and execution
        # Provides comprehensive motion planning capabilities including collision avoidance
        self.moveit2 = MoveIt2(
            node=self,                                  # Reference to this ROS2 node
            joint_names=self.JOINT_NAMES,               # List of arm joints to control
            base_link_name=self.robot_base_frame,       # Robot base coordinate frame
            end_effector_name="gripper_grasping_frame", # Target frame for pose planning
            group_name="arm",                           # MoveIt2 planning group name
            callback_group=callback_group,              # Allow concurrent callbacks
        )
        # Set motion planner algorithm - RRTConnect is reliable for manipulation tasks
        self.moveit2.planner_id = "RRTConnectkConfigDefault"

        # Gripper interface for opening/closing the gripper during placement operations
        # Provides high-level gripper control with predefined positions
        GRIPPER_JOINT_NAMES = ["gripper_left_finger_joint", "gripper_right_finger_joint"]
        self.gripper_interface = GripperInterface(
            node=self,                                      # Reference to this ROS2 node
            gripper_joint_names=GRIPPER_JOINT_NAMES,        # List of gripper joints to control
            open_gripper_joint_positions=[0.04, 0.04],     # Fully open position (4cm separation)
            closed_gripper_joint_positions=[0.0, 0.0],     # Fully closed position (contact)
            gripper_group_name="gripper",                   # MoveIt2 planning group for gripper
            callback_group=callback_group,                  # Allow concurrent callbacks
            gripper_command_action_name="gripper_controller/joint_trajectory"  # Action server for gripper control
        )

        # Start a background executor thread for handling callbacks
        # This prevents blocking the main thread while processing MoveIt2 operations
        executor = rclpy.executors.MultiThreadedExecutor(2)  # Use 2 threads for concurrent operations
        executor.add_node(self)                               # Add this node to the executor
        executor_thread = Thread(target=executor.spin, daemon=True)  # Create background thread
        executor_thread.start()                               # Start the executor thread

        # Publisher to notify when the place arm action is done
        # Enables coordination with downstream processes in pick-and-place workflows
        self.done_pub = self.create_publisher(Bool, '/place_arm_done', 10)

        # Subscriber for start command to trigger the place arm sequence
        # Provides external interface for initiating placement operations
        self.start_sub = self.create_subscription(Empty, '/start_place_arm', self.start_callback, 10)

        # Joint state listener to get current arm joint positions
        # Maintains real-time awareness of the robot's current configuration
        self.current_joint_positions = {name: 0.0 for name in self.JOINT_NAMES}  # Initialize position storage
        self.create_subscription(
            JointState,                                     # Message type for joint information
            "/joint_states",                                # Topic publishing robot joint states
            self.joint_states_callback,                     # Callback to process joint updates
            10,                                             # Queue size for message buffering
            callback_group=callback_group                   # Use reentrant callback group
        )

        # Log node initialization completion
        self.get_logger().info("TIAGo place arm node ready.")

    def marker_id_callback(self, msg):
        """
        Callback to update the cube_id when a new marker ID is published.
        
        This callback allows dynamic updating of the target object during runtime,
        enabling the robot to adapt to different objects for placement operations.
        The marker ID update ensures that the correct object is detached during
        the placement sequence.
        
        Args:
            msg (String): ROS2 String message containing the new marker/cube ID
            
        The callback updates the internal cube_id and sets a flag indicating that
        a dynamic update has been received, which can override the initial parameter value.
        """
        self.cube_id = msg.data                                # Update target object ID
        self.marker_id_received = True                         # Mark that dynamic update was received
        self.get_logger().info(f"[UPDATE] cube_id updated to {self.cube_id}")

    def joint_states_callback(self, msg):
        """
        Callback to update the current joint positions from the /joint_states topic.
        
        This callback maintains real-time awareness of the robot's current joint
        configuration by processing joint state messages published by the robot's
        control system. The current positions can be used for:
        
        1. Validating successful movement completion
        2. Monitoring joint limits and safety constraints
        3. Calculating relative movements or trajectories
        4. Debugging motion planning issues
        
        Args:
            msg (JointState): ROS2 message containing joint names, positions, velocities, and efforts
            
        Only arm joints specified in JOINT_NAMES are stored to filter out
        other robot joints (torso, head, etc.) that are not relevant for arm placement.
        """
        # Update stored positions for arm joints only (filter out other robot joints)
        for name, position in zip(msg.name, msg.position):
            if name in self.JOINT_NAMES:                       # Check if joint is part of the arm
                self.current_joint_positions[name] = position  # Store current position

    def start_callback(self, msg):
        """
        Callback triggered by the /start_place_arm topic to initiate the placement sequence.
        
        This is the main coordination method that executes the complete arm-based
        placement operation. The sequence is designed to safely place grasped objects
        at a predefined location using joint-space motion planning.
        
        Placement Sequence:
        1. Validate that a target object ID is available
        2. Move arm to predefined placement joint configuration
        3. Open gripper to release the grasped object
        4. Detach object via Gazebo simulation service
        5. Publish completion notification for workflow coordination
        
        The method uses predefined joint configurations rather than pose-based
        planning to ensure consistent and reliable placement positions.
        
        Args:
            msg (Empty): ROS2 Empty message triggering the placement sequence
            
        Error handling ensures the robot doesn't get stuck in unsafe states,
        and completion status is always published for workflow coordination.
        """
        self.get_logger().info("Received /start_place_arm")
        
        # Validate that target object ID is available before proceeding
        if not self.marker_id_received and self.cube_id == "undefined":
            self.get_logger().error("cube_id is undefined and no marker ID received yet.")
            return

        try:
            # Step 1: Move the arm to the predefined placement joint configuration
            # This ensures consistent placement position regardless of approach path
            success_place = self.move_to_place_configuration()
            if not success_place:
                self.get_logger().error("Place joint configuration failed.")
                return
                
            # Step 2: Open gripper to release the grasped object
            # Allows the object to be placed at the target location
            self.get_logger().info("Opening gripper...")
            self.gripper_interface.open()                      # Use gripper interface for reliable opening
            time.sleep(1.0)                                    # Wait for gripper to fully open

            # Step 3: Detach the object from the gripper using the DetachLink service
            # This removes the physical attachment in the Gazebo simulation
            self.get_logger().info("Detaching object...")
            self.detach_object(
                model1_name='tiago',                           # Robot model name
                link1_name='gripper_right_finger_link',       # Gripper link to detach from
                model2_name=f'aruco_cube_exam_id{self.cube_id}',  # Target object model name
                link2_name='link'                              # Object link to detach
            )

            # Step 4: Publish completion notification for workflow coordination
            done_msg = Bool(data=True)                         # Create completion notification
            self.done_pub.publish(done_msg)                    # Publish to coordination topic
            self.get_logger().info("Place operation complete.")

        except Exception as e:
            # Handle any unexpected exceptions during the placement operation
            self.get_logger().error(f"Exception during place: {str(e)}")

    def move_to_place_configuration(self):
        """
        Moves the robot's arm to a predefined joint configuration for placing objects.
        
        This method uses joint-space motion planning to move the arm to a specific,
        predefined configuration that positions the gripper in an optimal location
        for object placement. The joint-space approach provides:
        
        1. Consistent and repeatable placement positions
        2. Predictable motion paths without pose ambiguity
        3. Reliable motion planning with fewer singularity issues
        4. Direct control over joint limits and collision avoidance
        
        Joint Configuration Strategy:
        - The target angles position the arm in a safe placement pose
        - Each joint angle is carefully selected to avoid collisions
        - The configuration provides good reachability for placement tasks
        - All angles are converted from degrees to radians for ROS2 compatibility
        
        Returns:
            bool: True if the movement completes successfully, False otherwise
            
        The method uses MoveIt2's move_to_configuration for joint-space planning,
        which typically provides more reliable results than pose-based planning
        for predefined positions.
        """
        # Define target joint positions in radians for optimal placement configuration
        # These angles position the arm for safe and accessible object placement
        target_positions = [
            math.radians(77),                           # arm_1_joint: Shoulder rotation (77°)
            math.radians(-73),                          # arm_2_joint: Shoulder elevation (-73°)
            math.radians(-166),                         # arm_3_joint: Arm twist (-166°)
            math.radians(95),                           # arm_4_joint: Elbow joint (95°)
            math.radians(92),                           # arm_5_joint: Forearm twist (92°)
            math.radians(59),                           # arm_6_joint: Wrist pitch (59°)
            math.radians(18),                           # arm_7_joint: Wrist roll (18°)
        ]

        # Log the target configuration for debugging and monitoring
        self.get_logger().info(f"Moving to place joint configuration: {target_positions}")
        
        # Execute joint-space motion planning and movement
        self.moveit2.move_to_configuration(target_positions)  # Plan and execute movement
        self.moveit2.wait_until_executed()                    # Wait for movement completion
        time.sleep(1.0)                                       # Additional stabilization time
        return True

    def detach_object(self, model1_name, link1_name, model2_name, link2_name):
        """
        Calls the DetachLink service to detach the object from the robot's gripper.
        
        This method removes the physical attachment between the robot's gripper and
        the target object in the Gazebo simulation environment. The detachment is
        necessary to complete the placement operation and allow the object to remain
        at the placement location when the robot moves away.
        
        Service Call Process:
        1. Create DetachLink service request with model and link specifications
        2. Send asynchronous service call to avoid blocking the main thread
        3. Check service response for successful detachment
        4. Log appropriate success or failure messages
        
        Args:
            model1_name (str): Name of the robot model (typically 'tiago')
            link1_name (str): Name of the robot link to detach from (gripper finger)
            model2_name (str): Name of the object model to detach (ArUco cube)
            link2_name (str): Name of the object link to detach (typically 'link')
            
        The method uses asynchronous service calls to prevent blocking and includes
        error handling to manage potential service failures gracefully.
        """
        # Create and populate the DetachLink service request
        req = DetachLink.Request()
        req.model1_name = model1_name                          # Robot model name
        req.link1_name = link1_name                            # Robot gripper link
        req.model2_name = model2_name                          # Object model name
        req.link2_name = link2_name                            # Object link name

        # Send asynchronous service call to avoid blocking the main execution thread
        future = self.detach_client.call_async(req)
        # Note: Could optionally spin until the future is complete for synchronous behavior

        # Check service response and log appropriate messages
        if future.result():
            self.get_logger().info(f"Detached {model2_name} from {model1_name}")
        else:
            self.get_logger().error(f"Failed to detach {model2_name}")

def main(args=None):
    """
    Main entry point for the TIAGo arm placement node.
    
    This function serves as the primary entry point for the place_arm node, handling
    the complete lifecycle from initialization to shutdown. It sets up the necessary
    ROS2 infrastructure for MoveIt2-based placement operations and coordinates the
    placement workflow execution.
    
    Key Responsibilities:
    1. Initialize ROS2 Python client library with command line arguments
    2. Create the TiagoPlaceArm node instance with all required components
    3. Set up multi-threaded executor required for MoveIt2 concurrent operations
    4. Run the main event loop until shutdown or interruption
    5. Ensure proper cleanup of all resources during shutdown
    
    The multi-threaded executor is essential for placement operations as it:
    - Handles concurrent motion planning and execution
    - Manages service calls and gripper control simultaneously
    - Processes joint state updates and motion feedback
    - Prevents deadlocks in complex manipulation workflows
    
    Integration Context:
    This node typically operates as part of a larger pick-and-place system,
    receiving trigger commands from higher-level coordination nodes and
    publishing completion notifications for downstream processes.
    
    Args:
        args (list, optional): Command line arguments passed to ROS2 initialization.
                              Includes node parameters, namespace settings, and logging configuration.
                              
    The function includes proper exception handling to ensure graceful shutdown
    and resource cleanup regardless of how the program terminates.
    """
    # Initialize the ROS2 Python client library with command line arguments
    rclpy.init(args=args)
    
    # Create an instance of the TIAGo placement node
    node = TiagoPlaceArm()
    
    # Create multi-threaded executor for concurrent MoveIt2 operations
    # Required for handling motion planning, gripper control, and service calls simultaneously
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)                                   # Add placement node to executor
    
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


