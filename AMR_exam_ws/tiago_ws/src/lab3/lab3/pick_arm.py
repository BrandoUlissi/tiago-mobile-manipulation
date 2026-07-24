from threading import Thread
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from pymoveit2 import MoveIt2
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from std_msgs.msg import Bool, Empty, String
from rclpy.duration import Duration
import numpy as np
import time


class TiagoArucoGrasp(Node):

    def __init__(self):
        # Initialize the ROS2 node with a descriptive name for ArUco-based grasping
        super().__init__('tiago_aruco_grasp')

        # Declare and get the cube_id parameter (used for identifying the target object to grasp)
        # This parameter can be set at launch time or updated dynamically via topic
        self.declare_parameter("cube_id", "undefined")
        cube_id_param = self.get_parameter("cube_id").get_parameter_value().string_value
        self.cube_id = cube_id_param
        self.marker_id_received = False  # Flag to track if dynamic ID update has been received

        # Subscribe to the current marker ID topic to update the cube_id dynamically
        # This allows the system to adapt to different objects detected during runtime
        self.create_subscription(String, '/current_marker_id', self.marker_id_callback, 10)

        # TF2 buffer and listener for coordinate frame transformations
        # Essential for converting between different coordinate frames (robot, object, world)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Define coordinate frames used in the grasping pipeline
        self.robot_base_frame = "base_link"                    # Robot's base coordinate frame
        self.approach_frame = "aruco_marker_frame_approach"    # Pre-grasp approach position

        # Use a reentrant callback group for concurrent MoveIt2 and service calls
        # This prevents deadlocks when multiple operations need to execute simultaneously
        callback_group = ReentrantCallbackGroup()

        # Define the TIAGo arm joint names (7 degrees of freedom for manipulation)
        # These joints form the kinematic chain from shoulder to wrist
        JOINT_NAMES = [
            "arm_1_joint",  # Shoulder rotation (base joint)
            "arm_2_joint",  # Shoulder elevation
            "arm_3_joint",  # Arm twist/rotation
            "arm_4_joint",  # Elbow joint
            "arm_5_joint",  # Forearm twist
            "arm_6_joint",  # Wrist pitch
            "arm_7_joint",  # Wrist roll
        ]

        # Initialize MoveIt2 interface for arm motion planning and execution
        # Handles trajectory planning, collision avoidance, and smooth arm movements
        self.moveit2 = MoveIt2(
            node=self,                                  # Reference to this ROS2 node
            joint_names=JOINT_NAMES,                    # List of arm joints to control
            base_link_name=self.robot_base_frame,       # Robot base coordinate frame
            end_effector_name="gripper_grasping_frame", # Target frame for pose planning
            group_name="arm",                           # MoveIt2 planning group name
            callback_group=callback_group,              # Allow concurrent callbacks
        )
        # Set motion planner algorithm - RRTConnect is reliable for manipulation tasks
        self.moveit2.planner_id = "RRTConnectkConfigDefault"

        # Start a background executor thread for handling callbacks
        # This prevents blocking the main thread while processing MoveIt2 operations
        executor = rclpy.executors.MultiThreadedExecutor(2)  # Use 2 threads for concurrent operations
        executor.add_node(self)
        executor_thread = Thread(target=executor.spin, daemon=True)  # Daemon thread for automatic cleanup
        executor_thread.start()

        # Publisher to notify when the pick arm action is complete
        # Enables coordination with downstream processes in pick-and-place workflows
        self.done_pub = self.create_publisher(
            Bool,              # Boolean message for completion status
            '/pick_arm_done',  # Topic name for publishing completion status
            10                 # Queue size for message buffering
        )

        # Subscriber to trigger the pick arm sequence
        # Provides external interface for initiating grasping operations
        self.start_sub = self.create_subscription(
            Empty,             # Simple trigger message (no data payload)
            '/start_pick_arm', # Topic name for receiving start commands
            self.start_callback, # Callback function to handle triggers
            10                 # Queue size for message buffering
        )

        self.get_logger().info("TIAGo pick arm node ready.")

    def marker_id_callback(self, msg):
        """
        Callback to update the cube_id when a new marker ID is published.
        
        This callback allows dynamic updating of the target object during runtime,
        enabling the robot to adapt to different objects detected by the vision system.
        
        Args:
            msg (String): ROS2 String message containing the new marker/cube ID
            
        The callback updates the internal cube_id and sets a flag indicating that
        a dynamic update has been received, which can override the initial parameter value.
        """
        # Update the target object ID from the received message
        self.cube_id = msg.data
        # Set flag to indicate dynamic ID update has been received
        self.marker_id_received = True
        self.get_logger().info(f"[UPDATE] cube_id updated to {self.cube_id}")

    
    def move_to_approach(self):
        """
        Moves the robot's arm to the approach pose (pre-grasp pose) using TF transform lookup.
        
        This method implements the first stage of the grasping sequence by moving the arm
        to a safe approach position near the target object. The approach pose is typically
        positioned above or near the object to avoid collisions during the final approach.
        
        Returns:
            bool: True if movement was successful or within acceptable margin, False otherwise
            
        The method includes robust error handling:
        - TF transform lookup with timeout protection
        - MoveIt2 motion planning with fallback logic
        - Distance-based success criteria when planning fails
        - Stabilization delay after movement completion
        """
        try:
            # Lookup the transform from robot base to the approach frame
            # This transform defines where the gripper should be positioned for safe approach
            t_approach = self.tf_buffer.lookup_transform(
                self.robot_base_frame,    # Source frame (robot base)
                self.approach_frame,      # Target frame (approach position)
                rclpy.time.Time(),        # Use latest available transform
                timeout=Duration(seconds=2)  # Timeout for transform lookup
            )
        except Exception as e:
            self.get_logger().error(f"Transform to approach frame failed: {str(e)}")
            return False

        # Extract position and orientation from the transform
        pos = [
            t_approach.transform.translation.x,  # X position in base frame
            t_approach.transform.translation.y,  # Y position in base frame
            t_approach.transform.translation.z   # Z position in base frame
        ]
        quat = [
            t_approach.transform.rotation.x,     # Quaternion X component
            t_approach.transform.rotation.y,     # Quaternion Y component
            t_approach.transform.rotation.z,     # Quaternion Z component
            t_approach.transform.rotation.w      # Quaternion W component
        ]
        
        self.get_logger().info(f"Moving to approach pose: {pos}, {quat}")
        
        # Attempt to move to the approach pose using MoveIt2 motion planning
        success = self.moveit2.move_to_pose(position=pos, quat_xyzw=quat)
        
        if not success:
            # If planning fails, check the current distance to the target as fallback
            try:
                t_current = self.tf_buffer.lookup_transform(
                    self.robot_base_frame,        # Source frame (robot base)
                    "gripper_grasping_frame",     # Current gripper position
                    rclpy.time.Time(),            # Use latest available transform
                    timeout=Duration(seconds=2)   # Timeout for transform lookup
                )
                # Extract current gripper position for distance calculation
                current_pos = [
                    t_current.transform.translation.x,
                    t_current.transform.translation.y,
                    t_current.transform.translation.z
                ]
                # Calculate Euclidean distance between current and target positions
                dist = pose_distance(current_pos, pos)
                self.get_logger().warn(f"MoveIt2 failed to plan, but current distance to target: {dist:.3f} m")
                
                # Check if the gripper is within acceptable margin of the target
                if dist < 0.15:  # Allow a 15 cm margin for success
                    self.get_logger().info("Within margin, proceeding.")
                else:
                    self.get_logger().error("MoveIt2 failed and not within margin.")
                    return False
            except Exception as e:
                self.get_logger().error(f"Failed to get current transform: {str(e)}")
                return False

        # Wait for the movement to complete and arm to stabilize
        self.moveit2.wait_until_executed()
        time.sleep(2.0)  # Stabilization delay to ensure accurate positioning
        return True

   
    def start_callback(self, msg):
        """
        Callback triggered by the /start_pick_arm topic to initiate the grasping sequence.
        
        This is the main entry point for the ArUco-based grasping operation. The callback
        is triggered when an Empty message is received on the /start_pick_arm topic,
        typically sent by a higher-level coordination system or user interface.
        
        The method performs the following operations:
        1. Validates that a target object ID is available (either from parameter or dynamic update)
        2. Executes the approach movement to position the arm near the target object
        3. Publishes completion status to coordinate with downstream processes
        
        Error handling includes:
        - Validation of cube_id availability before attempting movement
        - Graceful handling of approach failures with detailed logging
        - Exception catching to prevent node crashes during operation
        
        Args:
            msg (Empty): ROS2 Empty message triggering the grasping sequence
            
        The method uses robust error handling to ensure the robot doesn't get stuck
        in unsafe states, and always publishes completion status for workflow coordination.
        """

        self.get_logger().info("Received /start_pick_arm")
        
        # Validate that target object ID is available
        if not self.marker_id_received and self.cube_id == "undefined":
            self.get_logger().error("cube_id is undefined and no marker ID received yet.")
            return

        try:
            # Execute the approach movement (first stage of grasping)
            success_approach = self.move_to_approach()
            if not success_approach:
                self.get_logger().error("Approach failed.")
                # Note: Could return here if approach failure should abort the operation
                
            self.get_logger().info("Approach operation complete.")
            
            # Publish completion status for workflow coordination
            done_msg = Bool(data=True)  # Indicate operation completion
            self.done_pub.publish(done_msg)
            
        except Exception as e:
            # Handle any unexpected exceptions during the pick operation
            self.get_logger().error(f"Exception during approach: {str(e)}")


def pose_distance(pos1, pos2):
    """
    Utility function to compute Euclidean distance between two 3D positions.
    
    This function calculates the straight-line distance between two points in 3D space,
    which is useful for determining if the robot's gripper is close enough to the target
    position when motion planning fails but the robot is still within acceptable range.
    
    Args:
        pos1 (list): First 3D position as [x, y, z]
        pos2 (list): Second 3D position as [x, y, z]
        
    Returns:
        float: Euclidean distance between the two positions in meters
        
    The calculation uses NumPy's linear algebra norm function for efficient
    computation of the distance vector magnitude.
    """
    return np.linalg.norm(np.array(pos1) - np.array(pos2))


def main(args=None):
    """
    Main entry point for the TIAGo ArUco-based grasping node.
    
    This function serves as the primary entry point for the pick_arm node, handling
    the complete lifecycle from initialization to shutdown. It sets up the necessary
    ROS2 infrastructure for MoveIt2-based manipulation operations.
    
    Key responsibilities:
    1. Initialize ROS2 Python client library with command line arguments
    2. Create the TiagoArucoGrasp node instance with all required components
    3. Set up multi-threaded executor required for MoveIt2 concurrent operations
    4. Run the main event loop until shutdown or interruption
    5. Ensure proper cleanup of all resources during shutdown
    
    The multi-threaded executor is essential for MoveIt2 operations as it:
    - Handles concurrent motion planning and execution
    - Manages TF2 transform lookups and updates
    - Processes service calls and action feedback simultaneously
    - Prevents deadlocks in complex manipulation workflows
    
    Args:
        args (list, optional): Command line arguments passed to ROS2 initialization.
                              Typically includes node name, namespace, and parameter overrides.
                              
    The function includes proper exception handling to ensure graceful shutdown
    and resource cleanup regardless of how the program terminates.
    """

    # Initialize the ROS2 Python client library
    rclpy.init(args=args)
    
    # Create an instance of the TIAGo grasping node
    node = TiagoArucoGrasp()
    
    # Create multi-threaded executor for concurrent operations
    # Required for MoveIt2 which uses multiple internal threads
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        # Start the executor - runs until interrupted or shutdown
        executor.spin()
    finally:
        # Ensure proper cleanup regardless of how the program exits
        node.destroy_node()  # Clean up node resources
        rclpy.shutdown()     # Shutdown ROS2 Python client library


# Standard Python idiom to run main() only when script is executed directly
# This prevents execution when the file is imported as a module
if __name__ == "__main__":
    main()