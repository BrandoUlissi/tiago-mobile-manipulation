# Import necessary modules for ROS2 node development and robot control
from threading import Thread                                    # For running background executor threads
import rclpy                                                   # ROS2 Python client library
from rclpy.callback_groups import ReentrantCallbackGroup       # Allow concurrent callback execution
from rclpy.node import Node                                    # Base class for ROS2 nodes
from tf2_ros.buffer import Buffer                              # TF2 transformation buffer for coordinate frames
from tf2_ros.transform_listener import TransformListener       # Listen to TF2 transformation broadcasts
from linkattacher_msgs.srv import AttachLink                   # Service for attaching objects in Gazebo simulation
from std_msgs.msg import Bool, Empty, String                   # Standard ROS2 message types
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint  # Joint trajectory control messages
from rclpy.duration import Duration                            # ROS2 time duration utilities
import numpy as np                                             # Numerical operations for position calculations
import time                                                    # Python time utilities for delays

class TiagoArucoTorso(Node):
    """
    ROS2 Node for controlling TIAGo robot's torso and gripper for object grasping.
    
    This node handles torso-based grasping operations by:
    1. Moving the torso down to reach objects at lower heights
    2. Using direct joint trajectory control for gripper and torso
    3. Attaching objects via Gazebo simulation services
    4. Coordinating the complete pick sequence without MoveIt2
    
    The node subscribes to marker ID updates and trigger topics,
    then executes a predefined sequence: open gripper -> lower torso -> 
    attach object -> close gripper -> raise torso.
    """
    def __init__(self):
        # Initialize the ROS2 node with a descriptive name
        super().__init__('tiago_aruco_torso')

        # Declare and get the cube_id parameter (used for identifying the object to grasp)
        # This parameter can be set at launch time to specify which ArUco cube to target
        self.declare_parameter("cube_id", "undefined")
        cube_id_param = self.get_parameter("cube_id").get_parameter_value().string_value
        self.cube_id = cube_id_param                           # Store the target cube identifier
        self.marker_id_received = False                        # Flag to track if marker ID was updated dynamically

        # Subscribe to the current marker ID topic to update the cube_id dynamically
        # This allows the system to retarget different objects during runtime
        self.create_subscription(String, '/current_marker_id', self.marker_id_callback, 10)

        # TF2 buffer and listener for frame transformations (not used for movement, but kept for marker ID logic)
        # These components handle coordinate transformations between different robot frames
        self.tf_buffer = Buffer()                              # Buffer to store transformation history
        self.tf_listener = TransformListener(self.tf_buffer, self)  # Listener for TF2 broadcasts
        self.robot_base_frame = "base_link"                    # Reference frame for robot base

        # Use a reentrant callback group for concurrent service calls
        # This allows multiple callbacks to execute simultaneously without blocking
        callback_group = ReentrantCallbackGroup()

        # Create a client for the AttachLink service (used to attach the object to the gripper)
        # This service simulates physical attachment of objects in the Gazebo simulation environment
        self.attach_client = self.create_client(AttachLink, '/ATTACHLINK')
        while not self.attach_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('AttachLink service not available, waiting...')

        # Publisher for torso trajectory commands (direct control, not via MoveIt2)
        # Sends joint trajectory commands directly to the torso controller for vertical movement
        self.torso_publisher = self.create_publisher(
            JointTrajectory,                                   # Message type for joint trajectories
            '/torso_controller/joint_trajectory',              # Topic for torso controller
            10                                                 # Queue size for message buffering
        )

        # Publisher for gripper trajectory commands (direct control, not via MoveIt2)
        # Controls gripper opening and closing through direct joint trajectory commands
        self.gripper_publisher = self.create_publisher(
            JointTrajectory,                                   # Message type for joint trajectories
            '/gripper_controller/joint_trajectory',            # Topic for gripper controller
            10                                                 # Queue size for message buffering
        )

        # Start a background executor thread for handling callbacks
        # This prevents blocking the main thread while processing concurrent operations
        executor = rclpy.executors.MultiThreadedExecutor(2)   # Use 2 threads for concurrent operations
        executor.add_node(self)                               # Add this node to the executor
        executor_thread = Thread(target=executor.spin, daemon=True)  # Create background thread
        executor_thread.start()                               # Start the executor thread

        # Publisher to notify when the pick torso action is done
        # Other nodes can subscribe to this topic to know when the grasping sequence completes
        self.done_pub = self.create_publisher(Bool, '/pick_torso_done', 10)

        # Subscriber to trigger the pick torso sequence
        # Listens for Empty messages on this topic to start the grasping operation
        self.start_sub = self.create_subscription(Empty, '/start_pick_torso', self.start_callback, 10)

        # Log node initialization completion
        self.get_logger().info("TIAGo pick torso node ready.")

    def marker_id_callback(self, msg):
        """
        Callback to update the cube_id when a new marker ID is published.
        
        This allows dynamic retargeting of different ArUco cubes during operation.
        The marker ID is received as a String message and updates the target object
        for subsequent grasping operations.
        
        Args:
            msg (String): ROS2 String message containing the new marker/cube ID
        """
        self.cube_id = msg.data                                # Update target cube identifier
        self.marker_id_received = True                         # Mark that we received a dynamic update
        self.get_logger().info(f"[UPDATE] cube_id updated to {self.cube_id}")

    def attach_object(self, model1_name, link1_name, model2_name, link2_name):
        """
        Calls the AttachLink service to attach the detected object to the robot's gripper.
        
        This service creates a physical connection between the robot's gripper and the target
        object in the Gazebo simulation environment. The attachment ensures that the object
        moves with the robot during subsequent motions.
        
        Args:
            model1_name (str): Name of the robot model (typically 'tiago')
            link1_name (str): Name of the robot link to attach to (gripper finger)
            model2_name (str): Name of the object model to attach (ArUco cube)
            link2_name (str): Name of the object link to attach (typically 'link')
            
        Returns:
            bool: True if the service call succeeds, False otherwise
        """
        self.get_logger().info("Calling attach service...")
        
        # Create and populate the service request
        req = AttachLink.Request()
        req.model1_name = model1_name                          # Robot model name
        req.link1_name = link1_name                            # Robot gripper link
        req.model2_name = model2_name                          # Object model name
        req.link2_name = link2_name                            # Object link name

        # Send asynchronous service call to avoid blocking
        future = self.attach_client.call_async(req)
        self.get_logger().info("Service call sent, waiting for result...")
        
        # Wait for the service to complete and check the result
        if future.done():
            try:
                result = future.result()                       # Get service response
                if result is not None:
                    self.get_logger().info(f"Attached {model2_name} to {model1_name}")
                    return True
                else:
                    self.get_logger().error(f"Failed to attach {model2_name}: No result")
                    return False
            except Exception as e:
                self.get_logger().error(f"Failed to attach {model2_name}: {str(e)}")
                return False
        else:
            self.get_logger().error(f"Attach service call timed out for {model2_name}")
            return False

    def move_torso_down(self):
        """
        Move the robot's torso down to a lower position (0.08m) using a JointTrajectory message.
        
        This method lowers the torso to enable grasping of objects placed on lower surfaces
        or tables. The movement is executed through direct joint trajectory control,
        bypassing MoveIt2 for simpler and more predictable motion.
        
        The torso_lift_joint controls the vertical position of the robot's torso,
        with lower values corresponding to lower positions.
        
        Returns:
            bool: True if the torso movement completes successfully, False otherwise
        """
        try:
            self.get_logger().info("Moving torso down to 0.08m for grasping...")
            time.sleep(1.0)                                    # Brief pause before movement

            # Create and populate the trajectory message for torso control
            trajectory_msg = JointTrajectory()
            trajectory_msg.joint_names = ['torso_lift_joint']  # Specify the torso lift joint

            # Define the target position and timing for the movement
            point = JointTrajectoryPoint()
            point.positions = [0.08]                           # Target position: 0.08m (low position)
            point.time_from_start.sec = 3                      # Duration: 3 seconds for smooth movement

            trajectory_msg.points.append(point)               # Add the waypoint to trajectory
            
            # Publish the trajectory to the torso controller
            self.torso_publisher.publish(trajectory_msg)
            self.get_logger().info("Torso trajectory published, waiting for movement to complete...")
            
            # Wait for the movement to finish (longer than trajectory time for safety)
            time.sleep(4.0)                                    # Wait for completion plus safety margin
            self.get_logger().info("Torso movement completed")
            return True
            
        except Exception as e:
            self.get_logger().error(f"Failed to move torso: {str(e)}")
            return False
        
    def move_torso_up(self):
        """
        Move the robot's torso back up to a higher position (0.25m) using a JointTrajectory message.
        
        This method raises the torso after successful object grasping to return to a safe
        transport position. The higher position prevents the grasped object from colliding
        with obstacles during navigation and provides better clearance.
        
        The movement uses the same joint trajectory approach as move_torso_down() but
        targets a higher position value for the torso_lift_joint.
        
        Returns:
            bool: True if the torso movement completes successfully, False otherwise
        """
        try:
            self.get_logger().info("Moving torso back up to 0.2m...")
            
            # Create and populate the trajectory message for upward movement
            trajectory_msg = JointTrajectory()
            trajectory_msg.joint_names = ['torso_lift_joint']  # Specify the torso lift joint

            # Define the target position and timing for the upward movement
            point = JointTrajectoryPoint()
            point.positions = [0.25]                           # Target position: 0.25m (elevated position)
            point.time_from_start.sec = 3                      # Duration: 3 seconds for smooth movement

            trajectory_msg.points.append(point)               # Add the waypoint to trajectory
            
            # Publish the trajectory to the torso controller
            self.torso_publisher.publish(trajectory_msg)
            self.get_logger().info("Torso trajectory published, waiting for movement to complete...")
            
            # Wait for the movement to finish (longer than trajectory time for safety)
            time.sleep(4.0)                                    # Wait for completion plus safety margin
            self.get_logger().info("Torso movement completed")
            return True
            
        except Exception as e:
            self.get_logger().error(f"Failed to move torso: {str(e)}")
            return False

    def open_gripper(self):
        """
        Open the robot's gripper using a JointTrajectory message.
        
        This method opens the two-finger parallel gripper by sending joint trajectory
        commands directly to the gripper controller. The gripper opens to a position
        of 0.04m (4cm) for each finger, providing sufficient clearance to approach
        and position around target objects.
        
        The direct trajectory control bypasses MoveIt2 for simpler and more
        predictable gripper operation during the torso-based grasping sequence.
        
        Returns:
            bool: True if the gripper opens successfully, False otherwise
        """
        try:
            self.get_logger().info("Opening gripper with trajectory...")
            
            # Create trajectory message for gripper control
            trajectory_msg = JointTrajectory()
            trajectory_msg.joint_names = ['gripper_left_finger_joint', 'gripper_right_finger_joint']

            # Define target positions for both gripper fingers
            point = JointTrajectoryPoint()
            point.positions = [0.04, 0.04]                     # Open positions: 4cm separation for both fingers
            point.time_from_start.sec = 2                      # Duration: 2 seconds for smooth opening

            trajectory_msg.points.append(point)               # Add waypoint to trajectory
            self.gripper_publisher.publish(trajectory_msg)    # Send command to gripper controller
            
            self.get_logger().info("Gripper open command sent")
            time.sleep(3.0)                                    # Wait for gripper to fully open
            self.get_logger().info("Gripper opened")
            return True
            
        except Exception as e:
            self.get_logger().error(f"Failed to open gripper: {str(e)}")
            return False

    def close_gripper(self):
        """
        Close the robot's gripper using a JointTrajectory message.
        
        This method closes the two-finger parallel gripper to grasp the target object.
        The fingers move to a position of 0.028m (2.8cm) which provides sufficient
        grip force while avoiding over-compression of delicate objects.
        
        The closing motion is coordinated with the object attachment service to
        ensure proper grasping in the simulation environment. Both fingers move
        symmetrically to maintain balanced grip force.
        
        Returns:
            bool: True if the gripper closes successfully, False otherwise
        """
        try:
            self.get_logger().info("Closing gripper with trajectory...")
            
            # Create trajectory message for gripper closing
            trajectory_msg = JointTrajectory()
            trajectory_msg.joint_names = ['gripper_left_finger_joint', 'gripper_right_finger_joint']

            # Define target positions for gripper closing
            point = JointTrajectoryPoint()
            point.positions = [0.028, 0.028]                   # Close positions: 2.8cm for both fingers (grasping)
            point.time_from_start.sec = 2                      # Duration: 2 seconds for controlled closing

            trajectory_msg.points.append(point)               # Add waypoint to trajectory
            self.gripper_publisher.publish(trajectory_msg)    # Send command to gripper controller
            
            self.get_logger().info("Gripper close command sent")
            time.sleep(3.0)                                    # Wait for gripper to fully close
            self.get_logger().info("Gripper closed")
            return True
            
        except Exception as e:
            self.get_logger().error(f"Failed to close gripper: {str(e)}")
            return False

    def start_callback(self, msg):
        """
        Callback triggered by the /start_pick_torso topic.
        
        This is the main coordination method that executes the complete torso-based
        grasping sequence. The sequence is designed for objects that require lowering
        the torso to reach them, such as items on low tables or surfaces.
        
        Execution sequence:
        1. Open gripper to prepare for grasping
        2. Lower torso to reach the object level
        3. Attach object via Gazebo simulation service
        4. Close gripper to secure the object
        5. Raise torso to transport position
        6. Publish completion notification
        
        The method includes error handling and continues execution even if some
        steps fail, ensuring the robot doesn't get stuck in an unsafe state.
        
        Args:
            msg (Empty): Empty ROS2 message triggering the sequence
        """
        self.get_logger().info("Received /start_pick_torso")
        
        # Validate that we have a valid target object identifier
        if not self.marker_id_received and self.cube_id == "undefined":
            self.get_logger().error("cube_id is undefined and no marker ID received yet.")
            return

        try:
            # Step 0: Open gripper first using trajectory control
            # This prepares the gripper for approaching and grasping the object
            self.get_logger().info("Opening gripper...")
            success_open = self.open_gripper()
            if not success_open:
                self.get_logger().warn("Gripper open failed, but continuing...")
            
            # Step 1: Move torso down to grasp position
            # Lower the robot's torso to reach objects at table height or below
            success_torso = self.move_torso_down()
            if not success_torso:
                self.get_logger().error("Torso move failed.")
                return
            
            # Step 2: Attach the object to the gripper using Gazebo simulation service
            # This creates a physical connection between the gripper and target object
            self.get_logger().info("Attaching object...")
            attach_success = self.attach_object(
                model1_name='tiago',                           # Robot model name
                link1_name='gripper_right_finger_link',       # Gripper link for attachment
                model2_name=f'aruco_cube_exam_id{self.cube_id}',  # Target ArUco cube model
                link2_name='link'                              # Object link for attachment
            )
            
            if not attach_success:
                self.get_logger().warn("Attach failed, but continuing with gripper closure...")
            
            time.sleep(2.0)                                    # Wait for attachment to stabilize

            # Step 3: Close gripper using trajectory control
            # Secure the attached object with gripper fingers
            self.get_logger().info("Closing gripper...")
            success_close = self.close_gripper()
            if not success_close:
                self.get_logger().warn("Gripper close failed, but continuing...")

            # Step 4: Move torso back up to original/safe position
            # Raise the torso for safe transport of the grasped object
            self.move_torso_up()
            time.sleep(1.0)                                    # Wait for torso movement completion
            
            # Step 5: Notify other nodes that the pick torso operation is complete
            self.get_logger().info("Pick torso operation complete.")
            done_msg = Bool(data=True)                         # Create completion notification
            self.done_pub.publish(done_msg)                    # Publish to coordination topic
            self.get_logger().info("Published /pick_torso_done = True")
            
        except Exception as e:
            self.get_logger().error(f"Exception during pick torso: {str(e)}")

def pose_distance(pos1, pos2):
    """
    Utility function to compute Euclidean distance between two 3D positions.
    
    This helper function calculates the straight-line distance between two points
    in 3D space using the Euclidean distance formula. While not currently used
    in the main grasping logic, it's available for distance-based validation
    or proximity checking if needed.
    
    Args:
        pos1 (array-like): First position as [x, y, z] coordinates
        pos2 (array-like): Second position as [x, y, z] coordinates
        
    Returns:
        float: Euclidean distance between the two positions
    """
    return np.linalg.norm(np.array(pos1) - np.array(pos2))

def main(args=None):
    """
    Main entry point for the TIAGo torso-based grasping node.
    
    This function initializes the ROS2 system, creates the TiagoArucoTorso node,
    and starts the ROS2 event loop. The node will continue running until
    interrupted by the user or system shutdown.
    
    The torso-based grasping approach is designed for scenarios where:
    - Objects are positioned at heights requiring torso adjustment
    - Direct joint control is preferred over MoveIt2 planning
    - Simple, predictable motion sequences are sufficient
    
    Args:
        args: Command line arguments passed to ROS2 initialization
    """
    rclpy.init(args=args)                                      # Initialize ROS2 system
    node = TiagoArucoTorso()                                   # Create the torso grasping node
    
    try:
        rclpy.spin(node)                                       # Start the ROS2 event loop
    finally:
        node.destroy_node()                                    # Clean up node resources
        rclpy.shutdown()                                       # Shutdown ROS2 system

if __name__ == "__main__":
    main()
