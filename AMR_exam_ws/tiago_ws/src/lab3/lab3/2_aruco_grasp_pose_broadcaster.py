# Import necessary modules for ROS2 ArUco-based grasping pose generation
import rclpy                                                   # ROS2 Python client library
from rclpy.node import Node                                    # Base class for ROS2 nodes
import numpy as np                                             # Numerical operations for transformations
from geometry_msgs.msg import TransformStamped, PoseStamped    # ROS2 geometry message types
from tf2_ros import TransformBroadcaster, Buffer, TransformListener  # TF2 coordinate frame management
from PyKDL import Frame, Vector, Rotation                      # KDL library for 3D transformations
from std_msgs.msg import Empty, Bool                           # Standard ROS2 message types

class ArucoGraspBroadcaster(Node):
    """
    ROS2 Node for detecting ArUco markers and generating grasping pose frames.
    
    This node processes ArUco marker detections from the robot's camera and generates
    appropriate coordinate frames for robotic grasping operations. It creates two
    key frames for each detected marker:
    
    1. Approach Frame: Positioned 20cm above the marker for safe pre-grasp positioning
    2. Target Frame: Positioned 5cm above the marker for actual grasping contact
    
    The node performs several critical functions:
    - Validates marker orientation (Z-axis pointing upward) before processing
    - Transforms poses between different coordinate frames (camera, base_link, map)
    - Publishes periodic TF broadcasts for real-time coordinate frame updates
    - Manages state transitions for coordinated grasping sequences
    - Provides completion notifications for workflow coordination
    
    Key Features:
    - Orientation filtering to ensure valid grasp poses
    - Multi-frame publishing (base_link and map coordinates)
    - State management for sequential grasping operations
    - Robust error handling for transform failures
    """
    def __init__(self):
        # Initialize the ROS2 node with a descriptive name for ArUco processing
        super().__init__('aruco_subscriber')
        
        # Subscribe to the topic to start the ArUco grasp sequence
        # This provides external control over when to begin marker detection and pose generation
        self.start_sub = self.create_subscription(Empty, '/start_aruco_grasp', self.start_cb, 10)
        
        # Publisher to notify when ArUco grasp frames are published
        # Enables coordination with downstream robotic manipulation processes
        self.done_pub = self.create_publisher(Bool, '/aruco_grasp_done', 10)
        
        # Subscribe to the ArUco marker transform topic
        # Receives real-time marker detections from the ArUco detection pipeline
        self.subscription = self.create_subscription(TransformStamped, '/aruco_single/transform', self.get_aruco_callback, 1)

        # TF broadcaster for publishing new coordinate frames
        # Publishes approach and target frames to the TF2 transformation tree
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Publisher for approach pose (not always used but available for debugging)
        # Can be used to visualize or debug the approach pose in external tools
        self.pose_publisher = self.create_publisher(PoseStamped, '/approach_pose', 1)

        # Timers for periodic frame publishing at 10Hz (0.1 second intervals)
        # Regular publishing ensures frames remain available in the TF2 tree
        self.timer_approach = self.create_timer(0.1, self.publish_frame_approach)   # Approach frame publisher
        self.timer_target = self.create_timer(0.1, self.publish_frame_target)       # Target frame publisher
        self.timer_aruco = self.create_timer(0.1, self.timer_tf_base)               # Transform updater

        # TF2 buffer and listener for coordinate frame transformations
        # Essential for converting between camera, robot, and world coordinate systems
        self.tf_buffer = Buffer()                              # Stores transformation history
        self.tf_listener = TransformListener(self.tf_buffer, self)  # Listens to TF2 broadcasts
        
        # Initialize transformation and frame storage variables
        self.t_base = None                                     # Transform from base_link to camera
        self.frame_aruco = None                                # Detected ArUco marker pose in camera frame
        self.frame_approach = None                             # Calculated approach pose
        self.frame_target = None                               # Calculated target pose
        self.frame_approach_in_map = None                      # Approach pose in map coordinates
        self.frame_target_in_map = None                        # Target pose in map coordinates
        self.t_map = None                                      # Transform from map to base_link

        # Frame names for robot, camera, and published coordinate frames
        # These define the coordinate system hierarchy used throughout the grasping pipeline
        self.robot_base_frame = "base_link"                            # Robot's base coordinate frame
        self.camera_frame = "head_front_camera_rgb_optical_frame"      # Camera's optical frame
        self.frame_target_name = "aruco_marker_frame_target"           # Target grasp frame name
        self.frame_approach_name = "aruco_marker_frame_approach"       # Approach frame name
        self.frame_map = "map"                                         # Global map coordinate frame
        self.frame_target_in_map_name = "aruco_marker_frame_target_in_map"     # Target frame in map coordinates
        self.frame_approach_in_map_name = "aruco_marker_frame_approach_in_map" # Approach frame in map coordinates

        # Control flags for managing the grasping sequence state machine
        # These flags coordinate the multi-step process of pose generation and publication
        self.should_start = False                              # Flag to enable/disable processing
        self.approach_published = False                        # Tracks if approach frame has been published
        self.target_published = False                          # Tracks if target frame has been published
        self.done_sent = False                                 # Prevents duplicate completion notifications

    def start_cb(self, msg):
        """
        Callback for /start_aruco_grasp topic - initiates a new grasping sequence.
        
        This callback is triggered when the system needs to detect and generate
        grasping poses for a new ArUco marker. It resets all state variables
        to ensure a clean start for the new detection cycle.
        
        The method performs the following state reset operations:
        1. Enables marker processing and pose generation
        2. Clears any previous marker detection data
        3. Resets publication tracking flags
        4. Prepares the system for fresh marker detection
        
        Args:
            msg (Empty): ROS2 Empty message triggering the sequence start
            
        This ensures that each grasping sequence starts with a clean state,
        preventing interference from previous detection cycles.
        """
        self.should_start = True                               # Enable marker processing
        self.frame_aruco = None                                # Reset pose to ensure fresh reading
        self.approach_published = False                        # Reset approach frame publication status
        self.target_published = False                          # Reset target frame publication status
        self.done_sent = False                                 # Reset completion notification status
        self.get_logger().info('Received /start_aruco_grasp, starting publishing...')

    def get_aruco_callback(self, msg):
        """
        Callback for /aruco_single/transform - processes detected ArUco marker poses.
        
        This callback receives ArUco marker detections from the vision system and
        validates them for robotic grasping. It implements orientation filtering
        to ensure that only properly oriented markers (with Z-axis pointing upward)
        are processed for grasping operations.
        
        The orientation check is critical because:
        1. It ensures the marker represents a graspable object orientation
        2. It filters out markers on walls or tilted surfaces
        3. It provides more reliable grasping poses for manipulation
        
        Validation Process:
        1. Extract position and orientation from the transform message
        2. Convert to PyKDL Frame for easier geometric operations
        3. Calculate the marker's Z-axis direction in the camera frame
        4. Check if Z-axis points upward (within angular threshold)
        5. Accept or reject the marker based on orientation criteria
        
        Args:
            msg (TransformStamped): ArUco marker transform from the detection system
            
        The method uses a threshold of 0.75 for the Z-component, corresponding
        to approximately 45 degrees maximum tilt from vertical.
        """
        # Only process markers when the system is actively looking for them
        if not self.should_start:
            return
            
        # Extract position and orientation from the transform message
        position = Vector(msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z)
        orientation = Rotation.Quaternion(
            msg.transform.rotation.x,               # Quaternion X component
            msg.transform.rotation.y,               # Quaternion Y component
            msg.transform.rotation.z,               # Quaternion Z component
            msg.transform.rotation.w                # Quaternion W component
        )
        # Create PyKDL Frame combining position and orientation
        frame = Frame(orientation, position)

        # Get the Z axis of the detected marker frame in the camera coordinate system
        # This tells us which direction is "up" for the detected marker
        z_axis = frame.M * Vector(0, 0, 1)         # Transform Z unit vector to camera frame

        # Check if Z axis is pointing upward (positive Z direction in camera/world frame)
        # The threshold 0.75 corresponds to approximately 45 degrees from vertical
        if abs(z_axis[2]) < 0.75:                  # Reject markers tilted more than ~45 degrees
            self.get_logger().info(f"Z axis component: {z_axis[2]}")
            self.get_logger().info("Aruco detected but Z axis is not pointing upward, ignoring.")
            return

        # Store the validated marker frame for pose generation
        self.frame_aruco = Frame(orientation, position)

    def get_frame_kdl(self, tf):
        """
        Converts a ROS TransformStamped message to a PyKDL Frame object.
        
        This utility function bridges between ROS2 geometry messages and PyKDL
        (Kinematics and Dynamics Library) representations. PyKDL provides more
        convenient methods for 3D transformations and geometric calculations
        compared to raw ROS2 messages.
        
        The conversion process:
        1. Extract position components (x, y, z) from the transform
        2. Extract quaternion components (x, y, z, w) from the transform
        3. Create PyKDL Rotation from quaternion representation
        4. Create PyKDL Vector from position components
        5. Combine into a PyKDL Frame for unified geometric operations
        
        Args:
            tf (TransformStamped): ROS2 transform message to convert
            
        Returns:
            Frame: PyKDL Frame object with equivalent pose information
            
        PyKDL Frames provide convenient methods for:
        - Frame composition and inversion
        - Rotation operations (DoRotX, DoRotY, DoRotZ)
        - Vector transformations
        - Geometric calculations
        """
        # Extract position components from ROS2 transform message
        pos = [tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z]
        # Extract quaternion components (x, y, z, w ordering for PyKDL)
        quat = [tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w]
        # Create PyKDL Frame with rotation from quaternion and translation from position
        return Frame(Rotation.Quaternion(*quat), Vector(*pos))

    def timer_tf_base(self):
        """
        Periodically updates coordinate frame transformations needed for pose calculations.
        
        This timer callback runs at 10Hz to maintain up-to-date transformations between
        the key coordinate frames in the robotic system. It handles two critical transforms:
        
        1. Base-to-Camera Transform: Converts marker poses from camera coordinates
           to robot base coordinates for motion planning and execution
        2. Map-to-Base Transform: Enables publishing poses in the global map frame
           for navigation and multi-robot coordination
        
        The method uses TF2 lookup with current timestamps to get the most recent
        available transformations. Error handling is minimal (commented out logging)
        to prevent spam during normal operation when transforms may be temporarily
        unavailable.
        
        Transform Chain:
        Camera Frame → Base Frame → Map Frame
        
        This enables the full coordinate transformation pipeline:
        ArUco Detection (camera) → Robot Coordinates (base) → World Coordinates (map)
        """
        try:
            # Lookup transform from robot base to camera optical frame
            # This transform is essential for converting marker poses from camera to robot coordinates
            self.t_base = self.tf_buffer.lookup_transform(
                self.robot_base_frame,              # Target frame: robot base_link
                self.camera_frame,                  # Source frame: camera optical frame
                rclpy.time.Time()                   # Use latest available transform
            )
            
            # Lookup transform from map to robot base frame
            # This enables publishing poses in global map coordinates for navigation
            self.t_map = self.tf_buffer.lookup_transform(
                self.frame_map,                     # Target frame: global map
                self.robot_base_frame,              # Source frame: robot base_link
                rclpy.time.Time()                   # Use latest available transform
            )
            
            # Log successful transform updates (can be disabled for performance)
            #self.get_logger().info(f"Transformation from {self.robot_base_frame} to {self.camera_frame} computed")
        except Exception as e:
            # Silently handle transform lookup failures to avoid log spam
            # Transform failures are common during system startup or frame interruptions
            self.get_logger().info(f'Could not transform from {self.robot_base_frame} to {self.camera_frame}: {e}')
            pass
        return

    def publish_frame_approach(self):
        """
        Publishes the approach frame positioned 20cm above the detected ArUco marker.
        
        This method generates and publishes the approach pose frame, which represents
        a safe pre-grasp position for the robot's end-effector. The approach frame
        is strategically positioned to provide:
        
        1. Safe clearance above the target object (20cm vertical offset)
        2. Proper orientation for downward grasping approach
        3. Collision-free positioning for motion planning
        
        Frame Calculation Process:
        1. Transform marker pose from camera to robot base coordinates
        2. Apply 20cm vertical offset (Z-axis translation)
        3. Apply 90-degree Y-axis rotation for proper gripper orientation
        4. Publish in both base_link and map coordinate frames
        
        The dual-frame publishing enables:
        - Robot motion planning (base_link frame)
        - Global navigation coordination (map frame)
        
        Only publishes when valid marker detection and transforms are available.
        Updates state tracking for sequence coordination.
        """
        # Ensure both robot-camera transform and marker detection are available
        if self.t_base is None or self.frame_aruco is None:
            return
            
        # Transform marker pose from camera coordinates to robot base coordinates
        frame_robot = self.get_frame_kdl(self.t_base)
        
        # Calculate approach frame: marker pose + 20cm Z offset + gripper orientation
        frame_approach = frame_robot * self.frame_aruco * Frame(Rotation(), Vector(0, 0, 0.2))
        
        # Apply 90-degree rotation about Y-axis for proper gripper approach orientation
        # This aligns the gripper for downward grasping motion
        frame_approach.M.DoRotY(np.pi / 2)
        
        # Publish approach frame in robot base coordinates
        self.publish_frame(frame_approach, self.robot_base_frame, self.frame_approach_name)
        
        # Transform and publish approach frame in global map coordinates
        frame_approach_in_map = self.get_frame_kdl(self.t_map) * frame_approach
        self.publish_frame(frame_approach_in_map, self.frame_map, self.frame_approach_in_map_name)
        
        # Update state tracking and check for completion
        self.approach_published = True
        self.check_and_publish_done()

    def publish_frame_target(self):
        """
        Publishes the target frame positioned 5cm above the detected ArUco marker.
        
        This method generates and publishes the target grasp pose frame, which represents
        the final contact position for the robot's gripper during grasping. The target
        frame is positioned to provide:
        
        1. Close proximity to the target object (5cm vertical offset)
        2. Proper contact positioning for secure grasping
        3. Optimal gripper orientation for object manipulation
        
        Frame Calculation Process:
        1. Transform marker pose from camera to robot base coordinates
        2. Apply 5cm vertical offset (Z-axis translation) for contact positioning
        3. Apply 90-degree Y-axis rotation for proper gripper orientation
        4. Publish in both base_link and map coordinate frames
        
        The smaller offset (5cm vs 20cm for approach) ensures:
        - Direct contact with the target object
        - Minimal clearance for secure grasping
        - Precise positioning for manipulation tasks
        
        Dual-frame publishing supports both local robot control and global coordination.
        Only publishes when valid marker detection and transforms are available.
        """
        # Ensure both robot-camera transform and marker detection are available
        if self.t_base is None or self.frame_aruco is None: 
            return
            
        # Transform marker pose from camera coordinates to robot base coordinates
        frame_robot = self.get_frame_kdl(self.t_base)
        
        # Calculate target frame: marker pose + 5cm Z offset + gripper orientation
        frame_target = frame_robot * self.frame_aruco * Frame(Rotation(), Vector(0, 0, 0.05))
        
        # Apply 90-degree rotation about Y-axis for proper gripper contact orientation
        # This aligns the gripper for optimal grasping contact
        frame_target.M.DoRotY(np.pi / 2)
        
        # Publish target frame in robot base coordinates
        self.publish_frame(frame_target, self.robot_base_frame, self.frame_target_name)
        
        # Transform and publish target frame in global map coordinates
        frame_target_in_map = self.get_frame_kdl(self.t_map) * frame_target
        self.publish_frame(frame_target_in_map, self.frame_map, self.frame_target_in_map_name)
        
        # Update state tracking and check for completion
        self.target_published = True
        self.check_and_publish_done()

    def publish_frame(self, frame, parent_name, tf_name):
        """
        Publishes a PyKDL Frame as a ROS TransformStamped message to the TF2 tree.
        
        This utility method converts PyKDL Frame objects into ROS2 TF2 transforms
        and broadcasts them to the transformation tree. The TF2 system enables
        other nodes to query these transforms for coordinate frame conversions.
        
        Conversion Process:
        1. Create ROS2 TransformStamped message with current timestamp
        2. Set parent and child frame identifiers
        3. Extract position (x, y, z) from PyKDL Frame
        4. Extract and normalize quaternion from PyKDL Rotation matrix
        5. Populate transform message with position and orientation data
        6. Broadcast transform to TF2 system
        
        Args:
            frame (Frame): PyKDL Frame containing pose information
            parent_name (str): Name of the parent coordinate frame
            tf_name (str): Name of the child coordinate frame being published
            
        The method includes quaternion normalization to ensure valid rotations
        and uses current timestamps for real-time coordinate frame updates.
        Published transforms become available for lookup by other ROS2 nodes.
        """
        # Create ROS2 TransformStamped message for TF2 broadcasting
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()       # Current timestamp for real-time updates
        t.header.frame_id = parent_name                        # Parent coordinate frame
        t.child_frame_id = tf_name                             # Child coordinate frame being published
        
        # Extract position components from PyKDL Frame
        t.transform.translation.x = frame.p.x()               # X position
        t.transform.translation.y = frame.p.y()               # Y position
        t.transform.translation.z = frame.p.z()               # Z position
        
        # Extract quaternion from PyKDL Rotation matrix
        quat = frame.M.GetQuaternion()                         # Get quaternion (x, y, z, w)
        quat = np.array(quat) / np.linalg.norm(quat)          # Normalize to ensure unit quaternion
        
        # Populate orientation components in ROS2 message
        t.transform.rotation.x = quat[0]                       # Quaternion X component
        t.transform.rotation.y = quat[1]                       # Quaternion Y component
        t.transform.rotation.z = quat[2]                       # Quaternion Z component
        t.transform.rotation.w = quat[3]                       # Quaternion W component
        
        # Broadcast transform to TF2 system for use by other nodes
        self.tf_broadcaster.sendTransform(t)

    def check_and_publish_done(self):
        """
        Checks completion status and publishes notification when both frames are ready.
        
        This method implements the completion logic for the grasping pose generation
        sequence. It monitors the publication status of both approach and target frames
        and coordinates the workflow completion notification.
        
        Completion Criteria:
        1. Approach frame has been successfully published
        2. Target frame has been successfully published
        3. Completion notification has not been sent yet (prevents duplicates)
        
        Upon meeting completion criteria:
        1. Publishes Boolean True message to /aruco_grasp_done topic
        2. Logs completion message for debugging and monitoring
        3. Sets completion flag to prevent duplicate notifications
        4. Disables further processing until next start command
        
        This coordination mechanism enables downstream processes to wait for
        complete pose generation before proceeding with grasping operations.
        The state machine approach ensures reliable sequencing of operations.
        """
        # Check if both frames have been published and completion notification hasn't been sent
        if self.approach_published and self.target_published and not self.done_sent:
            # Publish completion notification to coordinate downstream processes
            self.done_pub.publish(Bool(data=True))
            self.get_logger().info("Aruco grasp frames published, sending /aruco_grasp_done")
            
            # Update state to prevent duplicate notifications and disable processing
            self.done_sent = True                              # Mark completion notification as sent
            self.should_start = False                          # Disable processing until next start command

def main(args=None):
    """
    Main entry point for the ArUco grasp pose broadcaster node.
    
    This function initializes and runs the ArUco grasping pose generation system,
    which is responsible for detecting ArUco markers and generating appropriate
    coordinate frames for robotic grasping operations.
    
    System Capabilities:
    1. Real-time ArUco marker detection and pose validation
    2. Automatic generation of approach and target grasping frames
    3. Multi-coordinate frame publishing (base_link and map frames)
    4. Orientation filtering for reliable grasp pose generation
    5. Workflow coordination through completion notifications
    
    The node operates in a state-driven manner:
    - Waits for start commands to begin processing
    - Validates marker orientations before pose generation
    - Publishes periodic TF2 updates for real-time coordination
    - Provides completion notifications for workflow management
    
    Args:
        args (list, optional): Command line arguments for ROS2 initialization
        
    The function handles the complete node lifecycle from initialization
    to shutdown, ensuring proper resource cleanup upon termination.
    """
    # Initialize ROS2 Python client library
    rclpy.init(args=args)
    
    # Create the ArUco grasp broadcaster node instance
    aruco_node = ArucoGraspBroadcaster()
    
    # Run the node event loop until shutdown
    rclpy.spin(aruco_node)
    
    # Clean up resources upon shutdown
    aruco_node.destroy_node()
    rclpy.shutdown()

# Standard Python idiom for main function execution
# This ensures main() only runs when the script is executed directly,
# not when imported as a module by other Python files
if __name__ == '__main__':
    main()

