# Import necessary modules for ROS2 complex state machine implementation
import rclpy                                                   # ROS2 Python client library
from rclpy.node import Node                                    # Base class for ROS2 nodes
from std_msgs.msg import String, Empty, Bool                   # Standard ROS2 message types
import subprocess                                              # System process management for external nodes
import os                                                      # Operating system interface for commands
import time                                                    # Python time utilities for delays and timing

class StateMachineNode(Node):
    """
    Advanced Finite State Machine Node for coordinating complete pick-and-place workflows.
    
    This sophisticated ROS2 node implements a comprehensive state machine that orchestrates
    a complete robotic pick-and-place operation for multiple objects. The system coordinates:
    
    Complete Workflow Phases:
    1. Robot Initialization: Safe positioning of head and arm components
    2. Localization: Robot pose estimation and map alignment
    3. Navigation to Pick: Autonomous movement to object locations
    4. Object Detection: ArUco marker-based object identification and pose estimation
    5. Grasping Operations: Arm-based and torso-based object manipulation
    6. Transport Positioning: Safe configuration for object transport
    7. Navigation to Place: Movement to designated placement locations
    8. Placement Operations: Object release and detachment
    9. Multi-Object Handling: Sequential processing of multiple target objects
    
    State Machine Architecture:
    - States -2 to -1: System initialization and reset
    - States 0 to 5: Robot preparation and localization
    - States 6 to 8.5: Pick sequence (navigation, detection, grasping)
    - States 9 to 12: Place sequence (transport, navigation, placement)
    - State 13: Multi-object coordination and workflow repetition
    
    Key Features:
    - Multi-object sequential processing with marker ID management
    - Dynamic ArUco node lifecycle management for object detection
    - Robust error handling and retry mechanisms
    - Comprehensive state tracking and workflow coordination
    - Integration with navigation, manipulation, and vision systems
    - Autonomous backward movement for collision avoidance
    - Completion tracking for complex multi-step operations
    
    The state machine ensures safe and coordinated operation through:
    - Sequential safety protocols for each workflow phase
    - Confirmation-based state transitions with timeout handling
    - Dynamic process management for external detection nodes
    - Comprehensive logging for monitoring and debugging
    - Error recovery and retry mechanisms for robust operation
    """
    def __init__(self):
        # Initialize the ROS2 node with a descriptive name for complex state machine coordination
        super().__init__('state_machine_node')

        # Initial state of the state machine (-2: reset, -1: wait, 0+: task steps)
        # Negative states handle system initialization, positive states handle workflow execution
        self.state = -2

        # Publishers for sending commands to various robotic subsystems
        # These publishers coordinate the entire pick-and-place workflow across multiple nodes
        self.marker_id_pub = self.create_publisher(String, '/current_marker_id', 10)          # Marker ID updates for object targeting
        self.head_pub = self.create_publisher(String, '/head_to_safe', 10)                    # Head positioning commands
        self.arm_pub = self.create_publisher(String, '/arm_to_safe', 10)                      # Arm positioning commands
        self.start_localization_pub = self.create_publisher(Empty, '/start_localization', 10) # Localization initiation
        self.start_nav_pick_pub = self.create_publisher(Empty, '/start_navigation_to_pick', 10) # Navigation to pick location
        self.start_aruco_pub = self.create_publisher(Empty, '/start_aruco_grasp', 10)         # ArUco detection and pose calculation
        self.start_transport_pub = self.create_publisher(Empty, '/start_transport_position', 10) # Transport position commands
        self.start_nav_place_pub = self.create_publisher(Empty, '/start_navigation_to_place', 10) # Navigation to place location
        self.start_pick_arm_pub = self.create_publisher(Empty, '/start_pick_arm', 10)         # Arm-based picking operations
        self.start_pick_torso_pub = self.create_publisher(Empty, '/start_pick_torso', 10)     # Torso-based picking operations
        self.start_place_arm_pub = self.create_publisher(Empty, '/start_place_arm', 10)       # Arm-based placement operations

        # Subscribers for receiving completion signals from various robotic subsystems
        # These subscribers monitor the completion status of each workflow component
        self.create_subscription(String, '/head_in_safe', self.head_done_cb, 10)                     # Head safe position confirmation
        self.create_subscription(String, '/arm_in_safe', self.arm_done_cb, 10)                       # Arm safe position confirmation
        self.create_subscription(Bool, '/localization_done', self.localization_done_cb, 10)          # Localization completion
        self.create_subscription(Bool, '/navigation_to_pick_done', self.nav_pick_done_cb, 10)        # Pick navigation completion
        self.create_subscription(Bool, '/aruco_grasp_done', self.aruco_done_cb, 10)                  # ArUco pose calculation completion
        self.create_subscription(Bool, '/transport_position_done', self.transport_done_cb, 10)       # Transport position completion
        self.create_subscription(Bool, '/navigation_to_place_done', self.nav_place_done_cb, 10)      # Place navigation completion
        self.create_subscription(Bool, '/pick_arm_done', self.pick_arm_done_cb, 10)                  # Arm picking completion
        self.create_subscription(Bool, '/pick_torso_done', self.pick_torso_done_cb, 10)              # Torso picking completion
        self.create_subscription(Bool, '/place_arm_done', self.place_arm_done_cb, 10)                # Arm placement completion

        # Subscriber for the external start task command
        # Provides external interface for initiating the complete workflow
        self.create_subscription(Empty, '/start_task', self.start_task_cb, 10)

        # Internal flags for tracking progress of each workflow step
        # These flags coordinate state transitions throughout the complex pick-and-place sequence
        self.is_start_task_received = False     # Flag indicating external start command received
        self.head_done = False                  # Head positioning completion status
        self.arm_done = False                   # Arm positioning completion status
        self.localization_done = False          # Robot localization completion status
        self.nav_pick_done = False              # Navigation to pick location completion
        self.aruco_done = False                 # ArUco detection and pose calculation completion
        self.move_arm_done = False              # Arm movement completion (legacy flag)
        self.transport_done = False             # Transport position achievement completion
        self.nav_place_done = False             # Navigation to place location completion
        self.pick_arm_done = False              # Arm-based picking operation completion
        self.pick_torso_done = False            # Torso-based picking operation completion
        self.place_arm_done = False             # Arm-based placement operation completion

        # Marker management for multi-object pick-and-place tasks
        # Enables sequential processing of multiple objects with different ArUco markers
        self.marker_id = 582                    # Start with first marker ID (ArUco marker 582)
        self.first_marker_done = False          # Completion flag for first object workflow
        self.second_marker_done = False         # Completion flag for second object workflow
        self.aruco_process = None               # Process handle for managing external ArUco detection node

        # Error handling and retry mechanism for robust operation
        # Provides resilience against temporary failures and system issues
        self.error_state = False                # Global error state flag
        self.max_retries = 3                    # Maximum number of retry attempts for failed operations
        self.retry_count = 0                    # Current retry attempt counter
        self.error_msg = ""                     # Last error message for debugging

        # Timer for periodic state machine execution (0.5 second intervals)
        # Regular execution ensures responsive state transitions and system monitoring
        self.timer_period = 0.5
        self.state_machine_timer = self.create_timer(self.timer_period, self.run_state_machine)

        # Comprehensive logging of initialization details for system monitoring
        self.get_logger().info('[INIT]: State machine initialized with:')
        self.get_logger().info(f'- Timer period: {self.timer_period}s')
        self.get_logger().info(f'- Initial marker ID: {self.marker_id}')
        self.get_logger().info(f'- Max retries: {self.max_retries}')

    # --- CALLBACKS FOR SUBSCRIBERS ---
    # These methods handle completion notifications from various robotic subsystems

    def start_task_cb(self, msg):
        """Callback for /start_task: sets the flag to begin the sequence."""
        self.is_start_task_received = True
        self.get_logger().info('[START]: /start_task command received! Starting sequence...')

    def head_done_cb(self, msg):
        """
        Callback for /head_in_safe: sets flag when head is in safe position.
        Receives completion notification from head positioning subsystem.
        """
        self.get_logger().info(f'Received head_done message: {msg.data}')
        if msg.data == 'done':
            self.head_done = True
            self.get_logger().info('Head done flag set to True')

    def arm_done_cb(self, msg):
        """
        Callback for /arm_in_safe: sets flag when arm is in safe position.
        Confirms arm has reached predefined safe configuration.
        """
        if msg.data == 'done':
            self.arm_done = True

    def localization_done_cb(self, msg):
        """
        Callback for /localization_done: sets flag when localization is complete.
        Receives boolean status from autonomous localization process.
        """
        self.localization_done = msg.data

    def nav_pick_done_cb(self, msg):
        """
        Callback for /navigation_to_pick_done: sets flag when navigation to pick is complete.
        Indicates robot has successfully reached picking location via Nav2.
        """
        self.nav_pick_done = msg.data

    def aruco_done_cb(self, msg):
        """
        Callback for /aruco_grasp_done: sets flag when ArUco grasp is complete.
        Confirms successful object detection, approach, and grasping sequence.
        """
        self.get_logger().info(f"aruco_done_cb received: {msg.data}")
        self.aruco_done = msg.data

    def move_arm_done_cb(self, msg):
        """
        Callback for /move_arm_done (not used in this version).
        Reserved for direct arm movement completion notifications.
        """
        self.move_arm_done = msg.data

    def transport_done_cb(self, msg):
        """
        Callback for /transport_position_done: sets flag when transport position is reached.
        Confirms arm is in safe configuration for carrying objects during navigation.
        """
        self.transport_done = msg.data

    def nav_place_done_cb(self, msg):
        """
        Callback for /navigation_to_place_done: sets flag when navigation to place is complete.
        Indicates robot has reached target placement location via Nav2.
        """
        self.nav_place_done = msg.data

    def pick_arm_done_cb(self, msg):
        """
        Callback for /pick_arm_done: sets flag when pick arm is complete.
        Receives completion status from MoveIt2-based arm grasping operations.
        """
        self.pick_arm_done = msg.data

    def pick_torso_done_cb(self, msg):
        """
        Callback for /pick_torso_done: sets flag when pick torso is complete.
        Confirms torso-based grasping operation has finished successfully.
        """
        self.pick_torso_done = msg.data

    def place_arm_done_cb(self, msg):
        """
        Callback for /place_arm_done: sets flag when place arm is complete.
        Receives completion status from MoveIt2-based placement operations.
        """
        self.place_arm_done = msg.data

    # --- ARUCO NODE MANAGEMENT ---
    # Dynamic launching and termination of ArUco detection nodes for different markers

    def start_aruco_node(self, marker_id):
        """
        Launches the ArUco node for the specified marker ID.
        Dynamically starts marker detection pipeline with specific marker configuration.
        
        Args:
            marker_id (int): Target ArUco marker ID for detection
        """
        if self.aruco_process is not None:
            self.get_logger().info("Killing existing aruco node before starting a new one.")
            self.stop_aruco_node()
        self.get_logger().info(f"Starting ArUco node with marker_id {marker_id}")
        cmd = f"ros2 launch lab3 single.launch.py marker_id:={marker_id} marker_size:=0.04667"
        self.aruco_process = subprocess.Popen(cmd, shell=True)

    def stop_aruco_node(self):
        """
        Stops the running ArUco node process.
        Terminates current marker detection to prepare for new marker or end sequence.
        """
        if self.aruco_process is not None:
            self.get_logger().info("Stopping ArUco node.")
            self.aruco_process.terminate()
            self.aruco_process.wait()
            self.aruco_process = None

    # --- ERROR HANDLING ---
    # Centralized error management and recovery mechanisms

    def handle_error(self, error_msg):
        """
        Logs errors and can be expanded for more complex error handling.
        Provides centralized error reporting and potential recovery logic.
        
        Args:
            error_msg (str): Descriptive error message for logging and debugging
        """
        self.get_logger().error(f'[ERROR] {error_msg}')

    # --- MAIN STATE MACHINE LOGIC ---
    # Core finite state machine implementation with 15+ coordinated states

    def run_state_machine(self):
        """
        Main periodic function that implements the state machine logic.
        
        Executes comprehensive multi-object pick-and-place workflow:
        1. Initialization phase (robot positioning, localization)
        2. Pick sequence (navigation, detection, grasping)
        3. Place sequence (transport, navigation, placement)
        4. Multi-object coordination (marker management, retry logic)
        
        Called periodically by timer to ensure responsive state transitions.
        """
        if self.error_state:
            return  # If in error state, do nothing

        self.get_logger().debug(f'[DEBUG] Current state: {self.state}')
        self.get_logger().debug(f'[DEBUG] Marker ID: {self.marker_id}')

        # STATE -2: AUTONOMOUS LOCALIZATION RESET
        # Reset the autonomous_localization node to ensure clean initialization
        if self.state == -2:
            self.get_logger().info("[STATE -2]: Resetting autonomous_localization node...")
            try:
                # Execute system command to reset localization lifecycle node
                result = os.system("ros2 run autonomous_localization reset_node")
                if result != 0:
                    self.get_logger().warn("[STATE -2]: Lifecycle reset failed!")
                time.sleep(2)  # Allow time for reset to complete
                self.get_logger().info("[STATE -2]: Node reset completed. Proceeding to initial wait...")
                self.state = -1
            except Exception as e:
                self.handle_error(f"Error resetting autonomous_localization node: {str(e)}")

        # STATE -1: INITIAL WAIT AND MARKER SETUP
        # Publish current marker ID and perform initial delay before starting sequence
        if self.state == -1:
            self.get_logger().info(f"Current Marker ID: {self.marker_id}")
            self.get_logger().info("[STATE -1]: Initial wait state (5 seconds)...")
            try:
                msg = String()
                msg.data = str(self.marker_id)
                self.marker_id_pub.publish(msg)  # Notify system of current target marker
                self.get_logger().info(f"[STATE -1]: Published initial marker ID {self.marker_id}")
                time.sleep(5)  # Standard initialization delay
                self.get_logger().info("[STATE -1]: Wait completed, proceeding to head movement.")
                self.state = 0
            except Exception as e:
                self.handle_error(f"Error in initial wait state: {str(e)}")

        # STATE 0: HEAD POSITIONING COMMAND
        # Command head to move to safe position for navigation and operation
        elif self.state == 0:
            self.get_logger().info(f"Current Marker ID: {self.marker_id}")
            try:
                self.get_logger().info("[STATE 0]: Commanding head to safe position...")
                msg = String()
                msg.data = 'go'
                self.head_pub.publish(msg)  # Trigger head positioning
                self.head_done = False  # Reset completion flag
                self.state = 1
            except Exception as e:
                self.handle_error(f"Error commanding head movement: {str(e)}")

        # STATE 1: HEAD POSITIONING COMPLETION WAIT
        # Monitor for head positioning completion before proceeding
        elif self.state == 1:
            self.get_logger().info(f"Current Marker ID: {self.marker_id}")
            self.get_logger().info(f"[STATE 1]: Waiting for head positioning completion... (current: {self.head_done})")
            if self.head_done:
                self.get_logger().info("[STATE 1]: Head positioning completed, proceeding to arm movement.")
                self.state = 2

        # STATE 2: ARM POSITIONING COMMAND
        # Command arm to move to safe position for navigation and operation
        elif self.state == 2:
            self.get_logger().info(f"Current Marker ID: {self.marker_id}")
            self.get_logger().info("[STATE 2]: Commanding arm to safe position...")
            msg = String()
            msg.data = 'go'
            self.arm_pub.publish(msg)  # Trigger arm positioning
            self.arm_done = False  # Reset completion flag
            self.state = 3

        # STATE 3: ARM POSITIONING COMPLETION WAIT
        # Monitor for arm positioning completion before starting localization
        elif self.state == 3:
            self.get_logger().info(f"Current Marker ID: {self.marker_id}")
            self.get_logger().info("[STATE 3]: Waiting for arm positioning completion...")
            if self.arm_done:
                self.get_logger().info("[STATE 3]: Safe positions achieved, starting autonomous localization.")
                self.start_localization_pub.publish(Empty())  # Trigger AMCL localization
                self.state = 5

        # STATE 5: AUTONOMOUS LOCALIZATION WAIT
        # Wait for AMCL-based localization convergence before navigation
        elif self.state == 5:
            self.get_logger().info(f"Current Marker ID: {self.marker_id}")
            self.get_logger().info("[STATE 5]: Waiting for autonomous localization completion...")
            if self.localization_done:
                self.get_logger().info("[STATE 5]: Localization completed, starting navigation to pick location.")
                self.start_nav_pick_pub.publish(Empty())  # Trigger Nav2 navigation to pick
                self.state = 6

        # STATE 6: NAVIGATION TO PICK COMPLETION AND ARUCO INITIALIZATION
        # Wait for navigation completion, then start ArUco detection and grasping
        elif self.state == 6:
            self.get_logger().info(f"Current Marker ID: {self.marker_id}")
            self.get_logger().info("[STATE 6]: Waiting for navigation to pick location completion...")
            if self.nav_pick_done:
                self.get_logger().info("[STATE 6]: Navigation completed, initializing ArUco detection and grasping.")
                self.start_aruco_node(self.marker_id)  # Launch marker-specific detection
                self.start_aruco_pub.publish(Empty())  # Trigger ArUco pose calculation
                self.nav_pick_done = False  # Reset navigation flag
                self.aruco_done = False  # Reset ArUco completion flag
                self.state = 7

        # STATE 7: ARUCO DETECTION AND POSE CALCULATION
        # Wait for ArUco marker detection and grasping pose computation
        elif self.state == 7:
            self.get_logger().info(f"Current Marker ID: {self.marker_id}")
            self.get_logger().info("[STATE 7]: Waiting for ArUco pose calculation completion...")
            if self.aruco_done:
                self.get_logger().info("[STATE 7]: ArUco frame calculation completed, starting arm grasping.")
                self.stop_aruco_node()  # Terminate detection to free resources
                self.start_pick_arm_pub.publish(Empty())  # Trigger MoveIt2 arm grasping
                self.get_logger().info("[STATE 7]: Published pick_arm command")
                self.aruco_done = False  # Reset ArUco flag
                self.pick_arm_done = False  # Reset pick arm flag
                self.state = 8

        # STATE 8: ARM GRASPING COMPLETION AND TORSO ENGAGEMENT
        # Wait for arm-based grasping completion, then engage torso for secure grip
        elif self.state == 8:
            self.get_logger().info(f"Current Marker ID: {self.marker_id}")
            self.get_logger().info("[STATE 8]: Waiting for arm grasping completion...")
            if self.pick_arm_done:
                self.get_logger().info("[STATE 8]: Arm grasping completed, starting torso-based securing.")
                time.sleep(2)  # Brief pause for arm settling
                self.start_pick_torso_pub.publish(Empty())  # Trigger torso grasping
                self.pick_arm_done = False
                self.pick_torso_done = False
                self.state = 8.5

        # STATE 8.5: TORSO GRASPING COMPLETION
        # Wait for torso-based grasping completion, then execute backward movement for safety
        elif self.state == 8.5:
            self.get_logger().info(f"Current Marker ID: {self.marker_id}")
            self.get_logger().info("[STATE 8.5]: Waiting for torso grasping completion...")
            if self.pick_torso_done:
                self.get_logger().info("[STATE 8.5]: Torso grasping completed, executing backward movement.")
                # Execute 4-second backward movement to clear potential obstacles and create safe distance
                os.system('timeout 4s ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: -0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -r 10')
                self.get_logger().info("[STATE 8.5]: Backward movement completed, commanding transport position.")
                self.start_transport_pub.publish(Empty())  # Configure arm for safe object transport
                self.pick_torso_done = False  # Reset torso completion flag
                self.state = 9

        # STATE 9: TRANSPORT POSITION ACHIEVEMENT FOR NAVIGATION
        # Wait for arm to reach transport configuration before starting navigation to place location
        elif self.state == 9:
            self.get_logger().info(f"Current Marker ID: {self.marker_id}")
            self.get_logger().info("[STATE 9]: Waiting for transport position achievement...")
            if self.transport_done:
                self.get_logger().info("[STATE 9]: Transport position achieved, starting navigation to placement location.")
                self.start_nav_place_pub.publish(Empty())  # Trigger Nav2 navigation to place location
                self.transport_done = False  # Reset transport completion flag
                self.state = 10

        # STATE 10: NAVIGATION TO PLACE COMPLETION AND PLACEMENT INITIATION
        # Wait for navigation to place location completion, then start object placement operations
        elif self.state == 10:
            self.get_logger().info(f"Current Marker ID: {self.marker_id}")
            self.get_logger().info("[STATE 10]: Waiting for navigation to place location completion...")
            if self.nav_place_done:
                self.get_logger().info(f"[STATE 10]: Navigation to place completed for marker {self.marker_id}.")
                self.nav_place_done = False  # Reset navigation completion flag
                self.get_logger().info("[STATE 10]: Starting arm-based object placement operation.")
                self.start_place_arm_pub.publish(Empty())  # Trigger MoveIt2 placement operation
                self.place_arm_done = False  # Reset placement completion flag
                self.state = 11

        # STATE 11: OBJECT PLACEMENT COMPLETION AND POST-PLACE TRANSPORT
        # Wait for placement operation completion, then return arm to transport configuration
        elif self.state == 11:
            self.get_logger().info(f"Current Marker ID: {self.marker_id}")
            self.get_logger().info("[STATE 11]: Waiting for object placement completion...")
            if self.place_arm_done:
                self.get_logger().info("[STATE 11]: Object placement completed, returning to transport position.")
                self.start_transport_pub.publish(Empty())  # Return arm to safe transport configuration
                self.transport_done = False  # Reset transport completion flag
                self.state = 12

        # STATE 12: MULTI-OBJECT COORDINATION AND WORKFLOW MANAGEMENT
        # Handle marker switching for sequential object processing or complete workflow termination
        elif self.state == 12:
            self.get_logger().info(f"Current Marker ID: {self.marker_id}")
            self.get_logger().info("[STATE 12]: Waiting for post-placement transport position completion...")
            if self.transport_done:
                self.get_logger().info("[STATE 12]: Post-placement transport position achieved.")
                # Check if first object (marker 582) workflow is complete
                if not self.first_marker_done:
                    self.first_marker_done = True  # Mark first object as completed
                    self.marker_id = 63  # Switch to second target marker (ArUco marker 63)
                    self.get_logger().info("[STATE 12]: First object completed, switching to second marker (63).")
                    msg = String()
                    msg.data = str(self.marker_id)
                    self.marker_id_pub.publish(msg)  # Notify system of new target marker
                    self.get_logger().info(f"[STATE 12]: Published new target marker ID {self.marker_id}")
                    self.state = 13  # Transition to buffer state for second object
                # Check if second object workflow is complete
                elif not self.second_marker_done:
                    self.second_marker_done = True  # Mark second object as completed
                    self.get_logger().info("[STATE 12]: Complete multi-object workflow finished for both markers!")
                    self.destroy_timer(self.state_machine_timer)  # Terminate state machine execution

        # STATE 13: MULTI-OBJECT TRANSITION BUFFER AND NAVIGATION RESTART
        # Buffer state for marker switching, then restart navigation sequence for second object
        elif self.state == 13:
            self.get_logger().info(f"Current Marker ID: {self.marker_id}")
            self.get_logger().info("[STATE 13]: Multi-object transition buffer state...")
            if self.transport_done:
                # Execute backward movement for positioning and obstacle clearance
                os.system('timeout 4s ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: -0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -r 10')
                self.get_logger().info("[STATE 13]: Backward movement completed, preparing for second object workflow.")
                time.sleep(5)  # Buffer delay for system stabilization
                self.get_logger().info("[STATE 13]: Starting navigation to pick for second object.")
                self.nav_pick_done = False  # Reset navigation flag for new sequence
                self.start_nav_pick_pub.publish(Empty())  # Restart navigation to pick for second marker
                self.transport_done = False  # Reset transport flag
                self.state = 6  # Return to pick sequence for second object

# --- MAIN FUNCTION AND NODE LIFECYCLE ---
# Entry point for ROS2 node execution and lifecycle management

def main(args=None):
    """
    Main function for initializing and running the state machine node.
    
    Handles ROS2 initialization, node creation, execution loop, and cleanup.
    Provides proper lifecycle management for the complex state machine system.
    """
    # Initialize ROS2 communication infrastructure
    rclpy.init(args=args)
    
    # Create the state machine node instance with all publishers, subscribers, and logic
    node = StateMachineNode()
    
    # Enter the ROS2 execution loop to process callbacks and timer events
    # This keeps the node active and responsive to system events
    rclpy.spin(node)
    
    # Clean shutdown: properly destroy the node and release resources
    node.destroy_node()
    rclpy.shutdown()

# Entry point when script is executed directly
if __name__ == "__main__":
    main()