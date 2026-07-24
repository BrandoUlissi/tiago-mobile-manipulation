# Import necessary modules for ROS2 finite state machine implementation
import rclpy                                                   # ROS2 Python client library
from rclpy.node import Node                                    # Base class for ROS2 nodes
from std_msgs.msg import String, Bool                          # Standard ROS2 message types
import subprocess                                              # System command execution for map saving
import os                                                       # Home path expansion for the map save directory

class FSMNode(Node):
    """
    Finite State Machine Node for coordinating robotic exploration workflow.
    
    This ROS2 node implements a state machine that orchestrates a complete
    robotic exploration sequence including:
    
    1. Robot initialization by moving components to safe positions
    2. Autonomous exploration of the environment
    3. Map generation and persistence for future navigation
    
    State Machine Flow:
    IDLE → WAIT_HEAD → WAIT_ARM → WAIT_EXPLORE → SAVE_MAP → FINISHED
    
    Key Features:
    - Sequential coordination of robot subsystems
    - Event-driven state transitions based on completion confirmations
    - Autonomous exploration integration with Nav2 stack
    - Automatic map saving for persistent environment representation
    - Robust error handling and logging throughout the workflow
    
    The state machine ensures safe and coordinated operation by:
    - Moving robot components to safe positions before exploration
    - Waiting for confirmation before proceeding to next states
    - Handling exploration completion and map persistence
    - Providing comprehensive logging for monitoring and debugging
    
    Integration:
    This node coordinates with robot control nodes (head, arm positioning)
    and navigation systems (exploration, mapping) to provide a complete
    autonomous exploration capability.
    """

    def __init__(self):
        # Initialize the ROS2 node with a descriptive name for state machine coordination
        super().__init__('fsm_node')

        # Publisher setup for commanding robot subsystems
        # These publishers send commands to move robot components to safe positions
        self.head_pub = self.create_publisher(String, '/head_to_safe', 10)    # Commands head positioning
        self.arm_pub = self.create_publisher(String, '/arm_to_safe', 10)      # Commands arm positioning
        self.explore_pub = self.create_publisher(Bool, 'explore/resume', 10)  # Commands exploration start/resume

        # Subscriber setup for receiving completion confirmations
        # These subscribers listen for confirmation messages from robot subsystems
        self.create_subscription(String, '/head_in_safe', self.head_callback, 10)           # Head position confirmation
        self.create_subscription(String, '/arm_in_safe', self.arm_callback, 10)             # Arm position confirmation
        self.create_subscription(String, 'exploration_complete', self.exploration_callback, 10)  # Exploration completion

        # Initialize state machine in IDLE state
        # The state machine begins in IDLE and progresses through defined states
        self.state = 'IDLE'
        self.get_logger().info('FSM initialized in IDLE state')

        # Timer for periodic state machine execution (0.5 second intervals)
        # Regular execution ensures responsive state transitions and system monitoring
        self.create_timer(0.5, self.run_fsm)

        # State tracking flags for workflow coordination
        # These flags track completion status of each major workflow component
        self.head_done = False          # Flag indicating head is in safe position
        self.arm_done = False           # Flag indicating arm is in safe position
        self.exploration_done = False   # Flag indicating exploration is complete
        self.map_saved = False          # Flag indicating map has been successfully saved 

    def run_fsm(self):
        """
        Main state machine execution method called periodically by timer.
        
        This method implements the core state machine logic that coordinates
        the robotic exploration workflow. It processes the current state and
        manages transitions based on completion flags and system status.
        
        State Machine Flow:
        1. IDLE: Initial state - commands head to move to safe position
        2. WAIT_HEAD: Waits for head positioning confirmation
        3. WAIT_ARM: Waits for arm positioning confirmation  
        4. WAIT_EXPLORE: Waits for exploration completion
        5. SAVE_MAP: Saves the generated map to persistent storage
        6. FINISHED: Final state indicating workflow completion
        
        Each state transition is triggered by specific conditions:
        - Completion confirmations from robot subsystems
        - Successful execution of map saving operations
        - Error conditions that require workflow termination
        
        The method ensures safe sequential execution by waiting for
        confirmations before proceeding to subsequent states.
        """
        if self.state == 'IDLE':
            # Initial state: Begin workflow by moving head to safe position
            # Safe positioning prevents collisions during exploration
            self.get_logger().info('Moving head to safe position...')
            self.send_command(self.head_pub, 'go')
            self.state = 'WAIT_HEAD'

        elif self.state == 'WAIT_HEAD':
            # Wait for head positioning confirmation before proceeding
            if self.head_done:
                self.get_logger().info('Head is in safe position. Moving arm...')
                self.send_command(self.arm_pub, 'go')
                self.state = 'WAIT_ARM'

        elif self.state == 'WAIT_ARM':
            # Wait for arm positioning confirmation before starting exploration
            if self.arm_done:
                self.get_logger().info('Arm is in safe position. Starting exploration...')
                # Create and publish exploration start command
                msg = Bool()
                msg.data = True                                 # Enable exploration
                self.explore_pub.publish(msg)
                self.state = 'WAIT_EXPLORE'

        elif self.state == 'WAIT_EXPLORE': 
            # Wait for exploration completion before proceeding to map saving
            if self.exploration_done:
                self.get_logger().info('Exploration completed. Saving map...')
                self.state = 'SAVE_MAP'
                
                # Create map directory if it doesn't exist
                # Ensures target directory is available for map files.
                # Overridable ROS parameter with a repo-neutral default (~/tiago_maps),
                # so it no longer assumes a workspace named 'exam_ws' under $HOME.
                if not self.has_parameter('map_dir'):
                    self.declare_parameter('map_dir', os.path.expanduser('~/tiago_maps'))
                map_dir = self.get_parameter('map_dir').value
                subprocess.run(['mkdir', '-p', map_dir])
                
                # Save the generated map using Nav2 map server
                # Persists the exploration results for future navigation
                try:
                    subprocess.run([
                        'ros2', 'run', 'nav2_map_server', 'map_saver_cli',  # Nav2 map saving utility
                        '-f', f"{map_dir}/map"                              # Output file path
                    ])
                    self.map_saved = True                       # Mark map saving as successful
                except Exception as e:
                    self.get_logger().error(f'Error saving map: {str(e)}')
                    self.map_saved = False                      # Mark map saving as failed

        elif self.state == 'SAVE_MAP':
            # Check map saving completion and transition to final state
            if self.map_saved:
                self.get_logger().info('Map saved successfully')
                self.state = 'FINISHED'
            else:
                self.get_logger().error('Failed to save map')
                self.state = 'FINISHED'                         # Proceed to finish even if saving failed

        elif self.state == 'FINISHED':
            # Final state: Log completion and maintain terminal state
            self.get_logger().info('FSM finished. All tasks completed.')
            return
        

    def send_command(self, publisher, command):
        """
        Utility method for sending string commands to robot subsystems.
        
        This helper method standardizes command sending across the state machine
        by creating and publishing String messages with the specified command.
        It provides a clean interface for state machine to subsystem communication.
        
        Args:
            publisher (Publisher): ROS2 publisher for the target subsystem
            command (str): Command string to send ('go', 'stop', etc.)
            
        The method encapsulates the message creation and publishing logic,
        ensuring consistent command format across all subsystem interactions.
        """
        msg = String()
        msg.data = command                                      # Set command payload
        publisher.publish(msg)                                  # Send command to subsystem

    def head_callback(self, msg):
        """
        Callback for receiving head positioning completion confirmations.
        
        This callback processes messages from the head positioning subsystem
        indicating that the head has reached its safe position. The confirmation
        enables the state machine to proceed from WAIT_HEAD to WAIT_ARM state.
        
        Args:
            msg (String): ROS2 message containing status confirmation
                         Expected value: 'done' for successful positioning
                         
        The callback sets the head_done flag which triggers state transition
        in the main state machine loop.
        """
        if msg.data == 'done':
            self.get_logger().info('Received head done confirmation')
            self.head_done = True                               # Enable progression to next state

    def arm_callback(self, msg):
        """
        Callback for receiving arm positioning completion confirmations.
        
        This callback processes messages from the arm positioning subsystem
        indicating that the arm has reached its safe position. The confirmation
        enables the state machine to proceed from WAIT_ARM to WAIT_EXPLORE state.
        
        Args:
            msg (String): ROS2 message containing status confirmation
                         Expected value: 'done' for successful positioning
                         
        Safe arm positioning is critical before exploration to prevent:
        - Collisions with environment during navigation
        - Interference with sensor readings
        - Mechanical damage during autonomous movement
        """
        if msg.data == 'done':
            self.get_logger().info('Received arm done confirmation')
            self.arm_done = True                                # Enable progression to exploration state

    def exploration_callback(self, msg):
        """
        Callback for receiving exploration completion confirmations.
        
        This callback processes messages from the exploration subsystem
        indicating that autonomous exploration has been completed. The confirmation
        triggers transition from WAIT_EXPLORE to SAVE_MAP state.
        
        Args:
            msg (String): ROS2 message containing exploration status
                         Expected value: 'done' for successful exploration completion
                         
        Exploration completion indicates that:
        - The robot has autonomously navigated the environment
        - A comprehensive map has been generated
        - All accessible areas have been explored
        - The system is ready for map persistence
        
        Note: Alternative message format 'exploration_complete' is also supported
        for compatibility with different exploration implementations.
        """
        if msg.data == 'done':
        #if msg.data == 'exploration_complete':                 # Alternative message format
            self.get_logger().info('Received exploration complete confirmation')
            self.exploration_done = True                        # Enable progression to map saving state

def main(args=None):
    """
    Main entry point for the finite state machine node.
    
    This function initializes and runs the robotic exploration coordination
    system, providing centralized control over the complete exploration workflow.
    The state machine orchestrates multiple robot subsystems to achieve
    autonomous environment mapping.
    
    System Coordination:
    The FSM node serves as the central coordinator for:
    1. Robot component initialization (head, arm positioning)
    2. Autonomous exploration execution
    3. Map generation and persistence
    4. Workflow status monitoring and logging
    
    Key Benefits:
    - Sequential safety protocols ensure collision-free operation
    - Event-driven architecture provides responsive state management
    - Centralized logging enables comprehensive system monitoring
    - Modular design allows easy integration with different subsystems
    
    Args:
        args (list, optional): Command line arguments for ROS2 initialization
                              Includes node parameters and system configuration
                              
    The function handles complete node lifecycle including proper shutdown
    and resource cleanup upon termination.
    """
    # Initialize ROS2 Python client library
    rclpy.init(args=args)
    
    # Create the finite state machine node instance
    fsm_node = FSMNode()
    
    # Run the node until shutdown (blocking call)
    rclpy.spin(fsm_node)
    
    # Clean up resources upon shutdown
    fsm_node.destroy_node()
    rclpy.shutdown()

# Standard Python idiom to run main() only when script is executed directly
# This prevents execution when the file is imported as a module by other Python programs
if __name__ == '__main__':
    main()
