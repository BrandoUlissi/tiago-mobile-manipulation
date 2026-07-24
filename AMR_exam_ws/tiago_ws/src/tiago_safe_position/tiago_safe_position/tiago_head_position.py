import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import String


class TiagoHeadPosition(Node):
    """
    ROS2 node for controlling TIAGo robot head movement to a predefined safe position.
    
    This node provides a simple interface for moving the robot's head to a safe configuration:
    1. Listens for movement commands on /head_to_safe topic
    2. Executes a trajectory to move the head to a safe downward-looking position
    3. Publishes completion confirmation on /head_in_safe topic
    
    The safe head position is designed to:
    - Point the head downward to avoid sensor interference during navigation
    - Protect cameras and sensors from potential impacts
    - Provide a compact head configuration for transport or storage
    - Enable clear forward movement without head obstruction
    """

    def __init__(self):
        # Initialize the ROS2 node with a descriptive name
        super().__init__('tiago_head_position')

        # Create publisher for sending joint trajectory commands to the head controller
        # Uses the standard ROS2 joint trajectory interface for smooth, timed movements
        self.publisher_ = self.create_publisher(
            JointTrajectory,                    # Message type for multi-joint trajectories
            '/head_controller/joint_trajectory', # Topic name for TIAGo head controller
            10                                  # Queue size for message buffering
        )

        # Publisher for sending completion feedback to coordination systems
        # Allows other nodes to know when the head positioning is complete
        self.feedback_pub = self.create_publisher(
            String,             # Simple string message for status communication
            '/head_in_safe',    # Topic name for publishing completion status
            10                  # Queue size for message buffering
        )

        # Subscriber to listen for movement trigger commands from state machine or user
        # Provides a simple command interface for initiating head movement
        self.subscriber_ = self.create_subscription(
            String,                # Message type for simple text commands
            '/head_to_safe',       # Topic name for receiving movement commands
            self.command_callback, # Callback function to handle incoming messages
            10                     # Queue size for message buffering
        )

    def command_callback(self, msg):
        """
        Callback function for processing incoming movement commands.
        
        Args:
            msg (String): ROS2 String message containing the command
            
        Supported commands:
            - "go": Executes the head movement to safe position
            
        The safe position moves the head to look downward, which:
        - Protects cameras and sensors during robot movement
        - Provides optimal sensor positioning for navigation
        - Minimizes collision risk with environment
        """
        # Check if the received command matches the expected trigger
        if msg.data == 'go':
            self.get_logger().info('Received head movement command, executing...')

            # Create joint trajectory message for coordinated head movement
            trajectory_msg = JointTrajectory()
            # Specify the joints to be controlled (TIAGo head has 2 DOF)
            trajectory_msg.joint_names = [
                'head_1_joint',  # Pan joint (left-right rotation)
                'head_2_joint'   # Tilt joint (up-down rotation)
            ]

            # Define the target trajectory point with desired joint positions
            point = JointTrajectoryPoint()
            # Set target joint positions for safe configuration
            point.positions = [
                0.0,   # head_1_joint: 0 radians (center position, no pan)
                -1.0   # head_2_joint: -1 radian (~-57°, looking downward)
            ]
            # Set execution time for smooth movement (2 seconds duration)
            point.time_from_start.sec = 2

            # Add the trajectory point to the message and publish
            trajectory_msg.points.append(point)
            self.publisher_.publish(trajectory_msg)

            # Send confirmation message immediately after publishing trajectory
            # This notifies other systems that the command has been sent
            done_msg = String()
            done_msg.data = 'done'  # Simple completion signal
            self.feedback_pub.publish(done_msg)
            self.get_logger().info('Published head in safe position confirmation')



def main(args=None):
    """
    Main entry point for the TIAGo head position control node.
    
    This function handles the complete lifecycle of the node:
    1. Initializes the ROS2 Python client library
    2. Creates and starts the head position node
    3. Spins the node to handle incoming messages and callbacks
    4. Ensures proper cleanup on shutdown
    
    Args:
        args: Command line arguments (optional, defaults to None)
        
    The node uses a simple single-threaded executor since head control
    doesn't require the complex threading needed for motion planning.
    """
    # Initialize the ROS2 Python client library with optional command line arguments
    rclpy.init(args=args)
    
    # Create an instance of the head position control node
    node = TiagoHeadPosition()
    
    # Spin the node to process callbacks and keep it alive
    # This will continue until the node is shutdown or the process is terminated
    rclpy.spin(node)
    
    # Clean shutdown of ROS2 when spinning stops
    rclpy.shutdown()

# Standard Python idiom to run main() only when script is executed directly
# Prevents execution when this file is imported as a module
if __name__ == '__main__':
    main()