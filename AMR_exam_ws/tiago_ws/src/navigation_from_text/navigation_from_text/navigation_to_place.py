import rclpy
import time
from rclpy.node import Node
import os
from geometry_msgs.msg import PoseStamped
from tf2_ros import Duration
from action_msgs.msg import GoalStatus
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import math
from std_msgs.msg import Empty, Bool
from ament_index_python.packages import get_package_share_directory

class NavigatorFromTextfile(Node):
    """
    ROS2 node for autonomous navigation to predefined place/drop-off locations using Nav2.
    
    This node provides a complete navigation system specifically designed for place operations:
    1. Reads navigation goals from a text file containing drop-off coordinates
    2. Waits for trigger commands to initiate navigation to place locations
    3. Uses Nav2 action server for autonomous path planning and execution
    4. Monitors navigation progress and handles timeouts/cancellations
    5. Publishes completion status regardless of success/failure for workflow continuation
    
    Key differences from pick navigation:
    - Uses different topic names (/start_navigation_to_place, /navigation_to_place_done)
    - Reads from goals_place.txt file for place-specific locations
    - Always publishes completion message to ensure workflow continuation
    - Designed for the "place" phase of pick-and-place operations
    
    The navigation system integrates with:
    - Nav2 navigation stack (AMCL localization, behavior tree navigator)
    - File-based goal configuration for easy place location management
    - State machine coordination through ROS2 topics
    - Robust completion signaling for pick-and-place workflows
    """
    def __init__(self):
        # Initialize the ROS2 node with a descriptive name for action server control
        super().__init__('action_server_control')
        
        # Create an action client for the NavigateToPose action
        # This interfaces with Nav2's navigation system for autonomous movement to place locations
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Wait for the action server to be available before proceeding
        # Ensures Nav2 navigation stack is ready to receive place navigation goals
        self.nav_to_pose_client.wait_for_server()
        
        # Subscribe to the topic to start navigation to place location
        # Provides external trigger interface for state machine coordination in place operations
        self.start_sub = self.create_subscription(
            Empty,                           # Simple trigger message (no data payload)
            '/start_navigation_to_place',    # Topic name for receiving place navigation commands
            self.start_cb,                   # Callback function to handle place triggers
            10                              # Queue size for message buffering
        )
        
        # Publisher to notify when navigation to place location is complete
        # Critical for pick-and-place workflow coordination - signals when robot is ready for drop-off
        self.done_pub = self.create_publisher(
            Bool,                           # Boolean message for completion status
            '/navigation_to_place_done',    # Topic name for publishing place navigation completion
            10                             # Queue size for message buffering
        )
        
        # Internal flag to trigger navigation execution
        # Set by subscription callback, checked in main loop for place operations
        self.should_start = False
        
        print('Init completed')

    def start_cb(self, msg):
        """
        Callback for the /start_navigation_to_place topic.
        
        This callback is triggered when an external system (like a state machine)
        sends a command to begin navigation to the place/drop-off location.
        
        Args:
            msg (Empty): Empty message used as a simple trigger
            
        The callback sets an internal flag that the main loop monitors and logs
        the event in Italian for debugging (maintaining original language for consistency).
        This provides clean separation between event handling and navigation execution.
        """
        # Set flag to indicate place navigation should start
        self.should_start = True
        # Log the event in Italian (maintaining original debug language)
        self.get_logger().info('Ricevuto comando di start navigation_to_place e inviato ACK')

    def goToPose(self, pose):
        """
        Sends a navigation goal to the NavigateToPose action server for place operations.
        
        This method handles the complete process of sending a place navigation goal:
        1. Ensures the action server is available
        2. Creates and sends the goal message with place location
        3. Waits for goal acceptance
        4. Stores handles for monitoring progress to drop-off location
        
        Args:
            pose (PoseStamped): Target place pose containing position and orientation for drop-off
            
        Returns:
            bool: True if goal was accepted, False if rejected
            
        The method uses asynchronous communication with the Nav2 action server,
        allowing for non-blocking goal submission and progress monitoring during place operations.
        """
        self.debug("Waiting for 'NavigateToPose' action server")
        # Ensure the action server is available before sending place navigation goals
        # This prevents errors if Nav2 navigation stack is not fully initialized
        while not self.nav_to_pose_client.wait_for_server(timeout_sec=1.0):
            self.info("'NavigateToPose' server not available, waiting...")

        # Create the navigation goal message with the target place location
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        # Log the place navigation target for debugging and monitoring
        self.info('Navigating to goal: ' + str(pose.pose.position.x) + ' ' + str(pose.pose.position.y) + '...')

        # Send the goal asynchronously and register feedback callback
        # The feedback callback allows real-time monitoring of place navigation progress
        send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg, self._feedbackCallback)
        rclpy.spin_until_future_complete(self, send_goal_future)

        # Store the goal handle for later monitoring and potential cancellation
        self.goal_handle = send_goal_future.result()
        if not self.goal_handle.accepted:
            # Log error if the place navigation goal was rejected by the action server
            self.error('Goal to ' + str(pose.pose.position.x) + ' ' + str(pose.pose.position.y) + ' was rejected!')
            return False

        # Store the result future for monitoring place navigation completion
        self.result_future = self.goal_handle.get_result_async()
        return True

    def isNavComplete(self):
        """
        Checks if the place navigation action is complete and handles result processing.
        
        This method monitors the place navigation progress and:
        1. Checks if the navigation task has finished (success, failure, or cancellation)
        2. Publishes completion notifications for ANY outcome (critical for workflow continuation)
        3. Updates internal status tracking
        
        Key difference from pick navigation: Always publishes completion message regardless
        of success/failure to ensure the pick-and-place workflow can continue or handle errors.
        
        Returns:
            bool: True if navigation is complete (regardless of outcome), False if still in progress
            
        The method uses non-blocking future checking to avoid freezing the main loop
        while providing timely completion detection for place operations.
        """
        # Check if no active navigation task exists (was cancelled or completed)
        if not self.result_future:
            return True

        # Use non-blocking check with short timeout to avoid blocking main loop
        # This allows the node to remain responsive while monitoring place navigation
        rclpy.spin_until_future_complete(self, self.result_future, timeout_sec=0.10)

        # Process the result if place navigation has completed
        if self.result_future.result():
            # Store the final status for later reference
            self.status = self.result_future.result().status
            
            # For place operations, publish completion message regardless of outcome
            # This ensures workflow continuation even if navigation fails
            if self.status != GoalStatus.STATUS_SUCCEEDED:
                # If navigation did not succeed, still publish done message for workflow coordination
                done_msg = Bool()
                done_msg.data = True  # Always True to signal completion, not success
                self.done_pub.publish(done_msg)
                self.get_logger().info("Navigation completed, published done message")
            return True
        else:
            # Place navigation still in progress
            return False

    def getFeedback(self):
        """
        Returns the latest feedback from the place navigation action.
        
        Feedback during place navigation typically includes:
        - Current robot position relative to drop-off location
        - Estimated time remaining to reach place position
        - Distance to drop-off goal
        - Navigation time elapsed
        - Current navigation state and behavior
        
        Returns:
            NavigateToPose.Feedback: Latest feedback message from Nav2, or None if unavailable
        """
        return self.feedback

    def getResult(self):
        """
        Returns the final status of the place navigation action.
        
        Possible status values for place operations:
        - STATUS_SUCCEEDED: Navigation to place location completed successfully
        - STATUS_CANCELED: Place navigation was cancelled by user or system
        - STATUS_ABORTED: Place navigation failed due to obstacles or planning failure
        
        Returns:
            int: GoalStatus constant indicating the final result of place navigation
        """
        return self.status

    def waitUntilNav2Active(self):
        """
        Waits until the Nav2 navigation stack is fully active and ready for place operations.
        
        This method ensures all critical Nav2 components are operational before place navigation:
        - AMCL (Adaptive Monte Carlo Localization) for robot pose estimation
        - BT Navigator (Behavior Tree Navigator) for navigation execution
        
        This is essential before sending place navigation goals to prevent failures
        due to incomplete system initialization during drop-off operations.
        """
        self.info('Wait for Nav2 to be ready...')
        # Wait for localization system to be active for place operations
        self._waitForNodeToActivate('amcl')
        # Wait for navigation behavior tree to be active for place navigation
        self._waitForNodeToActivate('bt_navigator')
        self.info('Nav2 is ready for use!')

    def _waitForNodeToActivate(self, node_name):
        """
        Waits for a specific lifecycle-managed node to reach the 'active' state.
        
        Nav2 uses lifecycle management where nodes transition through states:
        unconfigured -> inactive -> active -> finalized
        
        This method ensures a node is fully operational before proceeding with place operations.
        
        Args:
            node_name (str): Name of the lifecycle node to monitor
            
        The method polls the node's state service until it reports 'active',
        which indicates the node is fully initialized and ready for place navigation.
        """
        self.debug('Waiting for ' + node_name + ' to become active..')
        
        # Construct the service name for getting node state
        node_service = node_name + '/get_state'
        state_client = self.create_client(GetState, node_service)

        # Wait for the state service to become available
        while not state_client.wait_for_service(timeout_sec=1.0):
            self.info(node_service + ' service not available, waiting...')

        # Create request message for state query
        req = GetState.Request()
        state = 'unknown'
        
        # Poll the node state until it becomes 'active'
        while state != 'active':
            self.debug('Getting ' + node_name + ' state...')
            # Send asynchronous state request
            future = state_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            
            # Process the response if available
            if future.result() is not None:
                state = future.result().current_state.label
                self.debug('Result of get_state: %s' % state)
            
            # Short delay before next state check to avoid overwhelming the service
            time.sleep(2)

    def _feedbackCallback(self, msg):
        """
        Callback to receive real-time feedback from the place navigation action.
        
        This callback is called periodically during place navigation to provide updates on:
        - Current robot position relative to drop-off goal
        - Estimated time remaining to reach place position
        - Total navigation time elapsed during place operation
        - Current navigation behavior state
        
        Args:
            msg: NavigateToPose feedback message containing place navigation progress data
            
        The feedback is stored for access by other methods and can be used for
        progress monitoring, timeout detection, and user interface updates during place operations.
        """
        # Store the latest feedback for access by monitoring methods
        self.feedback = msg.feedback

    # Logging utility methods for different message severity levels
    # These provide consistent logging interface throughout the place navigation system
    
    def info(self, msg):
        """Log informational messages for normal place operation tracking."""
        self.get_logger().info(msg)

    def warn(self, msg):
        """Log warning messages for potential issues during place operations."""
        self.get_logger().warn(msg)

    def error(self, msg):
        """Log error messages for serious problems during place navigation."""
        self.get_logger().error(msg)

    def debug(self, msg):
        """Log debug messages for detailed place navigation troubleshooting."""
        self.get_logger().debug(msg)

    def cancelNav(self):
        """
        Cancels the current place navigation goal and stops robot movement.
        
        This method provides emergency stop functionality for place operations and can be called when:
        - Place navigation takes too long (timeout during drop-off approach)
        - External systems request cancellation
        - Error conditions are detected during place operations
        - User intervention is required
        
        The cancellation is sent asynchronously to the Nav2 action server,
        which will stop the robot and clean up the current place navigation task.
        """
        self.info('Cancelling navigation...')
        # Send cancellation request to the navigation action server
        self.goal_handle.cancel_goal_async()

    def navigation(self):
        """
        Main place navigation monitoring loop that tracks progress and handles completion.
        
        This method for place operations:
        1. Continuously monitors place navigation progress using feedback
        2. Displays estimated time of arrival to drop-off location periodically
        3. Implements timeout protection (600 seconds maximum for place operations)
        4. Handles all possible place navigation outcomes (success, cancellation, failure)
        
        The loop runs until place navigation completes and provides comprehensive
        status reporting for debugging and coordination with drop-off operations.
        """
        i = 0
        # Monitor place navigation progress until completion
        while not self.isNavComplete():
            i += 1
            feedback = self.getFeedback()
            
            # Display progress information periodically (every 5th iteration)
            # This provides user feedback without overwhelming the logs during place operations
            if feedback and i % 5 == 0:
                print('Estimated time of arrival: ' + '{0:.0f}'.format(
                    Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9) + ' seconds.')

            # Implement timeout protection to prevent infinite place navigation attempts
            # 600 seconds (10 minutes) is typically sufficient for most place navigation tasks
            if feedback and Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                self.cancelNav()

        # Process and report the final place navigation result
        result = self.getResult()
        if result == GoalStatus.STATUS_SUCCEEDED:
            self.info('Goal succeeded!')
        elif result == GoalStatus.STATUS_CANCELED:
            self.info('Goal was canceled!')
        elif result == GoalStatus.STATUS_ABORTED:
            self.info('Goal failed!')
        else:
            self.info('Goal has an invalid return status!')

def main(args=None):
    """
    Main entry point for the navigation to place system.
    
    This function implements the complete workflow for place operations:
    1. Initializes ROS2 and creates the place navigation node
    2. Reads and parses place navigation goals from goals_place.txt file
    3. Waits for external trigger commands to start place operations
    4. Executes navigation to the specified drop-off/place location
    5. Coordinates with other systems through status publishing for workflow continuation
    
    The system uses file-based configuration for easy place location management
    and provides robust error handling for various failure modes during drop-off operations.
    
    Args:
        args: Command line arguments (optional)
    """
    # Initialize ROS2 Python client library
    rclpy.init()
    # Create the place navigation node instance
    navigator = NavigatorFromTextfile()

    # READ AND VALIDATE THE PLACE GOALS FILE ONCE AT STARTUP
    # This approach loads place locations once and reuses them, improving performance
    path_to_textfile = os.path.join(
        get_package_share_directory('navigation_from_text'), 'goals_place.txt')
    
    # Check if the place goals file exists before attempting to read
    if os.path.exists(path_to_textfile):
        with open(path_to_textfile, "r") as file:
            # Read all lines and remove empty ones for robust parsing
            lines = [line.strip() for line in file if line.strip()]

        # Validate that the file contains at least one place goal
        if not lines:
            navigator.error("The goals file is empty or only contains blank lines.")
            return

        navigator.info(f"{len(lines)} goal(s) loaded from file.")
    else:
        navigator.error("The specified path doesn't exist.")
        return

    # PARSE ONLY THE FIRST PLACE GOAL FROM THE FILE
    # Expected format: "x y theta" (space-separated floating point values for drop-off location)
    try:
        goal_x, goal_y, goal_theta = map(float, lines[0].split())
    except ValueError:
        navigator.error("The first line of the goal file is invalid.")
        return

    # MAIN EXECUTION LOOP FOR PLACE OPERATIONS
    # Runs continuously, waiting for place triggers and executing drop-off navigation
    while rclpy.ok():
        # Wait for external start command for place operations (blocking until received)
        navigator.should_start = False
        while rclpy.ok() and not navigator.should_start:
            # Process ROS2 callbacks with short timeout to remain responsive
            rclpy.spin_once(navigator, timeout_sec=0.1)

        navigator.info("Start navigation to place received.")

        # Prepare PoseStamped goal message for Nav2 place navigation
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'  # Use map coordinate frame for place operations
        goal_pose.header.stamp = navigator.get_clock().now().to_msg()  # Current timestamp
        
        # Set target drop-off position from parsed file data
        goal_pose.pose.position.x = goal_x
        goal_pose.pose.position.y = goal_y

        # Convert theta (yaw angle) to quaternion for ROS2 orientation representation
        # Quaternion conversion for place orientation: q = [0, 0, sin(θ/2), cos(θ/2)] for z-axis rotation
        goal_pose.pose.orientation.z = math.sin(goal_theta / 2.0)  # Z component for place orientation
        goal_pose.pose.orientation.w = math.cos(goal_theta / 2.0)  # W component for place orientation

        # Execute the place navigation sequence
        navigator.goToPose(goal_pose)  # Send drop-off goal to Nav2
        navigator.navigation()        # Monitor progress until place navigation completion

        time.sleep(2)  # Brief pause before next iteration

        # Publish completion message for workflow coordination
        # This signals that the robot has reached the place location and is ready for drop-off operations
        navigator.done_pub.publish(Bool(data=True))

    # Clean shutdown when ROS2 is no longer OK
    navigator.destroy_node()
    rclpy.shutdown()

# Standard Python idiom to run main() only when script is executed directly
# This prevents execution when the file is imported as a module
if __name__ == '__main__':
    main()