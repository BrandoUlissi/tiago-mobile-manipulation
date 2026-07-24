import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty, Bool
import time
import threading
import numpy as np

ROTATION_VELOCITY = -0.6  # rad/s, angular velocity for robot rotation during localization
TWO_PI = 6.28318530718    # 2*pi, used for full rotation calculations

class LocalizationNode(Node):
    """
    ROS2 Node for autonomous robot localization using AMCL (Adaptive Monte Carlo Localization).
    
    This node implements a localization strategy where the robot:
    1. Sets an initial pose with high uncertainty
    2. Rotates in place to gather sensor data from multiple orientations
    3. Monitors AMCL covariance to determine when localization is precise enough
    4. Reports success/failure of the localization process
    """
    def __init__(self):
        # Initialize the ROS2 node with a descriptive name
        super().__init__('autonomous_localization_node')

        # --- Subscribers ---
        # Listen for a command to start localization (Empty message trigger)
        self.create_subscription(Empty, '/start_localization', self.command_callback, 10)
        # Listen for AMCL pose updates (to monitor covariance and localization quality)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.amcl_callback, 10)
        # Listen for odometry updates (to get current robot orientation for initial pose)
        self.create_subscription(Odometry, '/mobile_base_controller/odom', self.odom_callback, 10)

        # --- Publishers ---
        # Publish velocity commands to rotate the robot during localization
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        # Publish the initial pose with high uncertainty to trigger AMCL reset
        self.init_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        # Publish when localization is done (True for success, False for failure)
        self.localization_done_pub = self.create_publisher(Bool, '/localization_done', 10)

        # --- Internal State Variables ---
        self.localization_active = False  # Flag to indicate if localization procedure is currently running

        # Initial covariance matrix setup (high uncertainty for x, y, and theta)
        # The covariance matrix is 6x6 flattened to 36 elements for ROS message format
        cov = np.zeros(36)  # Initialize all covariance values to zero
        cov[0] = 200.0   # x position covariance (high uncertainty in x direction)
        cov[7] = 200.0   # y position covariance (high uncertainty in y direction) 
        cov[35] = 1.0    # theta (orientation) covariance (moderate uncertainty in rotation)
        self.initial_covariance = cov.tolist()  # Convert to list for ROS message compatibility

        # Covariance threshold for considering localization as "good enough"
        # Lower values mean more precise localization is required
        self.cov_threshold = 0.07

        # Robot orientation storage (quaternion format: [x, y, z, w])
        # Default to no rotation (identity quaternion)
        self.tiago_orientation = [0, 0, 0, 1]

        # Storage for the latest AMCL covariance data received
        self.latest_amcl_cov = None

        # Log successful initialization
        self.get_logger().info("Localization node initialized.")

    def command_callback(self, msg):
        """
        Callback for /start_localization topic.
        Starts the localization procedure in a separate thread if not already running.
        Uses threading to prevent blocking the main ROS2 spin loop.
        """
        # Check if localization is not already in progress to avoid multiple concurrent runs
        if not self.localization_active:
            self.get_logger().info("Received /start_localization command.")
            # Set flag to indicate localization is starting
            self.localization_active = True
            # Start localization in separate thread to avoid blocking callback execution
            threading.Thread(target=self.run_localization).start()

    def amcl_callback(self, msg):
        """
        Callback for /amcl_pose topic.
        Stores the latest covariance from AMCL for later evaluation.
        The covariance indicates how certain AMCL is about the robot's pose.
        """
        # Extract and store covariance matrix from AMCL pose message
        # This 36-element array represents a 6x6 covariance matrix (x,y,z,roll,pitch,yaw)
        self.latest_amcl_cov = msg.pose.covariance

    def odom_callback(self, msg):
        """
        Callback for /mobile_base_controller/odom topic.
        Updates the robot's current orientation from odometry data.
        This orientation is used when setting the initial pose for localization.
        """
        # Extract quaternion orientation from odometry message
        o = msg.pose.pose.orientation
        # Store as list in [x, y, z, w] format for later use in initial pose
        self.tiago_orientation = [o.x, o.y, o.z, o.w]

    def publish_initial_pose(self):
        """
        Publishes an initial pose with high covariance to /initialpose topic.
        This tells AMCL to start with a very uncertain position, essentially resetting localization.
        The high covariance allows AMCL to consider a wide range of possible robot positions.
        """
        time.sleep(1.0)  # Wait a moment to ensure ROS2 system and AMCL are ready to receive the message
        
        # Create and populate the initial pose message
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"  # Set reference frame to map coordinate system
        msg.header.stamp = self.get_clock().now().to_msg()  # Use current time for message timestamp
        
        # Set the robot's current orientation from odometry (position will be estimated by AMCL)
        msg.pose.pose.orientation.x = self.tiago_orientation[0]
        msg.pose.pose.orientation.y = self.tiago_orientation[1]
        msg.pose.pose.orientation.z = self.tiago_orientation[2]
        msg.pose.pose.orientation.w = self.tiago_orientation[3]
        
        # Apply high uncertainty covariance to allow AMCL to explore pose space
        msg.pose.covariance = self.initial_covariance

        # Publish the initial pose and log the action
        self.init_pose_pub.publish(msg)
        self.get_logger().info("Initial pose published.")

    def rotate_robot(self, duration):
        """
        Rotates the robot in place for a specified duration (in seconds).
        Used to help AMCL converge by providing sensor data from different angles.
        The rotation exposes the robot's sensors to different parts of the environment.
        """
        # Create twist message for angular rotation
        twist = Twist()
        twist.angular.z = ROTATION_VELOCITY  # Set angular velocity (negative = clockwise)
        
        # Record start time for duration calculation
        start = self.get_clock().now().nanoseconds / 1e9

        # Continue rotating while within time duration and localization is still active
        while (self.get_clock().now().nanoseconds / 1e9 - start) < duration and self.localization_active:
            self.cmd_vel_pub.publish(twist)  # Send rotation command
            time.sleep(0.1)  # Small delay to avoid overwhelming the system with commands

        # Stop the robot after rotation is complete
        self.stop_robot()

    def stop_robot(self):
        """
        Stops the robot by publishing a zero Twist message.
        This ensures all linear and angular velocities are set to zero.
        """
        # Publish empty Twist message (all velocities default to 0.0)
        self.cmd_vel_pub.publish(Twist())
        self.get_logger().info("Robot stopped.")

    def check_covariance(self):
        """
        Checks if the latest AMCL covariance is below the threshold for x, y, and theta.
        Returns True if localization is considered precise enough.
        
        The covariance matrix indices used:
        - Index 0: x position variance
        - Index 7: y position variance  
        - Index 35: theta (yaw) orientation variance
        """
        # Check if any AMCL data has been received yet
        if self.latest_amcl_cov is None:
            self.get_logger().warn("No AMCL data received yet.")
            return False

        # Extract variance values for x, y, and theta from covariance matrix
        x_cov = self.latest_amcl_cov[0]    # Position uncertainty in x direction
        y_cov = self.latest_amcl_cov[7]    # Position uncertainty in y direction  
        theta_cov = self.latest_amcl_cov[35]  # Orientation uncertainty (yaw angle)

        # Log current covariance values for debugging and monitoring
        self.get_logger().info(f"Covariances: x={x_cov:.3f}, y={y_cov:.3f}, theta={theta_cov:.3f}")
        
        # Return True only if ALL covariance values are below the threshold
        # This ensures the robot is localized precisely in both position and orientation
        return (x_cov < self.cov_threshold and
                y_cov < self.cov_threshold and
                theta_cov < self.cov_threshold)

    def run_localization(self):
        """
        Main localization procedure implementing an iterative approach:
        1. Publishes initial pose with high uncertainty to reset AMCL
        2. Rotates the robot in place several times, checking AMCL covariance after each rotation
        3. If covariance is low enough, publishes success; otherwise, retries up to 100 times
        4. If still not localized after all attempts, publishes failure
        
        The rotation strategy helps AMCL by:
        - Providing sensor data from multiple orientations
        - Allowing particle filter to converge on correct pose
        - Reducing position and orientation uncertainty over time
        """
        self.get_logger().info("Starting localization procedure...")
        # Step 1: Reset AMCL with high uncertainty initial pose
        self.publish_initial_pose()

        # Step 2: Iterative rotation and covariance checking (max 100 attempts)
        for _ in range(100):  # Try up to 100 times (approximately 4 full rotations at current settings)
            # Check if localization was cancelled externally
            if not self.localization_active:
                break
                
            self.get_logger().info("Rotating robot for localization...")
            # Rotate for half the time needed for a full rotation
            # This provides good sensor coverage without over-rotating
            self.rotate_robot(duration=TWO_PI / abs(ROTATION_VELOCITY) / 2)

            # Step 3: Check if localization is precise enough
            if self.check_covariance():
                self.get_logger().info("Localization successful!")
                # Publish success result
                self.localization_done_pub.publish(Bool(data=True))
                self.localization_active = False  # Mark localization as complete
                return  # Exit the function successfully

            # Log progress for debugging
            self.get_logger().info("Localization not yet precise. Retrying...")

        # Step 4: Handle failure case (reached maximum attempts without success)
        self.get_logger().warn("Localization failed after several attempts.")
        # Publish failure result
        self.localization_done_pub.publish(Bool(data=False))
        self.localization_active = False  # Mark localization as complete (failed)

def main():
    """
    Main entry point for the autonomous localization node.
    Initializes ROS2, creates the localization node, and handles the execution loop.
    Includes proper cleanup and error handling for graceful shutdown.
    """
    # Initialize the ROS2 Python client library
    rclpy.init()
    
    # Create an instance of the localization node
    node = LocalizationNode()
    
    try:
        # Spin the node to handle callbacks and keep it alive
        # This will run until the node is shutdown or an exception occurs
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Handle Ctrl+C gracefully
        node.get_logger().info("Interrupted by user.")
    finally:
        # Ensure proper cleanup regardless of how the program exits
        node.destroy_node()  # Clean up node resources
        rclpy.shutdown()     # Shutdown ROS2 Python client library

# Standard Python idiom to run main() only when script is executed directly
if __name__ == '__main__':
    main()