import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
import math

class ApproachNode(Node):
    def __init__(self):
        super().__init__('approach_node')

        qos_profile_high = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.position = None
        self.orientation = None
        self.is_centered = False
        self.distanza = 0.0
        self.SECURITY_FACTOR = 0.0 # in meters

        self.timer_period = 0.01
        self.routine_timer = self.create_timer(self.timer_period, self.run_routine)

        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.subscription = self.create_subscription(
            Odometry,
            '/mobile_base_controller/odom',
            self.odom_callback,
            10
        )

        self.distance_subscription = self.create_subscription(
            Float32,
            '/distanza_obiettivo',
            self.distance_callback,
            10
        )

        self.centered_subscription = self.create_subscription(
            Bool,
            '/target_centered',
            self.centered_callback,
            10)

        self.goal_pose_publisher = self.create_publisher(
            PoseStamped, '/goal_pose', qos_profile_high)

    def centered_callback(self, msg):
        self.is_centered = msg.data

    def euler_from_quaternion(self, x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.degrees(math.atan2(t0, t1))

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.degrees(math.asin(t2))

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.degrees(math.atan2(t3, t4))

        return roll_x, pitch_y, yaw_z

    def distance_callback(self, msg):
        self.distanza = msg.data

    def odom_callback(self, msg):
        self.position = msg.pose.pose.position
        self.orientation = msg.pose.pose.orientation
        self.roll, self.pitch, self.yaw = self.euler_from_quaternion(
            self.orientation.x, self.orientation.y, self.orientation.z, self.orientation.w
        )

    def run_routine(self):
        if self.distanza and self.position and self.is_centered:
            self.get_logger().info("DISTANCE MEASUREMENT RECEIVED")

            safe_distance = max(0, self.distanza - self.SECURITY_FACTOR)

            new_x = self.position.x + safe_distance * math.cos(math.radians(self.yaw))
            new_y = self.position.y + safe_distance * math.sin(math.radians(self.yaw))

            goal_msg = PoseStamped()
            goal_msg.header.frame_id = "map"
            goal_msg.header.stamp = self.get_clock().now().to_msg()
            goal_msg.pose.position.x = new_x
            goal_msg.pose.position.y = new_y
            goal_msg.pose.orientation = self.orientation

            self.goal_pose_publisher.publish(goal_msg)
            self.get_logger().info(f"Published waypoint at: X={new_x}, Y={new_y}")

            # Ripristino flag per consentire nuovi waypoint
            self.is_centered = False




def main(args=None):
    rclpy.init(args=args)
    node = ApproachNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()