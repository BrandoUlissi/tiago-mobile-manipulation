import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Empty, Bool
import time

IMAGE_WIDTH = 640
KP = 0.0035
ERROR_THRESHOLD = 75
TIMEOUT_DURATION = 0.5  # secondi

class RobotControllerNode(Node):
    def __init__(self):
        super().__init__('robot_controller_node')

        #############
        # SUBSCRIBERS
        #############

        self.bbox_center_subscriber = self.create_subscription(
            Point,
            '/truck_bounding_box_center',
            self.bbox_center_callback,
            10)

        #############
        # PUBLISHERS
        #############

        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.centered_publisher = self.create_publisher(Bool, '/target_centered', 10)
        
        ######
        # INIT
        ######

        # FLAGS
        self.stop_tracking = False
        
        # TIMERS
        self.last_update_time = time.time()
        self.timer = self.create_timer(0.1, self.check_timeout)  # Controlla ogni 100ms

        ###########
        # CALLBACKS
        ###########

    def bbox_center_callback(self, msg):
        self.last_update_time = time.time()  # Aggiorna il tempo dell'ultimo messaggio
        
        center_x = msg.x
        error_x = center_x - (IMAGE_WIDTH // 2)

        twist_msg = Twist()
        centered_msg = Bool()

        if not self.stop_tracking:
            if abs(error_x) > ERROR_THRESHOLD:
                self.get_logger().info("ERRORE, STO CORREGGENDO")
                twist_msg.angular.z = -KP * error_x
                twist_msg.linear.x = 0.1
                centered_msg.data = False
                self.cmd_vel_publisher.publish(twist_msg)
            else:
                self.get_logger().info("CORRETTO L'ERRORE: TRACKING CENTRATO")
                centered_msg.data = True
                self.centered_publisher.publish(centered_msg)
                #self.get_logger().info(f"Error: {error_x}, Centered: {centered_msg.data}, Tracking stopped")
                #self.stop_tracking = True

    def check_timeout(self):
        if (time.time() - self.last_update_time) > TIMEOUT_DURATION:
            centered_msg = Bool()
            centered_msg.data = False
            self.centered_publisher.publish(centered_msg)
           #self.stop_tracking = True
           #self.get_logger().info("Tracking lost - Centered set to False")

def main(args=None):
    rclpy.init(args=args)
    try:
        robot_controller_node = RobotControllerNode()
        rclpy.spin(robot_controller_node)
    except KeyboardInterrupt:
        pass
    finally:
        robot_controller_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
