#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import math

class OdomConverter(Node):

    def __init__(self):
        super().__init__('odom_converter')
        
        self.subscription = self.create_subscription(
            Odometry,
            '/mobile_base_controller/odom',
            self.odom_callback,
            10)
        self.subscription
        
        self.get_logger().info("Nodo odom converter avviato")

    def quaternion_to_euler(self, x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z  # in radians

    def odom_callback(self, msg):
        # Estrazione posizione
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        
        # Estrazione orientamento
        orientation_q = msg.pose.pose.orientation
        roll, pitch, yaw = self.quaternion_to_euler(
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w)
        
        # Log delle informazioni
        self.get_logger().info('\nPosizione: x=%.2f m, y=%.2f m, z=%.2f m' % (x, y, z))
        self.get_logger().info('Orientamento (rad): roll=%.2f, pitch=%.2f, yaw=%.2f' % (roll, pitch, yaw))
        self.get_logger().info('Orientamento (deg): roll=%.2f°, pitch=%.2f°, yaw=%.2f°' % 
                              (math.degrees(roll), math.degrees(pitch), math.degrees(yaw)))

def main(args=None):
    rclpy.init(args=args)
    odom_converter = OdomConverter()
    rclpy.spin(odom_converter)
    odom_converter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()