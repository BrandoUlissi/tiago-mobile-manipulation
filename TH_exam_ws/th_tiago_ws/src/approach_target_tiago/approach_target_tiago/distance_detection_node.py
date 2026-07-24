import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import numpy as np
import struct  # Per decodificare i dati binari
from std_msgs.msg import Float32, Bool

class PointCloudSubscriber(Node):
    
    def __init__(self):
        super().__init__('pointcloud_subscriber')

        #############
        # SUBSCRIBERS
        #############

        self.pointcloud_subscription = self.create_subscription(
            PointCloud2,
            '/head_front_camera/depth_registered/points',
            self.pointcloud_callback,
            10)

        self.centered_subscription = self.create_subscription(
            Bool,
            '/target_centered',
            self.centered_callback,
            10)

        #############
        # PUBLISHERS
        #############

        self.distance_publisher = self.create_publisher(Float32, '/distanza_obiettivo', 10)

        ######
        # INIT
        ######
        
        # FLAGS
        self.is_centered = False
        
        # VARIABLES
        self.distanza = 0.0

    ###########
    # CALLBACKS
    ###########

    def centered_callback(self, msg):
        self.is_centered = msg.data

    def pointcloud_callback(self, msg):
        data = np.frombuffer(msg.data, dtype=np.float32).reshape((msg.height, msg.width, -1))

        centro_x = msg.width // 2
        centro_y = msg.height // 2

        point = data[centro_y, centro_x]
        x, y, z = point[0], point[1], point[2]

        self.distanza = np.sqrt(x**2 + y**2 + z**2)

        if not self.is_centered:
            return  # Esci direttamente se l'obiettivo non è centrato

        distanza_msg = Float32()
        distanza_msg.data = self.distanza
        self.distance_publisher.publish(distanza_msg)

        
def main(args=None):
    rclpy.init(args=args)
    node = PointCloudSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
