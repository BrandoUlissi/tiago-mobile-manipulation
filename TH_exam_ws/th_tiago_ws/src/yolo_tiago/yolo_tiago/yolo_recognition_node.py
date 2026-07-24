import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import torch

class YOLONode(Node):
    def __init__(self):
        super().__init__('yolo_node')
        self.subscription = self.create_subscription(
            Image,
            '/head_front_camera/rgb/image_raw',  # Topic dell'immagine della camera
            self.image_callback,
            10)
        self.bridge = CvBridge()
        self.model = torch.hub.load('ultralytics/yolov5', 'yolov5s')  # Carica il modello YOLOv5

    def image_callback(self, msg):
        # Converti l'immagine ROS in formato OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Esegui YOLO sull'immagine
        results = self.model(cv_image)

        # Visualizza i risultati
        results.render()  # Aggiunge le bounding box all'immagine
        cv2.imshow('YOLO', cv_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    yolo_node = YOLONode()
    rclpy.spin(yolo_node)
    yolo_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()