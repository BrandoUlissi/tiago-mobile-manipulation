import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2
import torch
import warnings
warnings.filterwarnings("ignore")

class YOLONode(Node):
    def __init__(self):
        super().__init__('yolo_node')

        self.subscription = self.create_subscription(
            Image,
            '/head_front_camera/rgb/image_raw',
            self.image_callback,
            10)
        
        self.bridge = CvBridge()
        self.model = torch.hub.load('ultralytics/yolov5', 'yolov5s')
        
        self.bbox_center_publisher = self.create_publisher(Point, '/truck_bounding_box_center', 10)
        self.tracking_status_publisher = self.create_publisher(Bool, '/truck_tracking_status', 10)

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        results = self.model(cv_image)

        truck_box = None
        truck_label = None

        for *box, conf, cls in results.xyxy[0]:
            class_id = int(cls)
            class_name = self.model.names[class_id]

            x_min, y_min, x_max, y_max = map(int, box)

            # Disegna bounding box per tutti gli oggetti riconosciuti
            cv2.rectangle(cv_image, (x_min, y_min), (x_max, y_max), (255, 0, 0), 2)
            cv2.putText(cv_image, class_name, (x_min, y_min - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

            if class_name == "truck" or class_name == "car" or class_name == "bus" or class_name == "microwave" or class_name == "traffic light" or class_name == "tv":
                truck_box = (x_min, y_min, x_max, y_max)
                truck_label = "vehicle"

        tracking_status_msg = Bool()

        if truck_box:
            tracking_status_msg.data = True
            x_min, y_min, x_max, y_max = truck_box
            center_x = (x_min + x_max) // 2
            center_y = (y_min + y_max) // 2

            bbox_center_msg = Point()
            bbox_center_msg.x = float(center_x)
            bbox_center_msg.y = float(center_y)
            bbox_center_msg.z = 0.0
            self.bbox_center_publisher.publish(bbox_center_msg)

            cv2.rectangle(cv_image, (x_min, y_min), (x_max, y_max), (0, 255, 0), 4)

            shadow_offset = 5
            cv2.rectangle(cv_image, (x_min + shadow_offset, y_min + shadow_offset),
                          (x_max + shadow_offset, y_max + shadow_offset), (0, 0, 0), 4)

            cv2.circle(cv_image, (center_x, center_y), 8, (0, 0, 255), -1)

            label = f"{truck_label} ({center_x}, {center_y})"
            (label_width, label_height), baseline = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
            cv2.rectangle(cv_image, (x_min, y_min - label_height - 10),
                          (x_min + label_width, y_min - 10), (0, 255, 0), -1)
            cv2.putText(cv_image, label, (x_min, y_min - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

            cv2.line(cv_image, (center_x, y_min), (center_x, y_max), (0, 0, 255), 2, cv2.LINE_AA)
            cv2.line(cv_image, (x_min, center_y), (x_max, center_y), (0, 0, 255), 2, cv2.LINE_AA)

        else:
            tracking_status_msg.data = False

        self.tracking_status_publisher.publish(tracking_status_msg)

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