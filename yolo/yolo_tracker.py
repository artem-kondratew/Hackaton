import rclpy
import supervision as sv
import cv2
import numpy as np

from ultralytics import YOLO
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


# wheights for NN
MODEL = YOLO('weights/yolov8s.pt')

bridge = CvBridge()

class Camera_Subscriber(Node):

    def __init__(self):

        super().__init__('camera_subscriber')

        self.subscription = self.create_subscription(
            Image,
            'TODO: Gera should name a topic',
            self.camera_callback,
            10)
        self.subscription

        self.img_publisher = self.create_publisher(Image, "/yolo_tracker", 10)

    def camera_callback(self, data):
        frame = bridge.imgmsg_to_cv2(data, "bgr8")
        tracker = human_tracker(frame)
        img_msg = bridge.cv2_to_imgmsg(tracker)
        self.img_publisher.publish(img_msg)

    
def human_tracker(self, frame):
    
    results = MODEL(frame)
    frame_ = results[0].plot()
        
    boxes = results[0].boxes.xyxy.tolist()
    boxes = boxes[0]
    
    return frame_


def main(args=None):

    rclpy.init(args=args)
    camera_subscriber = Camera_Subscriber()
    rclpy.spin(camera_subscriber)
    rclpy.shutdown()


if __name__ == '__main__':
    main()