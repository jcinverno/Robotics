import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from image_geometry import PinholeCameraModel
import cv2
import numpy as np
import sys
import os
import math

class MyImageProcessing(Node):
    def __init__(self):
        super().__init__('my_filter')
        self.subscription = self.create_subscription(
            Image,
            '/camera/my_image',
            self.image_callback,
            10
        )
        self.bridge = CvBridge()
        self.camera_model = PinholeCameraModel()
        self.frame_id = '/base_link'
        self.rate = self.create_rate(15.0)
        self.publisher = self.create_publisher(Image, '/camera/my_filter', 10)

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
        #Hough Transform
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 90, 100, apertureSize=3)
        lines = cv2.HoughLines(edges, 1, np.pi / 180, threshold=200)

        if lines is not None:
            for rho, theta in lines[:, 0]:
                a = np.cos(theta)
                b = np.sin(theta)
                x0 = a * rho
                y0 = b * rho
                x1 = int(x0 + 1000 * (-b))
                y1 = int(y0 + 1000 * (a))
                x2 = int(x0 - 1000 * (-b))
                y2 = int(y0 - 1000 * (a))
                cv2.line(cv_image, (x1, y1), (x2, y2), (0, 0, 255), 2)
        
        processed_image_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
        processed_image_msg.header = msg.header
        self.publisher.publish(processed_image_msg)

def main(args=None):
    rclpy.init(args=args)
    image_rectifier_node = MyImageProcessing()
    rclpy.spin(image_rectifier_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
