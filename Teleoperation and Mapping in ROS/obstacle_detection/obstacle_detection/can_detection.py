#! usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from image_geometry import PinholeCameraModel
import cv2
import numpy as np

class CanDetection(Node):

    def __init__(self):
        super().__init__('can_detection')
        self.subscription = self.create_subscription(
            Image,
            '/depth_camera/image_raw',
            self.image_callback,
            10
        )

        self.bridge = CvBridge()
        self.camera_model = PinholeCameraModel()
        self.frame_id = '/base_link'
        self.rate = self.create_rate(15.0)

        self.publisher = self.create_publisher(Image, '/depth_camera/can_detect', 10)


    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # Perform object detection
        detect_red_cans = self.detect_red_cans(cv_image)

        processed_image_msg = self.bridge.cv2_to_imgmsg(detect_red_cans, "bgr8")
        processed_image_msg.header = msg.header
        self.publisher.publish(processed_image_msg)

    def detect_red_cans(self, image):
        min_contour_area_threshold = 50
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        lower_red = np.array([0, 50, 50])
        upper_red = np.array([10, 255, 255])

        mask = cv2.inRange(hsv, lower_red, upper_red)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            area = cv2.contourArea(contour)
            if area > min_contour_area_threshold:
                x, y, w, h = cv2.boundingRect(contour)
                cv2.rectangle(image, (x, y), (x + w, y + h), (255, 0, 0), 2)

        return image


def main(args=None):
    rclpy.init(args=args)
    can_detection_node = CanDetection()
    rclpy.spin(can_detection_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()