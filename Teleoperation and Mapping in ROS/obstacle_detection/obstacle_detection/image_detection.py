#! usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from image_geometry import PinholeCameraModel
import cv2
import numpy as np

class ImageDetection(Node):

    def __init__(self):
        super().__init__('image_detection')
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

        self.publisher = self.create_publisher(Image, '/depth_camera/image_detect', 10)


    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # Perform object detection
        identify_traffic_cone = self.identify_traffic_cone(cv_image)

        processed_image_msg = self.bridge.cv2_to_imgmsg(identify_traffic_cone, "bgr8")
        processed_image_msg.header = msg.header
        self.publisher.publish(processed_image_msg)

    def identify_traffic_cone(self, cv_image):

        # Convert the image to the HSV color space
        img_hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Define the lower and upper bounds of the orange color
        lower_orange = np.array([5, 100, 100])
        upper_orange = np.array([15, 255, 255])

        # Create a mask using the inRange function
        mask = cv2.inRange(img_hsv, lower_orange, upper_orange)

        # Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        identified_areas = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if 2500 > area > 100:
                identified_areas.append(contour)

        # Draw a single rectangle around all identified areas
        if identified_areas:
            x, y, w, h = cv2.boundingRect(np.vstack(identified_areas))
            cv2.rectangle(cv_image, (x, y), (x + w, y + h), color=(0, 255, 0), thickness=2)

        return cv_image



def main(args=None):
    rclpy.init(args=args)
    image_detection_node = ImageDetection()
    rclpy.spin(image_detection_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()