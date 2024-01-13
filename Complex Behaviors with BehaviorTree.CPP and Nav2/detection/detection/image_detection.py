import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge
from image_geometry import PinholeCameraModel
import cv2
import numpy as np

class ImageDetection(Node):

    def __init__(self):
        super().__init__('image_detect_node')
        self.subscription = self.create_subscription(
            Image,
            'intel_realsense_r200_depth/image_raw',
            self.image_callback,
            10
        )

        self.bridge = CvBridge()
        self.camera_model = PinholeCameraModel()
        self.frame_id = '/base_link'
        self.rate = self.create_rate(5.0)

        self.publisher = self.create_publisher(Bool, '/cone_detection', 10)

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # Perform object detection
        traffic_cone_detected = self.identify_traffic_cone(cv_image)

        # Publish the boolean topic
        self.publisher.publish(traffic_cone_detected)

        # Print the boolean value in the console
        self.get_logger().info(f'Traffic Cone Detected: {traffic_cone_detected.data}')

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

        # Check if any contours are found
        traffic_cone_detected = Bool()
        traffic_cone_detected.data = bool(contours)

        return traffic_cone_detected

def main(args=None):
    rclpy.init(args=args)
    image_detection_node = ImageDetection()
    rclpy.spin(image_detection_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
