import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
import yaml
from image_geometry import PinholeCameraModel
import cv2


class ImageProcessing(Node):

    def __init__(self):
        super().__init__('image_processing')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_rect',
            self.image_callback,
            10
        )

        self.bridge = CvBridge()
        self.camera_model = PinholeCameraModel()
        self.frame_id = '/base_link'
        self.rate = self.create_rate(15.0)

        self.publisher = self.create_publisher(Image, '/camera/image_processed', 10)
        
    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        # Convert the BGR image to grayscale
        gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        # Apply Gaussian blur with a 21x21 kernel and zero standard deviation
        blurred_image = cv2.GaussianBlur(gray_image, (21, 21), 0)
        # Apply Canny Edge Detection
        edges_image = cv2.Canny(blurred_image, threshold1=10, threshold2=50)  # You can adjust the thresholds

        processed_image_msg = self.bridge.cv2_to_imgmsg(edges_image, "mono8")
        processed_image_msg.header = msg.header
        self.publisher.publish(processed_image_msg)


def main(args=None):
    rclpy.init(args=args)
    image_rectifier_node = ImageProcessing()
    rclpy.spin(image_rectifier_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
