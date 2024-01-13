import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np

class ImageReader(Node):

    def __init__(self):
        super().__init__('image_reader')
        self.cv_bridge = CvBridge()

        # Create subscriptions for four different image topics
        self.subscription0 = self.create_subscription(Image, '/camera/image_raw', self.image_callback0, 10)
        self.subscription1 = self.create_subscription(Image, '/camera/image_rect', self.image_callback1, 10)
        self.subscription2 = self.create_subscription(Image, '/camera/image_processed', self.image_callback2, 10)
        self.subscription3 = self.create_subscription(Image, '/camera/my_filter', self.image_callback3, 10)

        self.cv_images = [None] * 4

    def image_callback0(self, msg):
        self.process_image(msg, 0)

    def image_callback1(self, msg):
        self.process_image(msg, 1)

    def image_callback2(self, msg):
        self.process_image(msg, 2)

    def image_callback3(self, msg):
        self.process_image(msg, 3)

    def process_image(self, msg, index):
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
            small_image = cv2.resize(cv_image, (320, 240))
            self.cv_images[index] = small_image
            self.display_images()

        except Exception as e:
            self.get_logger().error(f"Image processing error: {str(e)}")

    def display_images(self):
        default_image = np.zeros((240, 320, 3), dtype=np.uint8)

        for i in range(4):
            if self.cv_images[i] is None:
                self.cv_images[i] = default_image
                
        top_row = cv2.hconcat([self.cv_images[0], self.cv_images[1]])
        bottom_row = cv2.hconcat([self.cv_images[2], self.cv_images[3]])
        concatenated_image = cv2.vconcat([top_row, bottom_row])

        cv2.imshow("Image Viewer", concatenated_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    image_reader_node = ImageReader()
    rclpy.spin(image_reader_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
