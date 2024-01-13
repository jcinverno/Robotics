import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import ament_index_python
import os
from sensor_msgs.msg import Image, CameraInfo
import yaml
from image_geometry import PinholeCameraModel
import cv2


class ImageRectifier(Node):

    def __init__(self):
        super().__init__('image_rectifier')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        self.bridge = CvBridge()
        self.camera_info = None
        self.camera_model = PinholeCameraModel()
        self.frame_id = '/base_link'
        self.rate = self.create_rate(15.0)

        self.publisher = self.create_publisher(Image, '/camera/image_rect', 10)
        
        self.calibration_path = os.path.join( ament_index_python.get_package_share_directory('trsa_lab1'), 'calibration', 'ost.yaml')
        
    def image_callback(self, msg):
        if self.camera_info is not None:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            rectified_image = cv_image.copy()
            self.camera_model.fromCameraInfo(self.camera_info)
            self.camera_model.rectifyImage(cv_image, rectified_image)
            rectified_image_msg = self.bridge.cv2_to_imgmsg(rectified_image, "bgr8")
            rectified_image_msg.header = msg.header
            self.publisher.publish(rectified_image_msg)

    def load_camera_calibration(self):
        if os.path.isfile(self.calibration_path):
            with open(self.calibration_path, 'r') as file:
                calibration_data = yaml.safe_load(file)
                self.camera_info = CameraInfo()
                self.camera_info.width = calibration_data['image_width']
                self.camera_info.height = calibration_data['image_height']
                self.camera_info.distortion_model = calibration_data['distortion_model']
                self.camera_info.d = calibration_data['distortion_coefficients']['data']
                self.camera_info.k = calibration_data['camera_matrix']['data']
                self.camera_info.r = calibration_data['rectification_matrix']['data']
                self.camera_info.p = calibration_data['projection_matrix']['data']
                self.get_logger().info('Camera calibration loaded')
        else:
            self.get_logger().error(f'Calibration file not found: {self.calibration_path}')

def main(args=None):
    rclpy.init(args=args)
    image_rectifier_node = ImageRectifier()
    image_rectifier_node.load_camera_calibration()
    rclpy.spin(image_rectifier_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
