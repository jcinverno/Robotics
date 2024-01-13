import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
import cv2
import numpy as np
import yaml
from image_geometry import PinholeCameraModel
import os
import ament_index_python
from cv_bridge import CvBridge 
import threading

from sensor_msgs.srv import SetCameraInfo

class CameraDriver(Node):

    def __init__(self):
        super().__init__('camera_driver_node')
        
        # Initialize CameraInfo to None
        self.camera_info = None
        self.camera_model = PinholeCameraModel()
        
        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()
        self.frame_id = '/base_link'
        self.rate = self.create_rate(10.0)

        video_path = os.path.join(
            ament_index_python.get_package_share_directory('trsa_lab1'), 'video','test.mov')

        # Create publishers and subscribers
        self.image_pub_raw = self.create_publisher(Image, '/camera/image_raw', 1)

        #Spin in a separate thread to avoid blocking on rate.sleep() due to ROS2 execution model
        self.thread = threading.Thread(target=rclpy.spin, args=(self, ), daemon=True)
        self.thread.start()
        self.cap = cv2.VideoCapture(video_path)

    def publish_frames_from_video(self):
        while rclpy.ok() and self.cap.isOpened():
            ret, frame = self.cap.read()
            if not ret:
                break

            ros_img_raw = self.br.cv2_to_imgmsg(frame, 'bgr8')
            ros_img_raw.header.frame_id = self.frame_id
            ros_img_raw.header.stamp = self.get_clock().now().to_msg()
            self.image_pub_raw.publish(ros_img_raw)

            self.rate.sleep()

        self.cap.release()
        self.get_logger().warn('Video publishing finished...')

def main(args=None):
    rclpy.init(args=args)
    node = CameraDriver()
    node.publish_frames_from_video()
    node.thread.join()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()