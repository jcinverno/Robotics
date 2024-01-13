import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

class MyCamera(Node):
    def __init__(self):
        super().__init__('my_camera')
        
        self.br = CvBridge()
        self.image_pub_raw = self.create_publisher(Image, '/camera/my_image', 10)

    def publish_frames_from_video(self):
        cap = cv2.VideoCapture(0)  
        while rclpy.ok():
            ret, frame = cap.read()
            if not ret:
                self.get_logger().error("Error in OpenCV")
            else:
                ros_img_raw = self.br.cv2_to_imgmsg(frame, 'bgr8')
                self.image_pub_raw.publish(ros_img_raw)

def main(args=None):
    rclpy.init(args=args)
    node = MyCamera()
    node.publish_frames_from_video()
    rclpy.spin(node)  
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
