import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraPub(Node):
    def __init__(self):
        super().__init__('camera_pub')
        self.pub = self.create_publisher(Image, 'camera/image', 10)
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(0)  # 调用摄像头（0是默认摄像头）
        self.timer = self.create_timer(0.1, self.pub_img)  # 每0.1秒发一次图

    def pub_img(self):
        ret, frame = self.cap.read()
        if ret:
            img_msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
            self.pub.publish(img_msg)

def main(args=None):
    rclpy.init(args=args)
    cam_node = CameraPub()
    rclpy.spin(cam_node)
    cam_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
