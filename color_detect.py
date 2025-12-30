import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ColorDetect(Node):
    def __init__(self):
        super().__init__('color_detect')
        self.sub = self.create_subscription(Image, 'camera/image', self.detect_color, 10)
        self.bridge = CvBridge()

    def detect_color(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')  # 把ROS图像转成OpenCV格式

        # 方法1：识别蓝色（HSV阈值法）
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_blue = np.array([90, 120, 80])
        upper_blue = np.array([130, 255, 255])
        blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)
        blue = cv2.bitwise_and(frame, frame, mask=blue_mask)

        # 方法2：识别红色（轮廓法）
        lower_red1 = np.array([0, 120, 80])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 120, 80])
        upper_red2 = np.array([180, 255, 255])
        red_mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        red_mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        red_mask = cv2.bitwise_or(red_mask1, red_mask2)
        contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        red = frame.copy()
        if contours:
            for cnt in contours:
                if cv2.contourArea(cnt) > 100:
                    cv2.drawContours(red, [cnt], -1, (0, 255, 0), 2)

        # 显示结果
        cv2.imshow('Blue Detection', blue)
        cv2.imshow('Red Detection', red)
        cv2.waitKey(30)

def main(args=None):
    rclpy.init(args=args)
    detect_node = ColorDetect()
    try:
        rclpy.spin(detect_node)
    except KeyboardInterrupt:
        pass
    detect_node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
