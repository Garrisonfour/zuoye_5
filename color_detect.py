#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ColorDetector(Node):
    """识别节点：订阅图像并实现红、蓝色块阈值识别"""
    def __init__(self):
        super().__init__('color_detector')
        # 订阅摄像头发布的图像话题
        self.image_subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        self.bridge = CvBridge()

    def image_callback(self, msg):
        """图像回调：处理图像并识别色块"""
        # 将ROS2 Image消息转为OpenCV的BGR图像
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # 方法1：HSV色彩空间阈值分割识别红色块（跨0度的双区间）
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_red1 = np.array([0, 120, 70])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 120, 70])
        upper_red2 = np.array([180, 255, 255])
        red_mask1 = cv2.inRange(hsv_frame, lower_red1, upper_red1)
        red_mask2 = cv2.inRange(hsv_frame, lower_red2, upper_red2)
        red_mask = red_mask1 | red_mask2  # 合并红色掩码

        # 方法2：BGR色彩空间阈值分割识别蓝色块（直接基于通道值）
        lower_blue = np.array([100, 0, 0])   # B通道高
        upper_blue = np.array([255, 100, 100])  # G、R通道低
        blue_mask = cv2.inRange(frame, lower_blue, upper_blue)

        # 显示阈值结果（对应作业中的Threshold效果）
        cv2.imshow('Original Frame', frame)
        cv2.imshow('Red Threshold', red_mask)
        cv2.imshow('Blue Threshold', blue_mask)
        cv2.waitKey(1)  # 刷新窗口，1ms延迟

def main(args=None):
    """主函数：初始化节点并运行"""
    rclpy.init(args=args)
    detect_node = ColorDetector()
    rclpy.spin(detect_node)
    # 销毁节点并释放窗口
    detect_node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
