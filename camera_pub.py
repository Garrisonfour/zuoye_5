#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraPublisher(Node):
    """摄像头节点：调用摄像头并发布图像话题"""
    def __init__(self):
        super().__init__('camera_publisher')
        # 创建图像发布者，话题为/camera/image_raw，队列大小10
        self.image_publisher = self.create_publisher(Image, '/camera/image_raw', 10)
        # 设置定时器，30Hz发布图像（0.03秒一次）
        self.timer = self.create_timer(0.03, self.timer_callback)
        # 初始化CVBridge，实现OpenCV与ROS图像格式转换
        self.bridge = CvBridge()
        # 打开默认摄像头（设备号0）
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("无法打开摄像头！请检查设备连接")
            rclpy.shutdown()

    def timer_callback(self):
        """定时器回调：读取摄像头帧并发布"""
        ret, frame = self.cap.read()
        if ret:
            # 将OpenCV的BGR图像转为ROS2的Image消息
            ros_image = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.image_publisher.publish(ros_image)

    def destroy_node(self):
        """节点销毁时释放摄像头资源"""
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    """主函数：初始化节点并运行"""
    rclpy.init(args=args)
    camera_node = CameraPublisher()
    rclpy.spin(camera_node)
    camera_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
