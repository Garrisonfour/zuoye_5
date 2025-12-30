# ROS2 Humble 色块识别作业
本项目基于ROS2 Humble实现摄像头调用与红、蓝色块的阈值识别，包含两个核心节点：摄像头发布节点与色块识别节点。

## 功能说明
1. **camera_publisher**：调用本地摄像头，发布图像话题`/camera/image_raw`；
2. **color_detector**：订阅图像话题，通过HSV阈值识别红色块、BGR阈值识别蓝色块，输出阈值分割后的掩码效果。

## 环境依赖
- 系统：Ubuntu 22.04
- ROS2版本：Humble Hawksbill
- 依赖库：`rclpy`、`sensor_msgs`、`cv_bridge`、`image_transport`、`python3-opencv`

## 运行步骤
1. 编译工作空间：
   ```bash
   cd ~/color_ws
   colcon build --packages-select color_detection
   source install/setup.bash
