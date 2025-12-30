# ROS2 Humble 色块识别项目
本项目基于 ROS2 Humble（谦逊的玳瑁）实现**摄像头调用**与**红/蓝色块阈值识别**，通过 ROS2 节点通信完成图像的发布与色块检测，适用于机器人视觉入门实践。


## 一、项目架构
包含 2 个核心 ROS2 节点，通过话题通信实现功能：
1. **camera_publisher（摄像头发布节点）**：
   - 调用本地摄像头，采集实时图像；
   - 发布图像话题：`/camera/image_raw`（消息类型：`sensor_msgs/Image`）。

2. **color_detector（色块检测节点）**：
   - 订阅 `camera/image_raw` 话题，获取实时图像；
   - 通过 **HSV 阈值**识别红色块、**BGR 阈值**识别蓝色块；
   - 输出阈值分割后的掩码图像（可直观查看识别效果）。


## 二、环境依赖
| 依赖项          | 版本/说明                     |
|-----------------|------------------------------|
| 操作系统        | Ubuntu 22.04 LTS             |
| ROS2 版本       | Humble Hawksbill（谦逊的玳瑁）|
| 依赖库          | rclpy、sensor_msgs、cv_bridge、image_transport、python3-opencv |


## 三、快速运行
### 1. 编译工作空间
```bash
# 进入工作空间
cd ~/color_ws
# 编译指定功能包
colcon build --packages-select color_detection
# 加载环境变量
source install/setup.bash
```

### 2. 启动节点
- **启动摄像头发布节点**（新开终端）：
  ```bash
  source ~/color_ws/install/setup.bash
  ros2 run color_detection camera_pub
  ```

- **启动色块检测节点**（再开终端）：
  ```bash
  source ~/color_ws/install/setup.bash
  ros2 run color_detection color_det
  ```


## 四、参数调整（优化识别效果）
若色块识别不够精准，可修改 `color_det.py` 中的阈值参数：
- **红色块（HSV 阈值）**：
  ```python
  # 红色HSV阈值（可根据实际场景微调）
  lower_red1 = np.array([0, 150, 100])
  upper_red1 = np.array([8, 255, 255])
  lower_red2 = np.array([172, 150, 100])
  upper_red2 = np.array([180, 255, 255])
  ```
- **蓝色块（BGR 阈值）**：
  ```python
  # 蓝色BGR阈值（可根据实际场景微调）
  lower_blue = np.array([100, 50, 50])
  upper_blue = np.array([140, 255, 255])
  ```


## 五、话题列表
| 话题名称          | 消息类型          | 说明                     |
|-------------------|-------------------|--------------------------|
| `/camera/image_raw` | `sensor_msgs/Image` | 摄像头原始图像话题       |
| `/color_detection/red_mask` | `sensor_msgs/Image` | 红色块识别后的掩码图像   |
| `/color_detection/blue_mask` | `sensor_msgs/Image` | 蓝色块识别后的掩码图像   |

