from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 配置摄像头发布节点
    camera_node = Node(
        package='color_detection',
        executable='camera_pub',
        name='camera_publisher',
        output='screen'
    )

    # 配置色块识别节点
    detect_node = Node(
        package='color_detection',
        executable='color_detect',
        name='color_detector',
        output='screen'
    )

    # 组合节点并返回启动描述
    return LaunchDescription([camera_node, detect_node])
