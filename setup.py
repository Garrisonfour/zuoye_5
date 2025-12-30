from setuptools import setup
import os
from glob import glob

package_name = 'color_detection'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 配置launch文件的安装路径（若有）
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wyj',
    maintainer_email='2174609416l@qq.com',
    description='ROS2 Humble色块识别作业',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 节点入口：格式为「执行命令 = 包名.文件名:主函数」
            'camera_pub = color_detection.camera_pub:main',
            'color_detect = color_detection.color_detect:main',
        ],
    },
)
