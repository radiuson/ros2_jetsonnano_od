from setuptools import setup

package_name = 'ros2_vision_arm_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=[
        'setuptools',
        'opencv-python',
        'torch',
        'torchvision',
        'pyrealsense2',
        'numpy',
        'matplotlib',
        'rclpy',
        'sensor_msgs',
        'cv_bridge',
        'ikpy',
    ],
    zip_safe=True,
    maintainer='Runqian Zhang',
    maintainer_email='runqianzhang@imzrq.cn',
    description='A ROS2 package for controlling a robotic arm with YOLO, .',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_node = ros2_vision_arm_control.camera_node:main',
            'arm_control = ros2_vision_arm_control.arm_control:main',
        ],
    },
)