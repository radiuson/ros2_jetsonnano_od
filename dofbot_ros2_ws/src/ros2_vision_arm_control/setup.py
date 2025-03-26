from setuptools import setup
from setuptools import find_packages

setup(
    name='ros2_vision_arm_control',
    version='0.0.1',
    packages=find_packages(),
    package_dir={'': '.'},  # 让 Python 识别 `utils`
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Runqian zhang',
    maintainer_email='runqianzhang@imzrq.cn',
    description='ROS 2 package for vision-based robotic arm control',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arm_control = ros2_vision_arm_control.arm_control:main',
            'camera_node = ros2_vision_arm_control.camera_node:main',
        ],
    },
)