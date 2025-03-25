cmake_minimum_required(VERSION 3.8)
project(ros2_vision_arm_control)

# 查找 Python3 并启用 setuptools
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

install(
    PROGRAMS
    camera_node.py
    arm_control.py
    yolo_detector.py
    DESTINATION lib/${PROJECT_NAME}
)

# 安装 launch 文件（如果有）
install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
