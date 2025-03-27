cmake_minimum_required(VERSION 3.8)
project(ros2_vision_arm_control)

# 查找 Python3 并启用 setuptools
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

find_package(ament_cmake_python REQUIRED)
ament_python_install_package("utils")
ament_python_install_package("models")

# 安装 Python 脚本
install(
    PROGRAMS
    __init__.py
    camera_node.py
    arm_control.py
    yolo_detector.py
    DESTINATION lib/${PROJECT_NAME}
)

# 安装 launch 文件
install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)

install(
    DIRECTORY meshes
    DESTINATION lib/${PROJECT_NAME}
)

install(
    DIRECTORY yolo_weight
    DESTINATION lib/${PROJECT_NAME}
)

install(
    DIRECTORY urdf
    DESTINATION lib/${PROJECT_NAME}
)


ament_package()
