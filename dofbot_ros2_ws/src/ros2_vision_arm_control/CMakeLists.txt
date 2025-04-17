cmake_minimum_required(VERSION 3.8)
project(ros2_vision_arm_control)

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED) 
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_typesupport_fastrtps_c REQUIRED)
find_package(rosidl_typesupport_fastrtps_cpp REQUIRED)

# 生成消息接口
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/VisionDetection.msg"
  "msg/BoundingBox.msg"
  DEPENDENCIES builtin_interfaces sensor_msgs std_msgs
)

# 安装 Python 脚本
ament_python_install_package(models)
ament_python_install_package(utils)


install(
  PROGRAMS
    __init__.py
    camera_node.py
    arm1_control.py
    yolo_detector.py
    motion_planner.py
  DESTINATION lib/${PROJECT_NAME}
)

# 安装目录
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY meshes DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY yolo_weight DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY urdf DESTINATION lib/${PROJECT_NAME})

# 导出 Python 消息支持（关键）
ament_export_dependencies(rosidl_default_runtime)

ament_package()