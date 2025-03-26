from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_vision_arm_control',
            executable='arm_control.py',
            name='arm_control',
            output='screen'
        ),
        Node(
            package='ros2_vision_arm_control',
            executable='camera_node.py',
            name='camera_node',
            output='screen'
        ),
        Node(
            package='ros2_vision_arm_control',
            executable='yolo_detector.py',
            name='yolo_detector',
            output='screen'
        ),
    ])
