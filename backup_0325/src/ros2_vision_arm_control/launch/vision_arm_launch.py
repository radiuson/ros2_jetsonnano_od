from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_vision_arm_control',
            executable='camera_node',
            name='camera_node',
            output='screen'
        ),
        Node(
            package='ros2_vision_arm_control',
            executable='arm_control',
            name='arm_control',
            output='screen'
        ),
        Node(
            package='ros2_vision_arm_control',
            executable='yolo_detector',
            name='yolo_detector',
            output='screen'
        ),
    ])
