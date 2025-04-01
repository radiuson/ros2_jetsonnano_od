import numpy as np
__all__ = ['ikpy_utils', 'torch_utils']
URDF_PATH = "/home/jetson/code/dofbot_ros2_ws/src/ros2_vision_arm_control/urdf/dofbot.urdf"
WEIGHT_PATH = "/home/jetson/code/dofbot_ros2_ws/src/ros2_vision_arm_control/yolo_weight/yolov5t_0401.pt"
TOPIC_CAMERA_RGB = "/camera/rgb_image"
TOPIC_CAMERA_DEPTH = "/camera/depth_image"
TOPIC_YOLO_DETECTION = "/yolo_detection"
TOPIC_YOLO_DEPTH = "/yolo_depth"
TOPIC_ROBOT_STATUS = "/robot_status"
TOPIC_ROBOT_TRANSFORM = "/robot_transform"
TOPIC_ARM_CONTROL = "/arm_control"
TOPIC_CAMERA_INFO = "/camera_info"
TEST_IMG_PATH = "/home/jetson/code/dofbot_ros2_ws/src/ros2_vision_arm_control/output0113_mov-0057.jpg"
VISUALIZATION = False
FRAME_RATE = 1
CAMERA_MOUNT_INDEX=4
JOINT_INTERVAL = 0.5
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
CLASS_NAMES = ['leaf','stem','tomato']

MOUNT_TO_CAMERA_OFFSET = np.array([ # Camera mount to camera offset
    [1, 0, 0,  -0.06],  # y 偏移 6cm
    [0, 1, 0,  0.05],   # z 偏移 5cm
    [0, 0, 1,  -0.03],  # x 偏移 3cm
    [0, 0, 0,  1]
])