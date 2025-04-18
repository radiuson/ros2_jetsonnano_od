import numpy as np
import pyrealsense2 as rs
from utils.ikpy_utils import util_ikpy_d2r,util_ikpy_r2d
__all__ = ['ikpy_utils', 'torch_utils']
URDF_PATH = "/home/jetson/code/dofbot_ros2_ws/src/ros2_vision_arm_control/urdf/dofbot.urdf"
WEIGHT_PATH = "/home/jetson/code/dofbot_ros2_ws/src/ros2_vision_arm_control/yolo_weight/yolov5t_0418.pt"
TOPIC_CAMERA_RGB = "/camera/rgb_image"
TOPIC_CAMERA_DEPTH = "/camera/depth_image"
TOPIC_YOLO_DETECTION = "/yolo_detection"
TOPIC_YOLO_DEPTH = "/yolo_depth"
TOPIC_ROBOT1_STATUS = "/robot1_status"
TOPIC_ROBOT2_STATUS = "/robot2_status"
TOPIC_ROBOT1_TRANSFORM = "/robot1_transform"
TOPIC_ROBOT2_TRANSFORM = "/robot2_transform"
TOPIC_ARM1_CONTROL = "/arm1_control"
TOPIC_ARM2_CONTROL = "/arm2_control"
TOPIC_CAMERA_INFO = "/camera_info"
TRIGGER_CAMERA_INFO = '/generate_camera_info'
TRIGGER_YOLO_DEPTH = '/publish_depth_image'
TOPIC_YOLO_RESULT = '/yolo_result'
TOPIC_CAMERA_MSG = '/camera/rgb_depth_detection_msg'
TEST_IMG_PATH = "/home/jetson/code/dofbot_ros2_ws/src/ros2_vision_arm_control/output0113_mov-0057.jpg"
VISUALIZATION = True
ARM_COMPEN = True
FRAME_RATE = 1
CAMERA_MOUNT_INDEX=4
JOINT_INTERVAL = 0.5
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
DEPTH_IMAGE_SCALE = 1000
CLASS_NAMES = ['leaf','tomato']
DISTORTION_MAPPING = {
    'brown_conrady': rs.distortion.brown_conrady,
    'modified_brown_conrady': rs.distortion.modified_brown_conrady,
    'inverse_brown_conrady': rs.distortion.inverse_brown_conrady,
    'none': rs.distortion.none,
    'ftheta': rs.distortion.ftheta,
}

MOUNT_TO_CAMERA_OFFSET = np.array([ # Camera mount to camera offset
    [0, 0, -1,  -0.06],  
    [0, -1, 0,  0.05],   
    [1, 0, 0,  -0.03],  
    [0, 0, 0,  1]
    # [0, 0, -1,  0],  
    # [0, -1, 0,  0],   
    # [1, 0, 0,  0],  
    # [0, 0, 0,  1]
])

ROTATION_INTERIOR = np.array([
    [1, 0, 0],
    [0, 1, 0],
    [0, 0, 1]
])
INITIAL_POSITION = util_ikpy_d2r([90,180,18,0,90,90])