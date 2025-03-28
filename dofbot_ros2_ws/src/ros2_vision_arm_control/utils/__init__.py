
__all__ = ['ikpy_utils', 'torch_utils']
URDF_PATH = "/home/jetson/code/dofbot_ros2_ws/src/ros2_vision_arm_control/urdf/dofbot.urdf"
WEIGHT_PATH = "/home/jetson/code/dofbot_ros2_ws/src/ros2_vision_arm_control/yolo_weight/yolov5n_0214.pt"
TOPIC_CAMERA_RGB = "/camera/rgb_image"
TOPIC_CAMERA_DEPTH = "/camera/depth_image"
YOLO_DETECTION = "/yolo_detection"
YOLO_DEPTH = "/yolo_depth"
ROBOT_STATUS = "/robot_status"
TEST_IMG_PATH = "/home/jetson/code/dofbot_ros2_ws/src/ros2_vision_arm_control/output0113_mov-0057.jpg"
VISUALIZATION = False
FRAME_RATE = 4
CLASS_NAMES = ['leaf','stem','tomato']