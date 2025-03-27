#!/usr/bin/env python3
from collections import defaultdict
import json
import cv2
import numpy as np
import torch
from models.experimental import attempt_load
from utils.general import non_max_suppression, scale_coords
from utils.torch_utils import select_device
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from utils import WEIGHT_PATH,TOPIC_CAMERA_RGB,VISUALIZATION,CLASS_NAMES

class YoloDetector(Node):
    def __init__(self, model_path=WEIGHT_PATH, device='',visualization=VISUALIZATION):
        super().__init__('yolo_detector_node')
        self.device = select_device(device)
        self.model = self.load_model(model_path,device=self.device)
        self.bridge = CvBridge()
        self.visualization = visualization
        self.subscription = self.create_subscription(
            Image,
            TOPIC_CAMERA_RGB,  
            self.image_callback,
            10
        )
        self.publisher = self.create_publisher(String, 'yolo_detections', 10)
        self.get_logger().info("YoloDetector Node Initialized")
        # 颜色字典（用 defaultdict 生成新类别的随机颜色）
        self.class_colors = defaultdict(lambda: tuple(np.random.randint(0, 256, 3).tolist()))

    def load_model(self, model_path,device):
        self.get_logger().info("Start loading weight...")
        model = attempt_load(model_path,map_location=device).float()  # load to FP32
        model.eval()
        self.get_logger().info("Model Loaded")
        return model

    def image_callback(self, msg):
        self.get_logger().info("Received an image")
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            detections = self.detect(cv_image)

            pred_message = self.format_detections(detections)
            self.publisher.publish(pred_message)
            self.get_logger().info(f"Published detections")

            if self.visualization:
                processed_image = self.draw_detections(cv_image, detections, names=CLASS_NAMES)  # 替换为实际类别
                # 显示检测结果
                cv2.imshow("YOLO Detection", processed_image)
                cv2.waitKey(1)  # 必须调用以刷新窗口

            # 可在此处发布处理后的图片或其他操作
        except Exception as e:
            self.get_logger().error(f"Failed to process image: {e}")

    def detect(self, img, conf_threshold=0.3, iou_threshold=0.45):
        img = self.preprocess_image(img)
        with torch.no_grad():
            pred, *_ = self.model(img)
            pred = non_max_suppression(pred, conf_threshold, iou_threshold)
            self.get_logger().info(f"Detected {len(pred)} objects")

        return pred

    def preprocess_image(self, img):
        # 转换为 RGB
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        
        # 确保通道数是3 (RGB)
        if img.shape[-1] != 3:  # 检查通道数是否为3
            raise ValueError(f"Expected image with 3 channels, but got {img.shape[-1]} channels.")

        # 保证数据连续性
        img = np.ascontiguousarray(img)

        # 转换为 PyTorch 张量，并移到指定设备
        img = torch.from_numpy(img).to(self.device)

        # 转换为浮点型
        img = img.float()

        # 标准化：将像素值从 0-255 归一化到 0-1
        img /= 255.0
        
        # 调整通道顺序：从 (H, W, C) -> (C, H, W)
        img = img.permute(2, 0, 1)  # 变成 (3, 480, 640)

        # 如果图像是三维（H, W, C），增加 batch 维度
        if img.ndimension() == 3:
            img = img.unsqueeze(0)
    
        return img


    def postprocess_detections(self, detections, img_shape):
        results = []
        for det in detections:
            if det is not None and len(det):
                det[:, :4] = scale_coords(img_shape, det[:, :4], img_shape).round()
                results.append(det)
        return results

    def draw_detections(self, img, detections, names):
        if detections is None:
            return img
        for det in detections:
            for *xyxy, conf, cls in reversed(det):
                cls = int(cls)  # 确保类别索引是整数
                color = self.class_colors[cls]  # 获取或生成该类别的颜色
                
                label = f'{names[cls]} {conf:.2f}'
                cv2.rectangle(img, (int(xyxy[0]), int(xyxy[1])), (int(xyxy[2]), int(xyxy[3])), color, 2)
                cv2.putText(img, label, (int(xyxy[0]), int(xyxy[1]) - 5), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        return img
    
    def format_detections(self, detections):
        """ 将检测结果转换为 JSON 格式，方便发布到 ROS 话题 """
        results = []
        for det in detections:
            for *xyxy, conf, cls in reversed(det):
                result = {
                    "class": CLASS_NAMES[int(cls)],
                    "confidence": float(conf),
                    "bbox": [int(xyxy[0]), int(xyxy[1]), int(xyxy[2]), int(xyxy[3])]
                }
                results.append(result)
        
        json_message = json.dumps(results)
        ros_message = String()
        ros_message.data = json_message
        return ros_message

def main(args=None):
    rclpy.init(args=args)
    model_path = WEIGHT_PATH
    node = YoloDetector(model_path,visualization=VISUALIZATION)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 销毁节点并关闭所有OpenCV窗口
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

    # 测试单张图片
    # test_img = "/home/jetson/code/dofbot_ros2_ws/src/ros2_vision_arm_control/output0113_mov-0057.jpg"
    # rclpy.init(args=None)
    # model_path = WEIGHT_PATH
    # node = YoloDetector(model_path)
    # _img = cv2.imread(test_img)
    # detections = node.detect(_img)
    # YoloDetector.get_logger(node).info("detected an image")
    # print("Done")
