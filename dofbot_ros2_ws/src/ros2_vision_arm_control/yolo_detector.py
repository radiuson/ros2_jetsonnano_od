#!/usr/bin/env python3
import json
import cv2
import numpy as np
import torch
from models.experimental import attempt_load
from utils.general import non_max_suppression, scale_coords
from utils.torch_utils import select_device
import rclpy
import time
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import String
from sensor_msgs.msg import Image

from ros2_vision_arm_control.msg import VisionDetection, BoundingBox

from cv_bridge import CvBridge
from utils import (TOPIC_ROBOT1_STATUS,
                   TOPIC_YOLO_DEPTH,
                   TOPIC_YOLO_DETECTION,
                   WEIGHT_PATH,
                   TOPIC_CAMERA_RGB,
                   TOPIC_CAMERA_DEPTH,
                   VISUALIZATION,
                   CLASS_NAMES,
                   TEST_IMG_PATH,
                   TRIGGER_YOLO_DEPTH,
                   TOPIC_CAMERA_MSG,
                   CAMERA_HEIGHT,
                   CAMERA_WIDTH,
                   TOPIC_YOLO_RESULT,
                   )



class YoloDetector(Node):
    def __init__(self, model_path=WEIGHT_PATH, device='',visualization=VISUALIZATION,test_img_path=TEST_IMG_PATH):
        super().__init__('yolo_detector_node')
        self.device = select_device(device)
        self.model = self.load_model(model_path,device=self.device)

        self.bridge = CvBridge()
        self.visualization = visualization
        self.robot_status = 'IDLE'
        self.inferencing = False

        self.status_subscription = self.create_subscription(
            String, TOPIC_ROBOT1_STATUS, self.status_callback, 10
        )

        self.detection_publisher = self.create_publisher(VisionDetection, TOPIC_YOLO_RESULT, 2)

        self.detection_subscription = self.create_subscription(
                    VisionDetection,
                    TOPIC_CAMERA_MSG,  
                    self.detection_callback,
                    3  # 队列大小
                )
        # RGB话题订阅
        # self.subscription = self.create_subscription(
        #     Image,
        #     TOPIC_CAMERA_RGB,  
        #     self.image_callback,
        #     10
        # )
        self.depth_image_trigger = self.create_service(Trigger,TRIGGER_YOLO_DEPTH,self.depth_trigger_callback)

        # self.test_img_path = test_img_path
        # _ = self.model(self.get_test_img())
        # self.get_logger().info("First inference done")

        # DEPTH话题订阅
        # self.depth_subscription = self.create_subscription(
        #     Image,
        #     TOPIC_CAMERA_DEPTH,  
        #     self.depth_callback,
        #     10
        # )
        # 配套深度图发布以及YOLO结果发布
        # self.depth_publisher = self.create_publisher(Image, TOPIC_YOLO_DEPTH, 10)

        # self.publisher = self.create_publisher(String, TOPIC_YOLO_DETECTION, 10)
        self.get_logger().info("YoloDetector Node Initialized")

    def load_model(self, model_path,device):
        self.get_logger().info("Start loading weight...")
        model = attempt_load(model_path,map_location=device).float()  # load to FP32
        model.eval()
        self.get_logger().info("Model Loaded")
        return model
    
    def status_callback(self, msg):
        """ 监听机器人状态，当机械臂运动时，暂停 YOLO 推理 """
        self.robot_status = msg.data
        self.get_logger().info(f"Received Robot Status: {self.robot_status}")

    def detection_callback(self, msg:VisionDetection):
        # # Convert ROS Image message to OpenCV image
        # try:
        #     cv_image = self.bridge.imgmsg_to_cv2(msg.rgb_image, desired_encoding='bgr8')
        #     # Save the image to a local directory
        #     timestamp = int(time.time() * 1000)  # Use timestamp to ensure unique filenames
        #     save_path = f"/home/jetson/code/img/image_{timestamp}.jpg"
        #     cv2.imwrite(save_path, cv_image)
        #     self.get_logger().info(f"Image saved to {save_path}")
        # except Exception as e:
        #     self.get_logger().error(f"Failed to save image: {e}")





        self.inferencing = True
        self.get_logger().info("Received an image")
        
        if self.robot_status == "MOVING":
            self.get_logger().info("Robot is moving, skipping YOLO inference.")
            return 
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg.rgb_image, desired_encoding='bgr8')
            detections = self.detect(cv_image)
            self.format_boundingboxes(msg,detections[0])
            self.detection_publisher.publish(msg)
            
            self.get_logger().info(f"Published YOLO detections")

            if self.visualization:
                processed_image = self.draw_detections(cv_image, detections, names=CLASS_NAMES)  # 替换为实际类别
                # 显示检测结果
                cv2.imshow("YOLO Detection", processed_image)
                cv2.waitKey(100)  # 必须调用以刷新窗口

            time.sleep(0.3)
            self.inferencing = False
        except Exception as e:
            if self.visualization:
                processed_image = self.draw_detections(cv_image, None, names=CLASS_NAMES)  # 替换为实际类别
                # 显示检测结果
                cv2.imshow("YOLO Detection", processed_image)
                cv2.waitKey(100)  # 必须调用以刷新窗口
            self.get_logger().error(f"Failed to process image: {e}")
            time.sleep(0.3)
            self.inferencing = False


    def format_boundingboxes(self,msg:VisionDetection,detections:torch.Tensor):
        # 遍历检测结果，将每个检测框添加到 VisionDetection 消息的 boxes 数组中]
        msg.boxes.clear()
        for detection in detections:
            detection = detection.cpu().tolist()
            print(detection)
            # 假设 detection 是一个包含 bbox 信息的对象，格式为 [xmin, ymin, xmax, ymax, confidence, class_id, class_name]
            box = BoundingBox()
            box.xmin = int(detection[0])
            box.ymin = int(detection[1])
            box.xmax = int(detection[2])
            box.ymax = int(detection[3])
            box.confidence = detection[4]
            box.class_id = int(detection[5])
            box.class_name = CLASS_NAMES[int(detection[5])]
            msg.boxes.append(box)


    def image_callback(self, msg):
        self.inferencing = True
        
        self.get_logger().info("Received an image")
        
        if self.robot_status == "MOVING":
            self.get_logger().info("Robot is moving, skipping YOLO inference.")
            return 
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            detections = self.detect(cv_image)

            pred_message = self.format_detections(detections)
            self.publisher.publish(pred_message)
            self.get_logger().info(f"Published YOLO detections")

            if self.visualization:
                processed_image = self.draw_detections(cv_image, detections, names=CLASS_NAMES)  # 替换为实际类别
                # 显示检测结果
                cv2.imshow("YOLO Detection", processed_image)
                cv2.waitKey(100)  # 必须调用以刷新窗口

            # 可在此处发布处理后的图片或其他操作
            if self.latest_depth_image is not None:
                self.depth_publisher.publish(self.latest_depth_image)
                self.get_logger().info("Published corresponding depth image.")
            time.sleep(0.3)
            self.inferencing = False
        except Exception as e:
            if self.visualization:
                processed_image = self.draw_detections(cv_image, None, names=CLASS_NAMES)  # 替换为实际类别
                # 显示检测结果
                cv2.imshow("YOLO Detection", processed_image)
                cv2.waitKey(100)  # 必须调用以刷新窗口
            self.get_logger().error(f"Failed to process image: {e}")
            time.sleep(0.3)
            self.inferencing = False

    def depth_trigger_callback(self,request,response):
        self.depth_publisher.publish(self.latest_depth_image)
        # 设置 Trigger 响应
        response.success = True
        response.message = 'Depth image published'

        return response

    def detect(self, img, conf_threshold=0.5, iou_threshold=0.45):
        img = self.preprocess_image(img)
        with torch.no_grad():
            pred, *_ = self.model(img)
            pred = non_max_suppression(pred, conf_threshold, iou_threshold)
            # pred = self.postprocess_detections(pred,(640,640),(CAMERA_WIDTH,CAMERA_HEIGHT))
            if pred is not None:
                self.get_logger().info(f"Detected {len(pred[0])} objects")

        return pred
    
    def depth_callback(self, msg):
        if self.inferencing:
            return
        self.latest_depth_image = msg
        

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


    def postprocess_detections(self, detections, img1_shape,img0_shape):
        results = []
        for det in detections:
            if det is not None :
                det[:, :4] = scale_coords(img1_shape, det[:, :4], img0_shape).round()
                results.append(det)
        return results

    def draw_detections(self, img, detections, names):

        if detections is None:
            return img
        # 定义类别颜色映射（可以根据实际类别调整）
        colors = {
            0: (255, 0, 0),   # 类别0 - 红色
            1: (0, 255, 0),   # 类别1 - 绿色
            2: (0, 0, 255),   # 类别2 - 蓝色
            3: (255, 255, 0), # 类别3 - 青色
            4: (255, 0, 255), # 类别4 - 紫色
            5: (0, 255, 255)  # 类别5 - 黄色
        }

        for det in detections:
            for *xyxy, conf, cls in reversed(det):
                cls = int(cls)  # 确保类别索引是整数
                color = colors.get(cls, (200, 200, 200))  # 如果类别不在字典中，使用灰色
                
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
                    "confidence": float(f"{conf:.2f}"),
                    "bbox": [int(xyxy[0]), int(xyxy[1]), int(xyxy[2]), int(xyxy[3])]
                }
                results.append(result)
        
        json_message = json.dumps(results)
        ros_message = String()
        ros_message.data = json_message
        return ros_message
        
    def get_test_img(self):
        _img = cv2.imread(self.test_img_path)
        _img = self.preprocess_image(_img)
        return _img
def main(args=None):
    rclpy.init(args=args)
    model_path = WEIGHT_PATH
    node = YoloDetector(model_path)
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
