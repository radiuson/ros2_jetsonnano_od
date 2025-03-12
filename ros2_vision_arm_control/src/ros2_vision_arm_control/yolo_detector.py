import cv2
import numpy as np
import torch
from utils.general import non_max_suppression, scale_coords
from utils.torch_utils import select_device

class YoloDetector:
    def __init__(self, model_path, device=''):
        self.device = select_device(device)
        self.model = self.load_model(model_path)

    def load_model(self, model_path):
        model = torch.load(model_path, map_location=self.device)['model'].float()  # load to FP32
        model.eval()
        
        return model

    def detect(self, img, conf_threshold=0.25, iou_threshold=0.45):
        img = self.preprocess_image(img)
        with torch.no_grad():
            pred = self.model(img, augment=False)[0]
            pred = non_max_suppression(pred, conf_threshold, iou_threshold)

        return pred

    def preprocess_image(self, img):
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img = np.ascontiguousarray(img)
        img = torch.from_numpy(img).to(self.device)
        img = img.half() if self.device.type != 'cpu' else img.float()  # uint8 to fp16/32
        img /= 255.0  # 0 - 255 to 0.0 - 1.0
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
        for det in detections:
            for *xyxy, conf, cls in reversed(det):
                label = f'{names[int(cls)]} {conf:.2f}'
                cv2.rectangle(img, (int(xyxy[0]), int(xyxy[1])), (int(xyxy[2]), int(xyxy[3])), (255, 0, 0), 2)
                cv2.putText(img, label, (int(xyxy[0]), int(xyxy[1]) - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
        return img