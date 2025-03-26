import matplotlib.pyplot as plt
import numpy as np

def plot_one_box(xyxy, img, color=None, label=None, line_thickness=3):
    if color is None:
        color = (255, 0, 0)  # Default color is red
    x1, y1, x2, y2 = xyxy
    plt.rectangle((x1, y1), (x2, y2), color=color, linewidth=line_thickness)
    if label:
        plt.text(x1, y1, label, color=color, fontsize=12, bbox=dict(facecolor='white', alpha=0.5))

def plot_results(img, detections):
    plt.imshow(img)
    for det in detections:
        xyxy, conf, cls = det[:3]
        label = f'Class {cls} {conf:.2f}'
        plot_one_box(xyxy, img, label=label)
    plt.axis('off')
    plt.show()