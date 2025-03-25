import os
import cv2
import numpy as np

def load_images_from_folder(folder):
    images = []
    for filename in os.listdir(folder):
        img = cv2.imread(os.path.join(folder, filename))
        if img is not None:
            images.append(img)
    return images

def preprocess_image(image, img_size=(640, 640)):
    image = cv2.resize(image, img_size)
    image = image / 255.0  # Normalize to [0, 1]
    image = np.transpose(image, (2, 0, 1))  # Change to (C, H, W)
    return image

def load_dataset(dataset_path):
    images = load_images_from_folder(dataset_path)
    processed_images = [preprocess_image(img) for img in images]
    return processed_images