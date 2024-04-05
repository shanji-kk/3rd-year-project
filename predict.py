import cv2
import numpy as np
import torch
from ultralytics import YOLO
# Load an official or custom model
model = YOLO("C:/Users/dongg/Desktop/YOLOV8N/runs/detect/train8/weights/best.pt")  # Load an official Detect model

while True:
    # Perform tracking with the model
    results = model.predict(source="1", show=True, save=False,conf=0.4)  # Tracking with default tracker
    for result in results:
        boxes = result.boxes
        for box in boxes:
            xywh_tensor = boxes.xywh.clone().detach()
            x, y, w, h = xywh_tensor[box].tolist()
            initBB = (x, y, w, h)
            print(initBB)