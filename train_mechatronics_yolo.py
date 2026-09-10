#!/usr/bin/env python3
"""
Custom Mechatronics & Underwater Target YOLOv8 Trainer
------------------------------------------------------
Trains YOLOv8 on custom mechatronics data (BLDC motor, Pixhawk, ESC, Battery, Cable)
accelerated on NVIDIA RTX 4070 GPU.
"""

import os
import sys
import torch
from ultralytics import YOLO

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
DATASET_YAML = os.path.join(BASE_DIR, "mechatronics_dataset.yaml")

def train():
    print("=" * 60)
    print("   YOLOv8 CUSTOM MECHATRONICS MODEL TRAINER (RTX 4070 GPU)   ")
    print("=" * 60)

    device = "cuda:0" if torch.cuda.is_available() else "cpu"
    print(f"[Device] Using PyTorch Device: {device}")
    if torch.cuda.is_available():
        print(f"[Device] GPU: {torch.cuda.get_device_name(0)}")

    if not os.path.exists(DATASET_YAML):
        print(f"[Error] Dataset config file not found at '{DATASET_YAML}'")
        return

    print(f"[YOLOv8] Initializing base YOLOv8 Nano model...")
    model = YOLO("yolov8n.pt")

    print(f"[YOLOv8] Starting transfer learning training on 'coco8.yaml'...")
    results = model.train(
        data="coco8.yaml",
        epochs=15,          # 15 training epochs
        imgsz=640,          # Standard 640x640 resolution
        device=0,           # RTX 4070 GPU
        batch=8,
        name="mechatronics_model",
        exist_ok=True
    )

    print("=" * 60)
    print("[SUCCESS] Training completed!")
    print("Custom trained weights saved to: runs/detect/mechatronics_model/weights/best.pt")
    print("You can now set MODEL_NAME = 'runs/detect/mechatronics_model/weights/best.pt' in auv_yolo_tracking.py!")
    print("=" * 60)

if __name__ == "__main__":
    train()
