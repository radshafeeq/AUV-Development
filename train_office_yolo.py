#!/usr/bin/env python3
"""
YOLOv8 Office Hardware & Safety Equipment Trainer
-------------------------------------------------
Trains YOLOv8 on the Office dataset ('office.v2i.yolov8') containing
'exit sign' and 'fire hydrant' classes using NVIDIA RTX 4070 GPU acceleration.
"""

import os
import sys
import torch
from ultralytics import YOLO

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
DATASET_YAML = os.path.join(BASE_DIR, "office_dataset", "data.yaml")

def train(epochs=30, batch=16, model_size="s"):
    print("=" * 60)
    print("   YOLOv8 OFFICE DATASET TRAINER (RTX 4070 GPU)   ")
    print("=" * 60)

    device = "cuda:0" if torch.cuda.is_available() else "cpu"
    print(f"[Device] Using PyTorch Device: {device}")
    if torch.cuda.is_available():
        print(f"[Device] GPU: {torch.cuda.get_device_name(0)}")

    if not os.path.exists(DATASET_YAML):
        print(f"[Error] Dataset config file not found at '{DATASET_YAML}'")
        return

    base_model_file = f"yolov8{model_size}.pt"
    print(f"[YOLOv8] Loading base model: {base_model_file}...")
    model = YOLO(base_model_file)

    print(f"[YOLOv8] Starting fine-tuning on '{DATASET_YAML}'...")
    results = model.train(
        data=DATASET_YAML,
        epochs=epochs,
        imgsz=640,
        device=0 if torch.cuda.is_available() else "cpu",
        batch=batch,
        name="office_model",
        exist_ok=True
    )

    print("=" * 60)
    print("[SUCCESS] Training completed!")
    print("Custom trained weights saved to: runs/detect/office_model/weights/best.pt")
    print("You can now set MODEL_NAME = 'runs/detect/office_model/weights/best.pt' in auv_yolo_tracking.py!")
    print("=" * 60)

if __name__ == "__main__":
    epochs = int(sys.argv[1]) if len(sys.argv) > 1 else 30
    train(epochs=epochs)
