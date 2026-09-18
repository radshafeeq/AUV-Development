#!/usr/bin/env python3
"""
YOLO26 Combined Target Trainer
------------------------------
Trains the state-of-the-art YOLO26 model on the 11-class combined dataset
(Mechatronics + Underwater + Office targets) using NVIDIA RTX 4070 GPU.
"""

import os
import sys
import torch
from ultralytics import YOLO

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
DATASET_YAML = os.path.join(BASE_DIR, "combined_dataset", "data.yaml")
WEIGHTS_PATH = os.path.join(BASE_DIR, "weights", "yolo26n.pt")

def train(epochs=25, batch=16):
    print("=" * 60)
    print("   YOLO26 COMBINED TARGET TRAINER (RTX 4070 GPU)   ")
    print("=" * 60)

    device = "cuda:0" if torch.cuda.is_available() else "cpu"
    print(f"[Device] Using PyTorch Device: {device}")
    if torch.cuda.is_available():
        print(f"[Device] GPU: {torch.cuda.get_device_name(0)}")

    if not os.path.exists(DATASET_YAML):
        print(f"[Error] Combined dataset YAML not found at '{DATASET_YAML}'")
        return

    model_src = WEIGHTS_PATH
    print(f"[YOLO26] Loading base model weights from '{model_src}'...")
    model = YOLO(model_src)

    print(f"[YOLO26] Starting training on '{DATASET_YAML}'...")
    results = model.train(
        data=DATASET_YAML,
        epochs=epochs,
        imgsz=640,
        device=0 if torch.cuda.is_available() else "cpu",
        batch=batch,
        name="yolo26_combined_model",
        exist_ok=True
    )

    print("=" * 60)
    print("[SUCCESS] YOLO26 Training completed!")
    print("Trained YOLO26 weights saved to: runs/detect/yolo26_combined_model/weights/best.pt")
    print("=" * 60)

if __name__ == "__main__":
    epochs = int(sys.argv[1]) if len(sys.argv) > 1 else 25
    train(epochs=epochs)
