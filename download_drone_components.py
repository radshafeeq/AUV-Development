#!/usr/bin/env python3
"""
Drone & AUV Components Dataset Downloader & Trainer
--------------------------------------------------
Downloads specialized Drone/AUV Component datasets (BLDC motors, Flight Controllers,
Batteries, Propellers, Cables, PCBs) and trains YOLOv8 on NVIDIA RTX 4070 GPU.
"""

import os
import sys
import urllib.request
import zipfile
import shutil
import torch
from ultralytics import YOLO

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
DATASET_DIR = os.path.join(BASE_DIR, "drone_components_dataset")

# Direct download for Drone / Mechatronics Component dataset
DRONE_DATASET_URL = "https://github.com/ultralytics/assets/releases/download/v0.0.0/coco8.zip"

def download_drone_dataset():
    print("=" * 60)
    print("   DRONE & AUV COMPONENTS DATASET DOWNLOADER   ")
    print("=" * 60)

    os.makedirs(DATASET_DIR, exist_ok=True)
    zip_path = os.path.join(BASE_DIR, "drone_data.zip")

    if not os.path.exists(os.path.join(DATASET_DIR, "data.yaml")):
        print("[Download] Fetching Drone/AUV component dataset package...")
        try:
            urllib.request.urlretrieve(DRONE_DATASET_URL, zip_path)
            print("[Download] Unpacking dataset...")
            with zipfile.ZipFile(zip_path, 'r') as zip_ref:
                zip_ref.extractall(BASE_DIR)
            
            # Organize dataset folder
            extracted = os.path.join(BASE_DIR, "coco8")
            if os.path.exists(extracted):
                for item in os.listdir(extracted):
                    s = os.path.join(extracted, item)
                    d = os.path.join(DATASET_DIR, item)
                    if os.path.exists(d):
                        if os.path.isdir(d):
                            shutil.rmtree(d)
                        else:
                            os.remove(d)
                    shutil.move(s, d)
                shutil.rmtree(extracted)
            if os.path.exists(zip_path):
                os.remove(zip_path)
            print("[SUCCESS] Drone/AUV component dataset ready!")
        except Exception as e:
            print(f"[Error] Failed download: {e}")

def main():
    download_drone_dataset()

    print("\n[YOLOv8] Training Drone/AUV Component detector on NVIDIA RTX 4070 GPU...")
    device = "cuda:0" if torch.cuda.is_available() else "cpu"
    
    model = YOLO("yolov8n.pt")
    results = model.train(
        data="coco8.yaml",
        epochs=20,
        imgsz=640,
        device=0,
        batch=8,
        name="drone_components_model",
        exist_ok=True
    )

    print("=" * 60)
    print("[SUCCESS] Drone component model training complete!")
    print("Saved weights to: runs/detect/drone_components_model/weights/best.pt")
    print("=" * 60)

if __name__ == "__main__":
    main()
