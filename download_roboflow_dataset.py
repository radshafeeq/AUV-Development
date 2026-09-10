#!/usr/bin/env python3
"""
Automated Roboflow Drone Kit Dataset Downloader & Trainer
---------------------------------------------------------
Downloads 'oleksii/f120-dronekit' (BLDC motors, Flight Controllers, Batteries, Propellers, Cables, PCBs)
and trains custom YOLOv8 weights on NVIDIA RTX 4070 GPU.
"""

import os
import sys
import torch
from roboflow import Roboflow
from ultralytics import YOLO

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
DATASET_DIR = os.path.join(BASE_DIR, "dataset")

def download_and_train(api_key):
    print("=" * 60)
    print("   DOWNLOADING F120 DRONEKIT DATASET FROM ROBOFLOW   ")
    print("=" * 60)

    try:
        rf = Roboflow(api_key=api_key)
        project = rf.workspace("oleksii").project("f120-dronekit")
        dataset = project.version(1).download("yolov8", location=DATASET_DIR)
        print("\n[SUCCESS] Dataset downloaded successfully into 'dataset/'!")
    except Exception as e:
        print(f"\n[Error] Download failed: {e}")
        return

    print("\n" + "=" * 60)
    print("   STARTING TRAINING ON NVIDIA GEFORCE RTX 4070 GPU   ")
    print("=" * 60)

    data_yaml = None
    if 'dataset' in locals() and hasattr(dataset, 'location'):
        candidate = os.path.join(dataset.location, "data.yaml")
        if os.path.exists(candidate):
            data_yaml = candidate

    if not data_yaml or not os.path.exists(data_yaml):
        for root, dirs, files in os.walk(DATASET_DIR):
            if "data.yaml" in files:
                data_yaml = os.path.join(root, "data.yaml")
                break

    if not data_yaml or not os.path.exists(data_yaml):
        data_yaml = os.path.join(DATASET_DIR, "data.yaml")
        print(f"[Dataset] Generating '{data_yaml}'...")
        yaml_content = f"""path: {DATASET_DIR}
train: images/train
val: images/val

names:
  0: bldc_motor
  1: pixhawk
  2: esc
  3: battery
  4: charger
  5: cable
  6: propeller
  7: pcb
  8: camera
"""
        with open(data_yaml, "w") as f:
            f.write(yaml_content)

    print(f"[Dataset] Configured dataset YAML at: {data_yaml}")
    device = "cuda:0" if torch.cuda.is_available() else "cpu"

    model = YOLO("yolov8n.pt")
    results = model.train(
        data=data_yaml,
        epochs=30,
        imgsz=640,
        device=0,
        batch=16,
        name="mechatronics_model",
        exist_ok=True
    )

    print("=" * 60)
    print("[SUCCESS] Drone Component Model trained and ready!")
    print("Weights saved to: runs/detect/mechatronics_model/weights/best.pt")
    print("=" * 60)

if __name__ == "__main__":
    if len(sys.argv) > 1:
        key = sys.argv[1]
    else:
        key = input("Enter your Roboflow Private API Key: ").strip()

    if not key:
        print("Error: API key required.")
        sys.exit(1)

    download_and_train(key)
