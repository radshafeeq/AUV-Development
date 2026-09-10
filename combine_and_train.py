#!/usr/bin/env python3
"""
Combine Datasets & Train Unified YOLOv8 Model
----------------------------------------------
Combines the Mechatronics/Hardware dataset and the Office dataset into a single
unified dataset ('combined_dataset/') and fine-tunes YOLOv8 on RTX 4070 GPU.
"""

import os
import sys
import shutil
import glob
import torch
from ultralytics import YOLO

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
COMBINED_DIR = os.path.join(BASE_DIR, "combined_dataset")
OFFICE_DIR = os.path.join(BASE_DIR, "office_dataset")
MECHA_DIR = os.path.join(BASE_DIR, "dataset")

# Unified Classes Mapping
UNIFIED_CLASSES = [
    "bldc_motor",        # 0
    "pixhawk",           # 1
    "esc",               # 2
    "battery",           # 3
    "charger",           # 4
    "cable",             # 5
    "soldering_iron",    # 6
    "underwater_buoy",   # 7
    "underwater_gate",   # 8
    "exit sign",         # 9
    "fire hydrant"       # 10
]

OFFICE_CLASS_MAP = {
    0: 9,   # 'exit sign' -> 9
    1: 10   # 'fire hydrant' -> 10
}

def setup_combined_directory():
    print("[Combined Dataset] Setting up clean 'combined_dataset/' structure...")
    for split in ["train", "valid"]:
        os.makedirs(os.path.join(COMBINED_DIR, split, "images"), exist_ok=True)
        os.makedirs(os.path.join(COMBINED_DIR, split, "labels"), exist_ok=True)

def process_office_dataset():
    print("[Combined Dataset] Merging office dataset (exit sign, fire hydrant)...")
    for split_in, split_out in [("train", "train"), ("valid", "valid")]:
        img_dir = os.path.join(OFFICE_DIR, split_in, "images")
        lbl_dir = os.path.join(OFFICE_DIR, split_in, "labels")

        if not os.path.exists(img_dir):
            continue

        images = glob.glob(os.path.join(img_dir, "*.*"))
        for img_path in images:
            basename = os.path.basename(img_path)
            stem, _ = os.path.splitext(basename)
            dest_img = os.path.join(COMBINED_DIR, split_out, "images", f"office_{basename}")
            shutil.copy2(img_path, dest_img)

            # Process label file with re-mapped class IDs
            lbl_path = os.path.join(lbl_dir, f"{stem}.txt")
            dest_lbl = os.path.join(COMBINED_DIR, split_out, "labels", f"office_{stem}.txt")

            if os.path.exists(lbl_path):
                new_lines = []
                with open(lbl_path, "r") as f:
                    lines = f.readlines()
                for line in lines:
                    parts = line.strip().split()
                    if not parts:
                        continue
                    old_cls = int(parts[0])
                    new_cls = OFFICE_CLASS_MAP.get(old_cls, old_cls + 9)
                    parts[0] = str(new_cls)
                    new_lines.append(" ".join(parts))
                with open(dest_lbl, "w") as f:
                    f.write("\n".join(new_lines) + "\n")

def process_mecha_dataset():
    print("[Combined Dataset] Merging mechatronics dataset...")
    for split_in, split_out in [("train", "train"), ("val", "valid")]:
        img_dir = os.path.join(MECHA_DIR, "images", split_in)
        lbl_dir = os.path.join(MECHA_DIR, "labels", split_in)

        if not os.path.exists(img_dir):
            continue

        images = glob.glob(os.path.join(img_dir, "*.*"))
        for img_path in images:
            basename = os.path.basename(img_path)
            stem, _ = os.path.splitext(basename)
            dest_img = os.path.join(COMBINED_DIR, split_out, "images", f"mecha_{basename}")
            shutil.copy2(img_path, dest_img)

            lbl_path = os.path.join(lbl_dir, f"{stem}.txt")
            dest_lbl = os.path.join(COMBINED_DIR, split_out, "labels", f"mecha_{stem}.txt")
            if os.path.exists(lbl_path):
                shutil.copy2(lbl_path, dest_lbl)

def write_combined_yaml():
    yaml_path = os.path.join(COMBINED_DIR, "data.yaml")
    print(f"[Combined Dataset] Writing combined data.yaml to '{yaml_path}'...")
    names_dict = {i: name for i, name in enumerate(UNIFIED_CLASSES)}
    yaml_content = f"""path: {COMBINED_DIR}
train: train/images
val: valid/images

nc: {len(UNIFIED_CLASSES)}
names:
"""
    for i, name in enumerate(UNIFIED_CLASSES):
        yaml_content += f"  {i}: '{name}'\n"

    with open(yaml_path, "w") as f:
        f.write(yaml_content)

    return yaml_path

def train_combined(epochs=30):
    setup_combined_directory()
    process_office_dataset()
    process_mecha_dataset()
    yaml_path = write_combined_yaml()

    print("=" * 60)
    print("   TRAINING UNIFIED YOLOv8 MODEL ON COMBINED DATASET   ")
    print("=" * 60)

    device = "cuda:0" if torch.cuda.is_available() else "cpu"
    print(f"[Device] Using PyTorch Device: {device}")
    if torch.cuda.is_available():
        print(f"[Device] GPU: {torch.cuda.get_device_name(0)}")

    model = YOLO("yolov8s.pt")
    results = model.train(
        data=yaml_path,
        epochs=epochs,
        imgsz=640,
        device=0 if torch.cuda.is_available() else "cpu",
        batch=16,
        name="combined_model",
        exist_ok=True
    )

    print("=" * 60)
    print("[SUCCESS] Unified training completed!")
    print("Combined weights saved to: runs/detect/combined_model/weights/best.pt")
    print("=" * 60)

if __name__ == "__main__":
    epochs = int(sys.argv[1]) if len(sys.argv) > 1 else 30
    train_combined(epochs=epochs)
