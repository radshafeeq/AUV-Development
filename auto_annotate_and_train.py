#!/usr/bin/env python3
"""
AUV Auto-Annotation & Fine-Tuning Pipeline
-------------------------------------------
1. Reads captured images from 'dataset_images/'
2. Uses open-vocabulary YOLO-World to auto-generate YOLO format annotation files (.txt)
3. Prepares train/val dataset structure & 'mechatronics_dataset.yaml'
4. Fine-tunes YOLOv8 Small model on NVIDIA RTX 4070 GPU
5. Updates 'auv_yolo_tracking.py' to use custom trained 'best.pt'
"""

import os
import sys
import shutil
import cv2
import torch
import numpy as np
from ultralytics import YOLO, YOLOWorld

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
IMAGES_DIR = os.path.join(BASE_DIR, "dataset_images")
DATASET_DIR = os.path.join(BASE_DIR, "mechatronics_dataset")

CLASSES = ["pixhawk", "bldc_motor", "esc", "battery", "cable"]

def auto_annotate_images():
    print("=" * 60)
    print(" 1. AUTO-ANNOTATING CAPTURED IMAGES USING YOLO-WORLD ")
    print("=" * 60)

    if not os.path.exists(IMAGES_DIR) or len(os.listdir(IMAGES_DIR)) == 0:
        print(f"[Error] No images found in '{IMAGES_DIR}'. Please run 'python3 capture_training_images.py' first!")
        sys.exit(1)

    # Prepare dataset folder structure
    images_train = os.path.join(DATASET_DIR, "images", "train")
    labels_train = os.path.join(DATASET_DIR, "labels", "train")
    images_val = os.path.join(DATASET_DIR, "images", "val")
    labels_val = os.path.join(DATASET_DIR, "labels", "val")

    for path in [images_train, labels_train, images_val, labels_val]:
        os.makedirs(path, exist_ok=True)

    # Initialize YOLO-World auto-labeler
    print("[AI Labeler] Loading YOLO-World for auto-annotation...")
    detector = YOLOWorld("yolov8s-world.pt")
    detector.set_classes(["pixhawk flight controller", "circuit board", "electronic module", "bldc motor", "esc", "battery", "cable"])

    image_files = [f for f in os.listdir(IMAGES_DIR) if f.lower().endswith(('.jpg', '.jpeg', '.png'))]
    print(f"[AI Labeler] Found {len(image_files)} training images. Annotating...")

    annotated_count = 0

    for idx, fname in enumerate(image_files):
        img_path = os.path.join(IMAGES_DIR, fname)
        frame = cv2.imread(img_path)
        if frame is None:
            continue

        h, w, _ = frame.shape
        results = detector.predict(frame, conf=0.08, verbose=False)[0]

        # Determine train vs val split (80% train, 20% val)
        is_val = (idx % 5 == 0)
        target_img_dir = images_val if is_val else images_train
        target_lbl_dir = labels_val if is_val else labels_train

        base_name = os.path.splitext(fname)[0]
        dst_img_path = os.path.join(target_img_dir, fname)
        dst_lbl_path = os.path.join(target_lbl_dir, base_name + ".txt")

        shutil.copy(img_path, dst_img_path)

        label_lines = []
        for box in results.boxes:
            x1, y1, x2, y2 = map(float, box.xyxy[0])
            
            # Convert xyxy to normalized YOLO format (class center_x center_y width height)
            center_x = ((x1 + x2) / 2.0) / w
            center_y = ((y1 + y2) / 2.0) / h
            box_w = (x2 - x1) / w
            box_h = (y2 - y1) / h

            # Map all detected flight controllers/boards to class 0 (pixhawk)
            class_id = 0
            label_lines.append(f"{class_id} {center_x:.6f} {center_y:.6f} {box_w:.6f} {box_h:.6f}")

        # If YOLO-World didn't catch the box, fall back to center crop box (assume user framed the object)
        if not label_lines:
            # Fallback box around frame center (40% width, 40% height)
            center_x, center_y = 0.5, 0.5
            box_w, box_h = 0.5, 0.5
            label_lines.append(f"0 {center_x:.6f} {center_y:.6f} {box_w:.6f} {box_h:.6f}")

        with open(dst_lbl_path, "w") as f:
            f.write("\n".join(label_lines) + "\n")

        annotated_count += 1

    print(f"[AI Labeler] Successfully annotated {annotated_count} images.")

    # Create mechatronics_dataset.yaml
    yaml_content = f"""path: {DATASET_DIR}
train: images/train
val: images/val

names:
  0: pixhawk
  1: bldc_motor
  2: esc
  3: battery
  4: cable
"""
    yaml_path = os.path.join(BASE_DIR, "mechatronics_dataset.yaml")
    with open(yaml_path, "w") as f:
        f.write(yaml_content)

    print(f"[Dataset] Created dataset YAML at '{yaml_path}'")
    return yaml_path

def train_model(yaml_path):
    print("=" * 60)
    print(" 2. TRAINING FINE-TUNED MODEL ON RTX 4070 GPU ")
    print("=" * 60)

    device = 0 if torch.cuda.is_available() else "cpu"
    print(f"[GPU Training] Using device: {torch.cuda.get_device_name(0) if torch.cuda.is_available() else 'CPU'}")

    model = YOLO("yolov8s.pt")
    results = model.train(
        data=yaml_path,
        epochs=35,
        imgsz=640,
        device=device,
        batch=8,
        name="mechatronics_model",
        exist_ok=True
    )

    best_weights = os.path.join(BASE_DIR, "runs", "detect", "mechatronics_model", "weights", "best.pt")
    print("=" * 60)
    print(f"[SUCCESS] Fine-tuning completed!")
    print(f"Trained weights saved to: {best_weights}")
    print("=" * 60)
    return best_weights

def update_tracking_script():
    tracking_script = os.path.join(BASE_DIR, "auv_yolo_tracking.py")
    if not os.path.exists(tracking_script):
        return

    with open(tracking_script, "r") as f:
        content = f.read()

    content = content.replace("USE_YOLO_WORLD = True", "USE_YOLO_WORLD = False")
    content = content.replace("CONF_THRESHOLD = 0.12", "CONF_THRESHOLD = 0.40")

    with open(tracking_script, "w") as f:
        f.write(content)

    print("[Config] Updated 'auv_yolo_tracking.py' to use custom trained weights with 0.40 threshold!")

def main():
    yaml_path = auto_annotate_images()
    best_weights = train_model(yaml_path)
    update_tracking_script()
    print("\nALL DONE! You can now run: python3 auv_yolo_tracking.py")

if __name__ == "__main__":
    main()
