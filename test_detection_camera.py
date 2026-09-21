#!/usr/bin/env python3
"""
AUV Detection Camera Bench Test: Direct USB Logitech C922 Evaluation
====================================================================
Designed for bench testing the detection camera directly connected to the laptop:
- Compares detection WITHOUT Kalman Filter (Raw YOLO) vs. WITH Kalman Filter (8D State Estimation)
- Real-time Jitter & Stability Metrics (Pixel fluctuation and noise reduction percentage)
- Side-by-Side (Split-Screen) mode and Overlay mode toggled via hotkey [s]
- Kalman Filter ON/OFF toggled via hotkey [k]
- CLAHE dynamic contrast enhancement toggled via hotkey [e]
- Does NOT alter the subsea BlueOS tether streaming architecture in auv_yolo_tracking.py
"""

import os
import sys
import time
import math
from collections import deque
import cv2
import numpy as np
import torch
from ultralytics import YOLOWorld

# Import the optimized AUV Kalman Filter
from kalman_filter import AUVVisualKalmanFilter

# ==========================================
# CONFIGURATION
# ==========================================
YOLO26_WORLD_WEIGHTS = "weights/yolo26_world.pt"

# Full 70+ open-vocabulary class dictionary identical to auv_yolo_tracking.py
YOLO26_WORLD_CLASSES = [
    # --- Benchtop Electronics & Mobile Gadgets ---
    "smartphone", "cell phone", "mobile phone",
    "computer mouse", "mouse",
    "computer keyboard", "keyboard",
    "laptop", "computer monitor", "tablet",
    "person", "hand",
    "bottle", "water bottle", "cup", "mug",
    "notebook",

    # --- Mechatronics Lab Tools & Workshop Instruments ---
    "digital multimeter", "multimeter",
    "oscilloscope",
    "soldering iron", "wire stripper",
    "screwdriver", "pliers", "wrench",
    "caliper", "vernier caliper", "ruler",
    "scissors", "pen",
    "breadboard", "jumper wire",
    "heat shrink tube",

    # --- AUV & Subsea Robotics Hardware ---
    "pixhawk", "flight controller",
    "bldc motor", "underwater thruster", "thruster", "propeller",
    "electronic speed controller", "esc",
    "lipo battery", "battery", "power bank",
    "charger", "power adapter",
    "ethernet cable", "tether", "cable", "wire",
    "printed circuit board", "circuit board", "pcb",
    "raspberry pi",
    "watertight enclosure", "acrylic tube",

    # --- Subsea Targets & Marine Inspection ---
    "underwater buoy", "marker buoy", "buoy",
    "underwater gate", "navigation gate", "transit gate",
    "torpedo target", "docking station",
    "subsea pipe", "underwater pipeline", "pipe",
    "subsea flange", "subsea valve",
    "underwater cable",
    "diver", "fish"
]

IMG_SIZE = 1024
CONF_THRESHOLD = 0.15

# Priority targets for bench evaluation
PRIORITY_TARGETS = {
    "smartphone", "cell phone", "mobile phone",
    "computer mouse", "mouse",
    "computer keyboard", "keyboard",
    "laptop", "computer monitor", "tablet",
    "bottle", "water bottle", "cup", "mug",
    "digital multimeter", "multimeter", "oscilloscope",
    "soldering iron", "wire stripper", "screwdriver", "pliers", "wrench",
    "caliper", "vernier caliper", "ruler", "scissors", "pen", "notebook",
    "breadboard", "jumper wire", "heat shrink tube",
    "pixhawk", "flight controller",
    "bldc motor", "underwater thruster", "thruster", "propeller",
    "electronic speed controller", "esc",
    "lipo battery", "battery", "power bank", "charger", "power adapter",
    "ethernet cable", "tether", "cable", "wire",
    "printed circuit board", "circuit board", "pcb", "raspberry pi",
    "watertight enclosure", "acrylic tube"
}

def apply_clahe(frame_bgr, clip_limit=2.5, tile_grid_size=(8, 8)):
    lab = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2LAB)
    l_channel, a_channel, b_channel = cv2.split(lab)
    clahe = cv2.createCLAHE(clipLimit=clip_limit, tileGridSize=tile_grid_size)
    cl = clahe.apply(l_channel)
    merged = cv2.merge((cl, a_channel, b_channel))
    return cv2.cvtColor(merged, cv2.COLOR_LAB2BGR)

def format_display_label(raw_name):
    n = raw_name.lower()
    if any(k in n for k in ["phone", "smartphone"]):
        return "Smartphone"
    elif "mouse" in n:
        return "Mouse"
    elif "keyboard" in n:
        return "Keyboard"
    elif "laptop" in n:
        return "Laptop"
    elif "monitor" in n:
        return "Monitor"
    elif "tablet" in n:
        return "Tablet"
    elif "multimeter" in n:
        return "Multimeter"
    elif "oscilloscope" in n:
        return "Oscilloscope"
    elif "soldering" in n:
        return "Soldering Iron"
    elif "screwdriver" in n:
        return "Screwdriver"
    elif "pliers" in n:
        return "Pliers"
    elif "wrench" in n:
        return "Wrench"
    elif "caliper" in n:
        return "Caliper"
    elif "breadboard" in n:
        return "Breadboard"
    elif "pixhawk" in n or "flight controller" in n:
        return "Pixhawk"
    elif "thruster" in n or "motor" in n or "propeller" in n:
        return "Thruster/Motor"
    elif "esc" in n or "speed controller" in n:
        return "ESC"
    elif "battery" in n or "power bank" in n:
        return "Battery"
    elif "charger" in n or "adapter" in n:
        return "Charger"
    elif "raspberry" in n:
        return "Raspberry Pi"
    elif "enclosure" in n or "acrylic" in n or "tube" in n:
        return "AUV Hull/Tube"
    elif "tether" in n:
        return "AUV Tether"
    elif "cable" in n or "wire" in n:
        return "Cable/Wire"
    elif "pcb" in n or "circuit" in n:
        return "PCB"
    elif "bottle" in n:
        return "Bottle"
    elif "cup" in n or "mug" in n:
        return "Cup"
    elif "person" in n:
        return "Person"
    return raw_name.title()

def auto_detect_camera():
    """Detect Logitech C922 on /dev/video* or fallback to default webcam."""
    candidates = []
    for idx in range(10):
        path = f"/sys/class/video4linux/video{idx}/name"
        if os.path.exists(path):
            try:
                with open(path, "r") as f:
                    name = f.read().strip()
                candidates.append((idx, name))
            except Exception:
                pass
    print("[Camera Discovery] Detected video devices:")
    for idx, name in candidates:
        print(f"  /dev/video{idx}: {name}")

    # Prioritize Logitech C922
    for idx, name in candidates:
        if "c922" in name.lower() or "logitech" in name.lower():
            print(f"[Camera Selection] Selecting Logitech C922 at /dev/video{idx}!")
            return idx

    # Fallback to index 2 (common for external USB) or index 0
    for idx in [2, 0]:
        if os.path.exists(f"/dev/video{idx}"):
            return idx
    return 0

def main():
    print("=" * 70)
    print("      AUV DETECTION CAMERA BENCH TEST: LOGITECH C922 EVALUATION      ")
    print("=" * 70)

    # 1. Initialize PyTorch Device
    device = "cuda:0" if torch.cuda.is_available() else "cpu"
    print(f"[Hardware] Device: {device}")
    if torch.cuda.is_available():
        print(f"[Hardware] GPU Acceleration: {torch.cuda.get_device_name(0)}")

    # 2. Load YOLO26 World
    print(f"[AI Model] Loading YOLO26 World from '{YOLO26_WORLD_WEIGHTS}'...")
    if not os.path.exists(YOLO26_WORLD_WEIGHTS):
        print(f"[Error] Weights file '{YOLO26_WORLD_WEIGHTS}' not found!")
        sys.exit(1)

    model = YOLOWorld(YOLO26_WORLD_WEIGHTS)
    model.set_classes(YOLO26_WORLD_CLASSES)
    model.to(device)
    print(f"[AI Model] Loaded {len(YOLO26_WORLD_CLASSES)} target classes successfully!")

    # 3. Open Logitech C922 Webcam
    cam_index = auto_detect_camera()
    print(f"[Webcam] Opening /dev/video{cam_index} at 1280x720...")
    cap = cv2.VideoCapture(cam_index, cv2.CAP_V4L2 if os.name == 'posix' else cv2.CAP_ANY)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    cap.set(cv2.CAP_PROP_FPS, 30)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    if not cap.isOpened():
        print(f"[Error] Could not open camera on /dev/video{cam_index}! Trying index 0...")
        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            print("[Error] No camera could be opened.")
            sys.exit(1)

    ret, test_frame = cap.read()
    if ret and test_frame is not None:
        h, w = test_frame.shape[:2]
        print(f"[Webcam] Streaming active: {w}x{h} resolution.")
    else:
        print("[Error] Could not read frame from webcam.")
        sys.exit(1)

    # 4. Initialize 8D Kalman Filter
    kf = AUVVisualKalmanFilter(dt=1.0 / 30.0, mode="8D")

    # State variables & flags
    kalman_enabled = True
    split_screen_mode = False   # False: Overlay comparison, True: Side-by-Side comparison
    clahe_enabled = False
    conf_thresh = CONF_THRESHOLD

    # Metrics history (last 20 frames) to compute live jitter (standard deviation of centroid delta)
    raw_history = deque(maxlen=20)
    kf_history = deque(maxlen=20)

    WINDOW_NAME = "AUV Detection Camera Bench Test: Raw vs. 8D Kalman Filter"
    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WINDOW_NAME, 1280, 720)

    print("\n" + "=" * 70)
    print("   BENCH TEST CONTROLS:")
    print("   [k] - Toggle Kalman Filter ON / OFF (Instant comparison)")
    print("   [s] - Toggle Split-Screen Mode (Side-by-Side vs. Single Overlay)")
    print("   [e] - Toggle CLAHE Dynamic Underwater Contrast Enhancement")
    print("   [+] / [-] - Adjust Confidence Threshold (+/- 0.02)")
    print("   [q] - Exit cleanly")
    print("=" * 70 + "\n")

    t_prev = time.perf_counter()
    fps_smooth = 30.0

    while True:
        ret, frame = cap.read()
        if not ret or frame is None:
            time.sleep(0.01)
            continue

        t_now = time.perf_counter()
        dt = max(0.001, t_now - t_prev)
        t_prev = t_now
        fps_smooth = 0.9 * fps_smooth + 0.1 * (1.0 / dt)

        if clahe_enabled:
            frame = apply_clahe(frame)

        h, w = frame.shape[:2]
        center_x, center_y = w // 2, h // 2

        # 1. Kalman Predict Step (Hardware adaptive dt)
        if kalman_enabled:
            kf.predict(dt=dt)

        # 2. Run YOLO26 World Inference on GPU
        results = model.predict(frame, conf=conf_thresh, imgsz=IMG_SIZE, device=device, agnostic_nms=True, verbose=False)[0]

        best_target = None
        max_score = 0

        # Find best candidate target
        for box in results.boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            conf = float(box.conf[0])
            cls_id = int(box.cls[0])
            raw_name = model.names[cls_id]
            pretty_label = format_display_label(raw_name)
            box_label = f"{pretty_label} {conf*100:.0f}%"

            area = (x2 - x1) * (y2 - y1)
            is_priority = (raw_name.lower() in PRIORITY_TARGETS)
            score = area * (10.0 if is_priority else 1.0) * conf
            if score > max_score:
                max_score = score
                best_target = ((x1 + x2) // 2, (y1 + y2) // 2, x1, y1, x2, y2, box_label, conf)

        # Process Tracking
        raw_x, raw_y = None, None
        kf_x, kf_y = None, None
        target_label = "None"
        is_occluded = False

        if best_target is not None:
            cx, cy, x1, y1, x2, y2, target_label, conf = best_target
            raw_x, raw_y = cx, cy
            raw_history.append((raw_x, raw_y))

            if kalman_enabled:
                fx, fy = kf.update_bbox(x1, y1, x2, y2, conf=conf)
                kf_x, kf_y = int(fx), int(fy)
                kf_history.append((kf_x, kf_y))
        else:
            if kalman_enabled and kf.initialized:
                pred_x, pred_y = kf.handle_missing_frame()
                if pred_x is not None:
                    kf_x, kf_y = int(pred_x), int(pred_y)
                    kf_history.append((kf_x, kf_y))
                    is_occluded = True

        # Compute Jitter Metrics (Root Mean Square Displacement of centroid fluctuations)
        raw_jitter = 0.0
        if len(raw_history) >= 2:
            deltas = [math.hypot(raw_history[i][0] - raw_history[i-1][0], raw_history[i][1] - raw_history[i-1][1])
                      for i in range(1, len(raw_history))]
            raw_jitter = np.std(deltas)

        kf_jitter = 0.0
        if len(kf_history) >= 2:
            deltas_kf = [math.hypot(kf_history[i][0] - kf_history[i-1][0], kf_history[i][1] - kf_history[i-1][1])
                         for i in range(1, len(kf_history))]
            kf_jitter = np.std(deltas_kf)

        jitter_reduction = 0.0
        if raw_jitter > 0.001:
            jitter_reduction = max(0.0, (raw_jitter - kf_jitter) / raw_jitter * 100.0)

        # -------------------------------------------------------------
        # RENDER VISUALIZATION
        # -------------------------------------------------------------
        if split_screen_mode:
            # === SIDE-BY-SIDE SPLIT SCREEN ===
            # Left: RAW DETECTION (WITHOUT KALMAN FILTER)
            frame_raw = frame.copy()
            # Right: KALMAN FILTER (WITH KALMAN FILTER)
            frame_kf = frame.copy()

            # --- Left: Raw YOLO Drawing ---
            cv2.putText(frame_raw, "[WITHOUT KALMAN FILTER: RAW YOLO]", (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            cv2.putText(frame_raw, f"RAW JITTER: +/-{raw_jitter:.1f} px", (20, 75),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            if best_target is not None:
                _, _, x1, y1, x2, y2, t_lbl, _ = best_target
                cv2.rectangle(frame_raw, (x1, y1), (x2, y2), (0, 0, 255), 2)
                cv2.circle(frame_raw, (raw_x, raw_y), 6, (0, 0, 255), -1)
                cv2.putText(frame_raw, f"{t_lbl} [RAW CHATTER]", (x1, max(20, y1 - 8)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2)
            else:
                cv2.putText(frame_raw, "TARGET LOST / OCCLUDED (ZERO SIGNAL)", (20, 115),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

            # --- Right: Kalman Filter Drawing ---
            cv2.putText(frame_kf, "[WITH 8D KALMAN FILTER]", (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(frame_kf, f"KALMAN JITTER: +/-{kf_jitter:.1f} px (STABILIZED)", (20, 75),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            if kf_x is not None and kf_y is not None:
                bbox_smooth = kf.get_bbox()
                color = (255, 255, 0) if is_occluded else (0, 255, 0)
                if bbox_smooth is not None:
                    sx1, sy1, sx2, sy2, sw, sh = bbox_smooth
                    cv2.rectangle(frame_kf, (sx1, sy1), (sx2, sy2), color, 2)
                cv2.circle(frame_kf, (kf_x, kf_y), 6, color, -1)

                vx, vy = kf.get_velocity()
                cv2.arrowedLine(frame_kf, (kf_x, kf_y), (int(kf_x + vx * 0.3), int(kf_y + vy * 0.3)), (0, 255, 255), 2)

                lbl = f"DEAD-RECKONING ({kf.missed_frames}f lost)" if is_occluded else f"{target_label} [8D KF SMOOTH]"
                cv2.putText(frame_kf, lbl, (max(10, kf_x - 70), max(25, kf_y - 12)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)
            else:
                cv2.putText(frame_kf, "SEARCHING...", (20, 115), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            # Resize both to half width and concatenate
            half_w = w // 2
            half_raw = cv2.resize(frame_raw, (half_w, h))
            half_kf = cv2.resize(frame_kf, (half_w, h))
            display = np.hstack((half_raw, half_kf))
            # Draw dividing vertical line
            cv2.line(display, (half_w, 0), (half_w, h), (255, 255, 255), 2)

        else:
            # === OVERLAY COMPARISON MODE ===
            display = frame

            # Center Crosshair
            cv2.line(display, (center_x - 15, center_y), (center_x + 15, center_y), (100, 100, 100), 1)
            cv2.line(display, (center_x, center_y - 15), (center_x, center_y + 15), (100, 100, 100), 1)

            # 1. Draw Raw Bounding Box (Red, thin)
            if best_target is not None:
                _, _, x1, y1, x2, y2, t_lbl, _ = best_target
                cv2.rectangle(display, (x1, y1), (x2, y2), (0, 0, 255), 1)
                cv2.circle(display, (raw_x, raw_y), 4, (0, 0, 255), -1)
                cv2.putText(display, f"RAW: {t_lbl}", (x1, max(15, y1 - 6)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 1)

            # 2. Draw Kalman Filtered Estimate (Bright Green / Cyan)
            if kalman_enabled and kf_x is not None and kf_y is not None:
                bbox_smooth = kf.get_bbox()
                color = (255, 255, 0) if is_occluded else (0, 255, 0)
                if bbox_smooth is not None:
                    sx1, sy1, sx2, sy2, sw, sh = bbox_smooth
                    cv2.rectangle(display, (sx1, sy1), (sx2, sy2), color, 2)
                cv2.circle(display, (kf_x, kf_y), 6, color, -1)

                # Line connecting center crosshair to Kalman Target
                cv2.line(display, (center_x, center_y), (kf_x, kf_y), (0, 255, 255), 2)

                # Velocity vector arrow
                vx, vy = kf.get_velocity()
                cv2.arrowedLine(display, (kf_x, kf_y), (int(kf_x + vx * 0.3), int(kf_y + vy * 0.3)), (0, 255, 255), 2)

                # Target area & range expansion indicator
                area, scale_rate = kf.get_scale_rates()
                approach = "APPROACHING" if scale_rate > 300 else ("RETREATING" if scale_rate < -300 else "HOLDING")

                status_txt = f"DEAD-RECKONING ({kf.missed_frames}f)" if is_occluded else f"8D KF: {approach} (v={math.hypot(vx, vy):.1f}px/s)"
                cv2.putText(display, status_txt, (max(10, kf_x - 70), max(25, kf_y - 12)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)

            # Top Dashboard Banner
            kf_status_badge = "[8D KALMAN: ON]" if kalman_enabled else "[8D KALMAN: OFF]"
            split_badge = "[VIEW: SPLIT]" if split_screen_mode else "[VIEW: OVERLAY]"
            clahe_badge = "[CLAHE: ON]" if clahe_enabled else "[CLAHE: OFF]"

            cv2.rectangle(display, (10, 10), (1270, 75), (20, 20, 20), -1)
            cv2.putText(display, f"LOGITECH C922 BENCH TEST | {kf_status_badge} {split_badge} {clahe_badge} | FPS: {fps_smooth:.1f}",
                        (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

            metric_color = (0, 255, 0) if jitter_reduction > 50 else (0, 200, 255)
            cv2.putText(display, f"RAW JITTER: +/-{raw_jitter:.1f} px (RED)  |  KALMAN JITTER: +/-{kf_jitter:.1f} px (GREEN)  |  NOISE REDUCTION: {jitter_reduction:.1f}%",
                        (20, 65), cv2.FONT_HERSHEY_SIMPLEX, 0.55, metric_color, 2)

        cv2.imshow(WINDOW_NAME, display)

        # Handle Keyboard Inputs
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            print("[Bench Test] Exiting...")
            break
        elif key == ord('k'):
            kalman_enabled = not kalman_enabled
            print(f"[Toggle] Kalman Filter: {'ENABLED' if kalman_enabled else 'DISABLED'}")
        elif key == ord('s'):
            split_screen_mode = not split_screen_mode
            print(f"[Toggle] Split-Screen Mode: {'SIDE-BY-SIDE' if split_screen_mode else 'OVERLAY'}")
        elif key == ord('e'):
            clahe_enabled = not clahe_enabled
            print(f"[Toggle] CLAHE Contrast Enhancement: {'ON' if clahe_enabled else 'OFF'}")
        elif key in [ord('+'), ord('=')]:
            conf_thresh = min(0.90, conf_thresh + 0.02)
            print(f"[Threshold] Confidence: {conf_thresh:.2f}")
        elif key in [ord('-'), ord('_')]:
            conf_thresh = max(0.05, conf_thresh - 0.02)
            print(f"[Threshold] Confidence: {conf_thresh:.2f}")

    cap.release()
    cv2.destroyAllWindows()
    print("[Bench Test] Test completed successfully.")

if __name__ == "__main__":
    main()
