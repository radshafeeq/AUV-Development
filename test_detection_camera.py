#!/usr/bin/env python3
"""
AUV Detection Camera Bench Test: Ultra-Responsive Logitech C922 Evaluation (v2.0)
=================================================================================
Optimizations & Fixes:
1. ZERO-LATENCY VIDEO PIPELINE:
   - Dedicated background thread with MJPG hardware decoding at 60 FPS
   - Kernel frame buffer continuously flushed to eliminate all capture lag
2. ULTRA-AGILE KALMAN FILTER (ZERO PHASE LAG):
   - Tuned process noise covariance (qs = 1.0) so the green box snaps instantly
     to target motion without visual lag behind the red box
   - Toggle Agility Mode via [a] (Agile Zero-Lag vs. Heavy Smooth)
3. TARGET LOCKING & OCCLUSION FIX:
   - "Hand" and "Person" treated as occluders rather than stealing target focus
   - Bounding box target persistence with IoU tracking
   - Putting your hand in front now correctly triggers DEAD-RECKONING (Cyan Box)
   - Left-Click on any object or press [t] to cycle/lock target!
4. HUMAN-INTUITIVE METRIC VELOCITY:
   - Displays real-world speed in cm/s and m/s with directional indicators
     e.g., "Speed: 28.5 cm/s (0.29 m/s) [Right ->]"
5. FAST GPU INFERENCE:
   - Default 640px tensor (~8 ms on RTX 4070 GPU) for butter-smooth 60+ FPS
   - Toggle to 1024px via [i]
"""

import os
import sys
import time
import math
import threading
from collections import deque
import cv2
import numpy as np
import torch
from ultralytics import YOLOWorld

# Import the optimized AUV Kalman Filter
from kalman_filter import AUVVisualKalmanFilter

# ==========================================
# CONFIGURATION & VOCABULARY
# ==========================================
YOLO26_WORLD_WEIGHTS = "weights/yolo26_world.pt"

# Full open-vocabulary classes
YOLO26_WORLD_CLASSES = [
    # --- Benchtop Electronics & Gadgets ---
    "laptop", "computer monitor", "smartphone", "cell phone",
    "computer mouse", "mouse", "computer keyboard", "keyboard", "tablet",
    "bottle", "water bottle", "cup", "mug", "notebook",

    # --- Mechatronics Lab Tools & Hardware ---
    "digital multimeter", "multimeter", "oscilloscope",
    "soldering iron", "wire stripper", "screwdriver", "pliers", "wrench",
    "caliper", "vernier caliper", "ruler", "scissors", "pen",
    "breadboard", "jumper wire", "heat shrink tube",

    # --- AUV Internal & Subsea Robotics Hardware ---
    "pixhawk", "flight controller",
    "bldc motor", "underwater thruster", "thruster", "propeller",
    "electronic speed controller", "esc",
    "lipo battery", "battery", "power bank", "charger", "power adapter",
    "ethernet cable", "tether", "cable", "wire",
    "printed circuit board", "circuit board", "pcb", "raspberry pi",
    "watertight enclosure", "acrylic tube",

    # --- Subsea Targets & Marine Inspection ---
    "underwater buoy", "marker buoy", "buoy",
    "underwater gate", "navigation gate", "transit gate",
    "torpedo target", "docking station",
    "subsea pipe", "underwater pipeline", "pipe",
    "subsea flange", "subsea valve", "underwater cable",

    # --- Ambient / Occluding Entities ---
    "person", "hand"
]

# Objects to actively track (excludes "hand" and "person" so hands act as occluders)
VALID_TRACKING_TARGETS = set(YOLO26_WORLD_CLASSES) - {"person", "hand"}

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
    elif "multimeter" in n:
        return "Multimeter"
    elif "soldering" in n:
        return "Soldering Iron"
    elif "screwdriver" in n:
        return "Screwdriver"
    elif "pliers" in n:
        return "Pliers"
    elif "breadboard" in n:
        return "Breadboard"
    elif "pixhawk" in n or "flight controller" in n:
        return "Pixhawk"
    elif "thruster" in n or "motor" in n or "propeller" in n:
        return "Thruster/Motor"
    elif "esc" in n or "speed controller" in n:
        return "ESC"
    elif "battery" in n:
        return "Battery"
    elif "raspberry" in n:
        return "Raspberry Pi"
    elif "enclosure" in n or "acrylic" in n:
        return "AUV Tube"
    elif "bottle" in n:
        return "Bottle"
    elif "cup" in n or "mug" in n:
        return "Cup"
    elif "hand" in n:
        return "Hand (Occluder)"
    elif "person" in n:
        return "Person"
    return raw_name.title()

class ThreadedWebcamCapture:
    """High-speed threaded frame grabber with hardware MJPG to eliminate camera buffer lag."""
    def __init__(self, src=2, width=1280, height=720, fps=60):
        self.cap = cv2.VideoCapture(src, cv2.CAP_V4L2 if os.name == 'posix' else cv2.CAP_ANY)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, fps)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        self.lock = threading.Lock()
        self.ret = False
        self.frame = None
        self.stopped = False

        if self.cap.isOpened():
            self.thread = threading.Thread(target=self._reader, daemon=True)
            self.thread.start()
            # Wait for first frame
            for _ in range(20):
                if self.frame is not None:
                    break
                time.sleep(0.05)

    def _reader(self):
        while not self.stopped:
            ret, frame = self.cap.read()
            if ret and frame is not None:
                with self.lock:
                    self.ret = ret
                    self.frame = frame
            else:
                time.sleep(0.002)

    def read(self):
        with self.lock:
            return self.ret, (self.frame.copy() if self.frame is not None else None)

    def isOpened(self):
        return self.cap.isOpened()

    def release(self):
        self.stopped = True
        if hasattr(self, 'cap'):
            self.cap.release()

def compute_iou(boxA, boxB):
    """Compute Intersection-over-Union (IoU) between two bounding boxes."""
    xA = max(boxA[0], boxB[0])
    yA = max(boxA[1], boxB[1])
    xB = min(boxA[2], boxB[2])
    yB = min(boxA[3], boxB[3])
    interArea = max(0, xB - xA) * max(0, yB - yA)
    boxAArea = max(1.0, (boxA[2] - boxA[0]) * (boxA[3] - boxA[1]))
    boxBArea = max(1.0, (boxB[2] - boxB[0]) * (boxB[3] - boxB[1]))
    return interArea / float(boxAArea + boxBArea - interArea)

def auto_detect_camera():
    """Detect Logitech C922 or fallback to available camera."""
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
    for idx, name in candidates:
        if "c922" in name.lower() or "logitech" in name.lower():
            return idx
    for idx in [2, 0]:
        if os.path.exists(f"/dev/video{idx}"):
            return idx
    return 0

# Mouse callback state for click-to-lock
click_coords = None
def on_mouse_click(event, x, y, flags, param):
    global click_coords
    if event == cv2.EVENT_LBUTTONDOWN:
        click_coords = (x, y)

def main():
    global click_coords
    print("=" * 75)
    print("   AUV BENCH TEST v2.0: ULTRA-RESPONSIVE ZERO-LAG KALMAN EVALUATION   ")
    print("=" * 75)

    device = "cuda:0" if torch.cuda.is_available() else "cpu"
    print(f"[Hardware] PyTorch Device: {device}")
    if torch.cuda.is_available():
        print(f"[Hardware] GPU Acceleration: {torch.cuda.get_device_name(0)}")

    print(f"[AI Model] Loading YOLO26 World from '{YOLO26_WORLD_WEIGHTS}'...")
    model = YOLOWorld(YOLO26_WORLD_WEIGHTS)
    model.set_classes(YOLO26_WORLD_CLASSES)
    model.to(device)
    print(f"[AI Model] Target dictionary ready ({len(YOLO26_WORLD_CLASSES)} classes).")

    cam_index = auto_detect_camera()
    print(f"[Camera] Initializing Threaded MJPG stream on /dev/video{cam_index}...")
    stream = ThreadedWebcamCapture(src=cam_index, width=1280, height=720, fps=60)
    if not stream.isOpened():
        print("[Error] Failed to open Logitech C922! Trying index 0...")
        stream = ThreadedWebcamCapture(src=0, width=1280, height=720, fps=60)

    # Kalman Filter initialization with AGILE tuning (qs=1.0 for instant zero-lag response)
    current_qs = 1.0
    kf = AUVVisualKalmanFilter(dt=1.0 / 60.0, mode="8D", qs=current_qs, r_var=0.15, gate_px=450.0)

    # Runtime toggles
    kalman_enabled = True
    split_screen_mode = False
    clahe_enabled = False
    current_imgsz = 640  # 640px = ~8ms inference for instant 60 FPS
    conf_thresh = 0.15

    # Target persistence / locking state
    locked_label = None
    locked_bbox = None

    # Jitter evaluation deques
    raw_history = deque(maxlen=20)
    kf_history = deque(maxlen=20)

    WINDOW_NAME = "AUV Bench Test v2.0 (Zero-Lag Logitech C922 + 8D Kalman Filter)"
    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WINDOW_NAME, 1280, 720)
    cv2.setMouseCallback(WINDOW_NAME, on_mouse_click)

    print("\n" + "=" * 75)
    print("   INTERACTIVE CONTROLS:")
    print("   [Left-Click] - Click directly on any object in video to LOCK onto it!")
    print("   [t]          - Cycle / Unlock Target (Auto-locks to best target)")
    print("   [k]          - Toggle Kalman Filter ON / OFF (Instant comparison)")
    print("   [s]          - Toggle Split-Screen (Side-by-Side vs. Overlay)")
    print("   [a]          - Toggle Agility: Agile Zero-Lag (qs=1.0) vs Heavy Smooth (qs=0.08)")
    print("   [i]          - Toggle Resolution: 640px (Ultra-Fast) vs 1024px (High-Res)")
    print("   [e]          - Toggle CLAHE Dynamic Underwater Contrast Enhancement")
    print("   [+] / [-]    - Adjust Confidence Threshold (+/- 0.02)")
    print("   [q]          - Exit cleanly")
    print("=" * 75 + "\n")

    t_prev = time.perf_counter()
    fps_smooth = 60.0

    while True:
        ret, frame = stream.read()
        if not ret or frame is None:
            time.sleep(0.005)
            continue

        t_now = time.perf_counter()
        dt = max(0.001, t_now - t_prev)
        t_prev = t_now
        fps_smooth = 0.92 * fps_smooth + 0.08 * (1.0 / dt)

        if clahe_enabled:
            frame = apply_clahe(frame)

        h, w = frame.shape[:2]
        center_x, center_y = w // 2, h // 2

        # 1. Kalman Predict Step
        if kalman_enabled:
            kf.predict(dt=dt)

        # 2. Fast YOLO26 World Inference on GPU
        results = model.predict(frame, conf=conf_thresh, imgsz=current_imgsz, device=device, agnostic_nms=True, verbose=False)[0]

        # Parse all detections
        candidate_boxes = []
        for box in results.boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            conf = float(box.conf[0])
            cls_id = int(box.cls[0])
            raw_name = model.names[cls_id].lower()
            pretty_label = format_display_label(raw_name)
            is_valid_target = (raw_name in VALID_TRACKING_TARGETS)
            candidate_boxes.append({
                "bbox": (x1, y1, x2, y2),
                "center": ((x1 + x2) // 2, (y1 + y2) // 2),
                "conf": conf,
                "label": pretty_label,
                "raw_name": raw_name,
                "is_valid": is_valid_target,
                "area": (x2 - x1) * (y2 - y1)
            })

        # Check Mouse Click to Lock onto an object
        if click_coords is not None:
            cx_click, cy_click = click_coords
            clicked_any = False
            for cand in candidate_boxes:
                x1, y1, x2, y2 = cand["bbox"]
                if x1 <= cx_click <= x2 and y1 <= cy_click <= y2 and cand["is_valid"]:
                    locked_label = cand["raw_name"]
                    locked_bbox = cand["bbox"]
                    clicked_any = True
                    print(f"[Target Lock] Manually locked onto: {cand['label']}!")
                    break
            if not clicked_any:
                print("[Target Lock] Unlocked. Returning to auto-select.")
                locked_label = None
                locked_bbox = None
            click_coords = None

        # Determine Best Target with IoU Continuity
        best_cand = None
        if locked_label is not None and locked_bbox is not None:
            # Look for the locked object with highest IoU / spatial proximity
            best_iou = -1.0
            for cand in candidate_boxes:
                if cand["raw_name"] == locked_label:
                    iou = compute_iou(locked_bbox, cand["bbox"])
                    if iou > best_iou:
                        best_iou = iou
                        best_cand = cand
            # If found with positive overlap, update locked_bbox
            if best_cand is not None and best_iou > 0.05:
                locked_bbox = best_cand["bbox"]
            else:
                # Target is occluded or lost!
                best_cand = None
        else:
            # Auto-selection: Pick largest valid non-hand/person object
            max_score = 0
            for cand in candidate_boxes:
                if cand["is_valid"]:
                    score = cand["area"] * cand["conf"]
                    if score > max_score:
                        max_score = score
                        best_cand = cand
            if best_cand is not None:
                locked_label = best_cand["raw_name"]
                locked_bbox = best_cand["bbox"]

        # Process Target Coordinates & Kalman Update
        raw_x, raw_y = None, None
        kf_x, kf_y = None, None
        target_display_name = "None"
        is_occluded = False

        if best_cand is not None:
            cx, cy = best_cand["center"]
            x1, y1, x2, y2 = best_cand["bbox"]
            conf = best_cand["conf"]
            raw_x, raw_y = cx, cy
            target_display_name = best_cand["label"]
            raw_history.append((raw_x, raw_y))

            if kalman_enabled:
                fx, fy = kf.update_bbox(x1, y1, x2, y2, conf=conf)
                kf_x, kf_y = int(fx), int(fy)
                kf_history.append((kf_x, kf_y))
        else:
            # Target is occluded (e.g., hand in front) -> TRIGGER DEAD-RECKONING!
            if kalman_enabled and kf.initialized:
                pred_x, pred_y = kf.handle_missing_frame()
                if pred_x is not None:
                    kf_x, kf_y = int(pred_x), int(pred_y)
                    kf_history.append((kf_x, kf_y))
                    is_occluded = True
                    target_display_name = format_display_label(locked_label if locked_label else "Target")

        # Jitter Computation
        raw_jitter = np.std([math.hypot(raw_history[i][0] - raw_history[i-1][0], raw_history[i][1] - raw_history[i-1][1])
                             for i in range(1, len(raw_history))]) if len(raw_history) >= 2 else 0.0
        kf_jitter = np.std([math.hypot(kf_history[i][0] - kf_history[i-1][0], kf_history[i][1] - kf_history[i-1][1])
                            for i in range(1, len(kf_history))]) if len(kf_history) >= 2 else 0.0
        stabilization_pct = max(0.0, (raw_jitter - kf_jitter) / raw_jitter * 100.0) if raw_jitter > 0.01 else 0.0

        # Physical Metric Velocity Computation (Assumes standard ~80 cm working standoff)
        # f_x ~ 900 px for 70.4 deg H-FOV on 1280x720
        vx_px, vy_px = kf.get_velocity() if (kalman_enabled and kf.initialized) else (0.0, 0.0)
        est_distance_cm = 80.0
        cm_per_px = (2.0 * est_distance_cm * math.tan(math.radians(35.2))) / 1280.0  # ~0.088 cm/px
        vx_metric_cms = vx_px * cm_per_px
        vy_metric_cms = -vy_px * cm_per_px  # Invert so positive is upwards
        speed_total_cms = math.hypot(vx_metric_cms, vy_metric_cms)
        speed_total_ms = speed_total_cms / 100.0

        # Cardinal Direction Description
        horiz_dir = "Right ->" if vx_metric_cms > 3.0 else ("Left <-" if vx_metric_cms < -3.0 else "Still")
        vert_dir = "Up ^" if vy_metric_cms > 3.0 else ("Down v" if vy_metric_cms < -3.0 else "Still")

        # -------------------------------------------------------------
        # VISUALIZATION
        # -------------------------------------------------------------
        if split_screen_mode:
            # === SIDE-BY-SIDE SPLIT SCREEN ===
            frame_raw = frame.copy()
            frame_kf = frame.copy()

            # Left: RAW YOLO
            cv2.putText(frame_raw, "[WITHOUT KALMAN: RAW YOLO]", (20, 35),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2)
            cv2.putText(frame_raw, f"RAW JITTER: +/-{raw_jitter:.1f} px", (20, 65),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2)
            if best_cand is not None:
                x1, y1, x2, y2 = best_cand["bbox"]
                cv2.rectangle(frame_raw, (x1, y1), (x2, y2), (0, 0, 255), 2)
                cv2.circle(frame_raw, (raw_x, raw_y), 6, (0, 0, 255), -1)
                cv2.putText(frame_raw, f"{target_display_name} (TWITCHING)", (x1, max(20, y1 - 8)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2)
            else:
                cv2.putText(frame_raw, "SIGNAL LOST (ZERO DEAD-RECKONING)", (20, 100),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

            # Right: KALMAN FILTER
            cv2.putText(frame_kf, "[WITH 8D KALMAN FILTER]", (20, 35),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2)
            cv2.putText(frame_kf, f"KALMAN JITTER: +/-{kf_jitter:.1f} px (STABLE)", (20, 65),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)
            if kf_x is not None and kf_y is not None:
                bbox_smooth = kf.get_bbox()
                col = (255, 255, 0) if is_occluded else (0, 255, 0)
                if bbox_smooth is not None:
                    sx1, sy1, sx2, sy2, _, _ = bbox_smooth
                    cv2.rectangle(frame_kf, (sx1, sy1), (sx2, sy2), col, 2)
                cv2.circle(frame_kf, (kf_x, kf_y), 6, col, -1)
                cv2.arrowedLine(frame_kf, (kf_x, kf_y), (int(kf_x + vx_px * 0.25), int(kf_y + vy_px * 0.25)), (0, 255, 255), 2)
                tag = f"DEAD-RECKONING ({kf.missed_frames}f lost)" if is_occluded else f"{target_display_name} [SMOOTH]"
                cv2.putText(frame_kf, tag, (max(10, kf_x - 70), max(25, kf_y - 12)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, col, 2)
            else:
                cv2.putText(frame_kf, "SEARCHING...", (20, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            half_w = w // 2
            display = np.hstack((cv2.resize(frame_raw, (half_w, h)), cv2.resize(frame_kf, (half_w, h))))
            cv2.line(display, (half_w, 0), (half_w, h), (255, 255, 255), 2)

        else:
            # === OVERLAY COMPARISON MODE ===
            display = frame

            # Center Crosshair
            cv2.line(display, (center_x - 12, center_y), (center_x + 12, center_y), (120, 120, 120), 1)
            cv2.line(display, (center_x, center_y - 12), (center_x, center_y + 12), (120, 120, 120), 1)

            # Draw Ambient/Hand Detections in faint orange to show it sees the hand without jumping to it
            for cand in candidate_boxes:
                if not cand["is_valid"]:
                    x1, y1, x2, y2 = cand["bbox"]
                    cv2.rectangle(display, (x1, y1), (x2, y2), (0, 140, 255), 1)
                    cv2.putText(display, cand["label"], (x1, max(15, y1 - 4)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.40, (0, 140, 255), 1)

            # 1. Draw Raw Bounding Box (Thin Red)
            if best_cand is not None:
                x1, y1, x2, y2 = best_cand["bbox"]
                cv2.rectangle(display, (x1, y1), (x2, y2), (0, 0, 255), 1)
                cv2.circle(display, (raw_x, raw_y), 4, (0, 0, 255), -1)
                cv2.putText(display, f"RAW: {target_display_name}", (x1, max(15, y1 - 6)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 1)

            # 2. Draw 8D Kalman Filtered Box (Bright Green / Cyan)
            if kalman_enabled and kf_x is not None and kf_y is not None:
                bbox_smooth = kf.get_bbox()
                col = (255, 255, 0) if is_occluded else (0, 255, 0)
                if bbox_smooth is not None:
                    sx1, sy1, sx2, sy2, _, _ = bbox_smooth
                    cv2.rectangle(display, (sx1, sy1), (sx2, sy2), col, 2)
                cv2.circle(display, (kf_x, kf_y), 6, col, -1)

                # Heading line from center
                cv2.line(display, (center_x, center_y), (kf_x, kf_y), (0, 255, 255), 2)

                # Velocity vector arrow
                cv2.arrowedLine(display, (kf_x, kf_y), (int(kf_x + vx_px * 0.25), int(kf_y + vy_px * 0.25)), (0, 255, 255), 2)

                lbl = f"DEAD-RECKONING ({kf.missed_frames}f behind hand)" if is_occluded else f"8D KF: {target_display_name}"
                cv2.putText(display, lbl, (max(10, kf_x - 70), max(25, kf_y - 10)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, col, 2)

            # Top Dashboard Banner
            kf_badge = "[8D KALMAN: ON]" if kalman_enabled else "[8D KALMAN: OFF]"
            agility_badge = f"[AGILITY: {'FAST/ZERO-LAG' if current_qs >= 0.5 else 'HEAVY-SMOOTH'}]"
            res_badge = f"[{current_imgsz}px @ {fps_smooth:.0f}FPS]"
            lock_badge = f"[LOCKED: {target_display_name}]" if locked_label else "[TARGET: AUTO]"

            cv2.rectangle(display, (10, 10), (1270, 80), (15, 15, 15), -1)
            cv2.putText(display, f"LOGITECH C922 BENCH TEST v2.0 | {kf_badge} {agility_badge} {res_badge} {lock_badge}",
                        (20, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)

            # Metric velocity readout
            speed_txt = f"VELOCITY: {speed_total_cms:.1f} cm/s ({speed_total_ms:.2f} m/s) [{horiz_dir}, {vert_dir}]"
            jitter_txt = f"JITTER: RAW +/-{raw_jitter:.1f}px -> KF +/-{kf_jitter:.1f}px ({stabilization_pct:.0f}% STABLE)"
            cv2.putText(display, f"{speed_txt}  |  {jitter_txt}",
                        (20, 62), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (0, 255, 0) if stabilization_pct > 40 else (0, 200, 255), 2)

        cv2.imshow(WINDOW_NAME, display)

        # Keyboard event loop
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('k'):
            kalman_enabled = not kalman_enabled
            print(f"[Toggle] Kalman Filter: {'ON' if kalman_enabled else 'OFF'}")
        elif key == ord('s'):
            split_screen_mode = not split_screen_mode
            print(f"[Toggle] View Mode: {'SPLIT SCREEN' if split_screen_mode else 'OVERLAY'}")
        elif key == ord('a'):
            # Toggle Agility between Fast Zero-Lag and Heavy Smooth
            if current_qs > 0.5:
                current_qs = 0.08
                print("[Agility] Switched to HEAVY SMOOTH Mode (qs = 0.08).")
            else:
                current_qs = 1.0
                print("[Agility] Switched to AGILE ZERO-LAG Mode (qs = 1.0).")
            kf = AUVVisualKalmanFilter(dt=1.0 / 60.0, mode="8D", qs=current_qs, r_var=0.15, gate_px=450.0)
        elif key == ord('i'):
            current_imgsz = 1024 if current_imgsz == 640 else 640
            print(f"[Resolution] Switched inference resolution to {current_imgsz}px.")
        elif key == ord('t'):
            locked_label = None
            locked_bbox = None
            print("[Target] Reset target lock. Re-locking to next target.")
        elif key == ord('e'):
            clahe_enabled = not clahe_enabled
            print(f"[Toggle] CLAHE: {'ON' if clahe_enabled else 'OFF'}")
        elif key in [ord('+'), ord('=')]:
            conf_thresh = min(0.90, conf_thresh + 0.02)
            print(f"[Confidence] Threshold: {conf_thresh:.2f}")
        elif key in [ord('-'), ord('_')]:
            conf_thresh = max(0.05, conf_thresh - 0.02)
            print(f"[Confidence] Threshold: {conf_thresh:.2f}")

    stream.release()
    cv2.destroyAllWindows()
    print("[Bench Test] Finished.")

if __name__ == "__main__":
    main()
