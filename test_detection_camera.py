#!/usr/bin/env python3
"""
AUV Detection Camera Bench Test: Ultra-Fidelity Logitech C922 Evaluation (v2.5)
=================================================================================
Optimizations & Enhancements:
1. NATIVE FULL HD 1080p DEFAULT SENSOR MODE:
   - Defaults to Full HD 1080p (1920x1080 @ 30 FPS, MJPG) to match 2.5K/4K displays with zero upscaling softness.
   - 2.25x higher pixel count reveals fine text, micro-components, sharp edges, and authentic surface textures.
2. CALIBRATED HARDWARE ISP SHARPNESS (140):
   - Reduced hardware unsharp-masking from 170 to 140, eliminating digital edge ringing (halos) and amplified chroma noise.
3. CLEAN VIEW MODE ([h] / [HUD: ON/OFF] BUTTON):
   - Instantly hides all bounding boxes, banners, and overlays to inspect pure optical sensor output.
4. ON-SCREEN RESOLUTION TOGGLE BUTTON:
   - Interactive [RES: 1080p] / [RES: 720p] button to switch modes dynamically on the fly.
5. PRECISION TARGET SELECTION (CLICK-TO-LOCK FIX):
   - Smallest-area-first selection: When multiple bounding boxes overlap (e.g. Laptop
     sitting on a Table, or Smartphone held by a Person), clicking ALWAYS selects the
     specific object (Laptop) rather than the parent container (Table / Person).
6. IRONCLAD MANUAL LOCKING (PREVENTS SWITCHING TO PERSON):
   - Once locked onto an object (e.g. Laptop), the tracker NEVER jumps to Person or another object.
   - During autofocus sweeps or momentary occlusion, the Kalman filter DEAD-RECKONS on the target
     (Cyan Box) and re-acquires it immediately once focus settles.
7. SYNCHRONIZED MANUAL FOCUS SLIDER:
   - Real-time matching between on-screen slider and autofocus motor.
   - Clickable [MODE: AUTO / MANUAL] and [AUTO FOCUS] buttons.
   - Quick presets [ROOM 15] and [DESK 40].
"""

import os
import sys
import time
import math
import fcntl
import struct
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

    # --- Common Room & Lab Entities ---
    "person", "chair", "table", "backpack", "camera", "tripod", "hand"
]

# Objects to actively track (hand acts as occluder for dead-reckoning)
VALID_TRACKING_TARGETS = set(YOLO26_WORLD_CLASSES) - {"hand"}

# Priority weights for auto-selection mode (heavily favors benchtop items over room background)
OBJECT_PRIORITY = {
    # High Priority: Bench Electronics & AUV Hardware
    "laptop": 3.0, "computer monitor": 2.5, "bottle": 2.5, "water bottle": 2.5,
    "mouse": 2.2, "computer mouse": 2.2, "smartphone": 2.2, "cell phone": 2.2,
    "multimeter": 3.0, "digital multimeter": 3.0, "oscilloscope": 3.0,
    "keyboard": 2.0, "computer keyboard": 2.0, "tablet": 2.2,
    "pixhawk": 3.5, "flight controller": 3.5, "raspberry pi": 3.5,
    "bldc motor": 3.5, "thruster": 3.5, "underwater thruster": 3.5,
    "esc": 3.0, "electronic speed controller": 3.0, "battery": 3.0, "lipo battery": 3.0,
    "power bank": 2.5, "charger": 2.5, "power adapter": 2.5,
    "circuit board": 3.0, "pcb": 3.0, "printed circuit board": 3.0,
    "watertight enclosure": 3.0, "acrylic tube": 3.0,
    "cup": 2.0, "mug": 2.0, "notebook": 1.8, "screwdriver": 2.2, "pliers": 2.2,

    # Low Priority: Ambient Room & Furniture
    "camera": 1.0, "tripod": 0.8, "backpack": 0.6, "chair": 0.2, "table": 0.1,
    # Person has very low auto-selection priority so it NEVER steals focus from benchtop items
    "person": 0.05,
}


# ==========================================
# V4L2 HARDWARE CAMERA CONTROLLER
# ==========================================
class V4L2HardwareController:
    """Direct V4L2 hardware ISP controller for Logitech C922 (Focus, Sharpness, Exposure)."""
    VIDIOC_S_CTRL = 0xc008561c
    VIDIOC_G_CTRL = 0xc008561b

    # V4L2 Control IDs
    CID_FOCUS_AUTO = 0x009a090c       # Focus, Automatic Continuous (0=off, 1=on)
    CID_FOCUS_ABSOLUTE = 0x009a090a   # Focus, Absolute (0..250, step 5)
    CID_SHARPNESS = 0x0098091b        # Sharpness (0..255, default 128)
    CID_BRIGHTNESS = 0x00980900       # Brightness (0..255)
    CID_CONTRAST = 0x00980901         # Contrast (0..255)
    CID_AUTO_EXPOSURE = 0x009a0901    # Auto Exposure (1=manual, 3=auto)
    CID_EXPOSURE_ABS = 0x009a0902     # Exposure Time, Absolute (3..2047)

    def __init__(self, dev_index=2):
        self.dev_path = f"/dev/video{dev_index}"
        self.fd = None
        self.current_focus_cache = 15
        self.is_auto_focus = False
        self.open_device()

        # Initialize to razor-sharp room standoff settings
        self.set_autofocus(False)
        self.set_focus(15)
        self.set_sharpness(140)

    def open_device(self):
        try:
            if os.path.exists(self.dev_path):
                self.fd = os.open(self.dev_path, os.O_RDWR | os.O_NONBLOCK)
                print(f"[V4L2 Hardware] Connected to {self.dev_path} for direct ISP register control.")
        except Exception as e:
            print(f"[V4L2 Hardware] Warning: Could not open {self.dev_path} for hardware control: {e}")
            self.fd = None

    def _set_ctrl(self, cid, value):
        if self.fd is None:
            return False
        try:
            buf = bytearray(struct.pack('=Ii', cid, int(value)))
            fcntl.ioctl(self.fd, self.VIDIOC_S_CTRL, buf)
            return True
        except Exception:
            return False

    def _get_ctrl(self, cid):
        if self.fd is None:
            return None
        try:
            buf = bytearray(struct.pack('=Ii', cid, 0))
            fcntl.ioctl(self.fd, self.VIDIOC_G_CTRL, buf)
            _, val = struct.unpack('=Ii', buf)
            return val
        except Exception:
            return None

    def set_autofocus(self, enable: bool):
        self.is_auto_focus = enable
        val = 1 if enable else 0
        return self._set_ctrl(self.CID_FOCUS_AUTO, val)

    def get_autofocus(self):
        val = self._get_ctrl(self.CID_FOCUS_AUTO)
        if val is not None:
            self.is_auto_focus = (val == 1)
            return self.is_auto_focus
        return self.is_auto_focus

    def set_focus(self, focus_val: int):
        focus_val = int(max(0, min(250, focus_val)))
        self.current_focus_cache = focus_val
        self.is_auto_focus = False
        # Disable continuous firmware AF when manual focus is commanded
        self._set_ctrl(self.CID_FOCUS_AUTO, 0)
        return self._set_ctrl(self.CID_FOCUS_ABSOLUTE, focus_val)

    def get_focus(self):
        val = self._get_ctrl(self.CID_FOCUS_ABSOLUTE)
        if val is not None:
            self.current_focus_cache = val
            return val
        return self.current_focus_cache

    def set_sharpness(self, sharpness_val: int):
        sharpness_val = int(max(0, min(255, sharpness_val)))
        return self._set_ctrl(self.CID_SHARPNESS, sharpness_val)

    def get_sharpness(self):
        val = self._get_ctrl(self.CID_SHARPNESS)
        return val if val is not None else 140

    def close(self):
        if self.fd is not None:
            try:
                os.close(self.fd)
            except Exception:
                pass
            self.fd = None


# ==========================================
# THREADED CAPTURE WITH FRAME SYNCHRONIZATION
# ==========================================
class ThreadedWebcamCapture:
    """High-speed threaded frame grabber with hardware MJPG and monotonic frame synchronization."""
    def __init__(self, src=2, width=1280, height=720, fps=60):
        self.src = src
        self.width = width
        self.height = height
        self.fps = fps

        self.lock = threading.Lock()
        self.ret = False
        self.frame = None
        self.frame_id = 0
        self.stopped = False
        self.cap = None

        self._start_capture(width, height, fps)

        self.thread = threading.Thread(target=self._reader, daemon=True)
        self.thread.start()

        # Wait for first valid frame
        for _ in range(25):
            if self.frame is not None:
                break
            time.sleep(0.04)

    def _start_capture(self, width, height, fps):
        if self.cap is not None:
            self.cap.release()
        self.width = width
        self.height = height
        self.fps = fps
        self.cap = cv2.VideoCapture(self.src, cv2.CAP_V4L2 if os.name == 'posix' else cv2.CAP_ANY)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, fps)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    def set_resolution(self, width, height, fps):
        """Dynamically switch sensor capture resolution without restarting the application."""
        with self.lock:
            self._start_capture(width, height, fps)
        time.sleep(0.12)
        print(f"[Camera Hardware] Switched capture mode: {width}x{height} @ {fps} FPS.")

    def _reader(self):
        while not self.stopped:
            with self.lock:
                cap_ref = self.cap
            if cap_ref is None or not cap_ref.isOpened():
                time.sleep(0.01)
                continue
            ret, frame = cap_ref.read()
            if ret and frame is not None:
                with self.lock:
                    self.ret = ret
                    self.frame = frame
                    self.frame_id += 1
            else:
                time.sleep(0.002)

    def read(self):
        with self.lock:
            return self.ret, (self.frame.copy() if self.frame is not None else None)

    def get_latest_frame(self):
        with self.lock:
            return self.frame.copy() if self.frame is not None else None

    def wait_for_fresh_frame(self, skip_frames=2, timeout=0.35):
        """Guarantee camera optical synchronization by waiting for newly exposed frames."""
        with self.lock:
            start_id = self.frame_id
        t0 = time.time()
        while time.time() - t0 < timeout:
            with self.lock:
                if self.frame_id >= start_id + skip_frames:
                    return self.frame.copy() if self.frame is not None else None
            time.sleep(0.01)
        with self.lock:
            return self.frame.copy() if self.frame is not None else None

    def isOpened(self):
        return self.cap.isOpened() if self.cap is not None else False

    def release(self):
        self.stopped = True
        if self.cap is not None:
            self.cap.release()


# ==========================================
# ROBUST ROI SHARPNESS & AUTOFOCUS ENGINE
# ==========================================
def calculate_sharpness(roi_bgr):
    """Compute Tenengrad / Laplacian variance sharpness score."""
    if roi_bgr is None or roi_bgr.size == 0:
        return 0.0
    gray = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2GRAY)
    rh, rw = gray.shape[:2]
    if rh > 40 and rw > 40:
        margin_y = int(rh * 0.12)
        margin_x = int(rw * 0.12)
        gray = gray[margin_y:rh - margin_y, margin_x:rw - margin_x]
    return float(cv2.Laplacian(gray, cv2.CV_64F).var())


class ClickToFocusEngine:
    """Asynchronous optical contrast-maximization autofocus worker with frame sync."""
    def __init__(self, v4l2_ctrl, stream):
        self.v4l2 = v4l2_ctrl
        self.stream = stream
        self.lock = threading.Lock()
        self.scanning = False
        self.status = "READY"
        self.target_label = None
        self.best_focus = 15
        self.best_score = 0.0
        self.last_lock_time = 0.0

    def is_scanning(self):
        with self.lock:
            return self.scanning

    def get_status(self):
        with self.lock:
            return self.status, self.target_label, self.best_focus, self.best_score

    def trigger_focus(self, bbox, label="Target"):
        with self.lock:
            if self.scanning:
                return
            self.scanning = True
            self.status = f"AUTOFOCUSING: '{label}'..."
            self.target_label = label

        t = threading.Thread(target=self._focus_worker, args=(bbox, label), daemon=True)
        t.start()

    def _focus_worker(self, bbox, label):
        x1, y1, x2, y2 = bbox
        # Realistic bench candidates with prioritized standoff range
        candidates = [0, 15, 30, 45, 65, 90, 130, 180]
        best_f = 15
        max_score = -1.0

        # Disable firmware AF
        self.v4l2.set_autofocus(False)
        time.sleep(0.03)

        # Coarse Sweep with monotonic frame synchronization
        for f_val in candidates:
            with self.lock:
                if not self.scanning:
                    return

            self.v4l2.set_focus(f_val)
            frame = self.stream.wait_for_fresh_frame(skip_frames=2, timeout=0.20)
            if frame is not None:
                h, w = frame.shape[:2]
                rx1, ry1 = max(0, x1), max(0, y1)
                rx2, ry2 = min(w, x2), min(h, y2)
                if rx2 > rx1 + 10 and ry2 > ry1 + 10:
                    roi = frame[ry1:ry2, rx1:rx2]
                    score = calculate_sharpness(roi)
                    if score > max_score:
                        max_score = score
                        best_f = f_val

        # Fine bracket around peak (+/- 10 in steps of 5)
        fine_candidates = [f for f in range(max(0, best_f - 10), min(250, best_f + 15), 5) if f != best_f]
        for f_val in fine_candidates:
            with self.lock:
                if not self.scanning:
                    return

            self.v4l2.set_focus(f_val)
            frame = self.stream.wait_for_fresh_frame(skip_frames=2, timeout=0.20)
            if frame is not None:
                h, w = frame.shape[:2]
                rx1, ry1 = max(0, x1), max(0, y1)
                rx2, ry2 = min(w, x2), min(h, y2)
                if rx2 > rx1 + 10 and ry2 > ry1 + 10:
                    roi = frame[ry1:ry2, rx1:rx2]
                    score = calculate_sharpness(roi)
                    if score > max_score:
                        max_score = score
                        best_f = f_val

        # Lock voice coil at peak
        self.v4l2.set_focus(best_f)
        self.stream.wait_for_fresh_frame(skip_frames=2, timeout=0.20)

        with self.lock:
            self.best_focus = best_f
            self.best_score = max_score
            self.scanning = False
            self.last_lock_time = time.time()
            self.status = f"LOCKED: Focus={best_f} (S={max_score:.0f})"
            print(f"[Click-to-Focus] Locked onto '{label}' at Focus={best_f} (Sharpness={max_score:.1f}).")

    def cancel(self):
        with self.lock:
            self.scanning = False
            self.status = "MANUAL"


# ==========================================
# INTERACTIVE FOCUS SLIDER UI (MATCHED TO AF)
# ==========================================
def draw_focus_control_card(display, current_focus, is_scanning, is_af, af_target_f, target_sharpness, slider_dragging, current_res_mode="1080p", clean_view=False):
    """Draw interactive on-screen focus slider perfectly matched to autofocus."""
    h, w = display.shape[:2]

    card_w = 120
    card_x = w - card_w - 10
    card_y = 105
    card_h = h - card_y - 20

    slider_x = card_x + 38
    slider_y_top = card_y + 95
    # Allocate room at bottom for presets, resolution toggle, and clean view toggle
    slider_y_bottom = card_y + card_h - 145
    slider_h = slider_y_bottom - slider_y_top

    # 1. Semi-transparent card background
    overlay = display.copy()
    cv2.rectangle(overlay, (card_x, card_y), (card_x + card_w, card_y + card_h), (20, 20, 20), -1)
    cv2.addWeighted(overlay, 0.85, display, 0.15, 0, display)
    cv2.rectangle(display, (card_x, card_y), (card_x + card_w, card_y + card_h), (80, 80, 80), 1)

    # 2. Interactive Mode Toggle Button: [AUTO AF] vs [MANUAL]
    btn_mode_y1 = card_y + 8
    btn_mode_y2 = btn_mode_y1 + 24
    mode_btn_col = (0, 160, 0) if is_af else (45, 45, 45)
    mode_txt_col = (255, 255, 255) if is_af else (0, 255, 255)
    cv2.rectangle(display, (card_x + 8, btn_mode_y1), (card_x + card_w - 8, btn_mode_y2), mode_btn_col, -1)
    cv2.rectangle(display, (card_x + 8, btn_mode_y1), (card_x + card_w - 8, btn_mode_y2), (0, 255, 0) if is_af else (100, 100, 100), 1)
    mode_label = "MODE: AUTO" if is_af else "MODE: MANUAL"
    cv2.putText(display, mode_label, (card_x + 14, btn_mode_y1 + 16), cv2.FONT_HERSHEY_SIMPLEX, 0.36, mode_txt_col, 1)

    # 3. [FOCUS NOW] Scan Button
    btn_scan_y1 = btn_mode_y2 + 6
    btn_scan_y2 = btn_scan_y1 + 22
    scan_btn_col = (0, 200, 255) if is_scanning else (35, 35, 35)
    cv2.rectangle(display, (card_x + 8, btn_scan_y1), (card_x + card_w - 8, btn_scan_y2), scan_btn_col if is_scanning else (35, 35, 35), -1)
    cv2.rectangle(display, (card_x + 8, btn_scan_y1), (card_x + card_w - 8, btn_scan_y2), (0, 255, 255), 1)
    scan_lbl = "SCANNING..." if is_scanning else "[AUTO FOCUS]"
    cv2.putText(display, scan_lbl, (card_x + 13, btn_scan_y1 + 15), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 0, 0) if is_scanning else (0, 255, 255), 1)

    # Header Value
    cv2.putText(display, f"FOCUS: {current_focus}", (card_x + 20, btn_scan_y2 + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.40, (255, 255, 255), 1)

    # 4. Slider Track
    cv2.rectangle(display, (slider_x - 3, slider_y_top), (slider_x + 3, slider_y_bottom), (50, 50, 50), -1)
    cv2.rectangle(display, (slider_x - 3, slider_y_top), (slider_x + 3, slider_y_bottom), (90, 90, 90), 1)

    # 5. Active Level Fill
    clamped_focus = max(0, min(250, current_focus))
    fill_y = slider_y_bottom - int((clamped_focus / 250.0) * slider_h)
    cv2.rectangle(display, (slider_x - 2, fill_y), (slider_x + 2, slider_y_bottom), (0, 255, 255), -1)

    # 6. Autofocus Target Marker (AF* Notch on Track)
    if af_target_f is not None and 0 <= af_target_f <= 250:
        af_y = slider_y_bottom - int((af_target_f / 250.0) * slider_h)
        cv2.line(display, (slider_x - 12, af_y), (slider_x + 12, af_y), (0, 255, 0), 2)
        cv2.putText(display, "AF*", (card_x + 6, af_y + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.32, (0, 255, 0), 1)

    # 7. Scale Ticks & Distance Labels
    ticks = [
        (250, "MACRO"),
        (150, "15cm"),
        (75, "40cm"),
        (30, "70cm"),
        (15, "ROOM"),
        (0, "INF")
    ]
    for val, name in ticks:
        ty = slider_y_bottom - int((val / 250.0) * slider_h)
        cv2.line(display, (slider_x - 6, ty), (slider_x - 4, ty), (150, 150, 150), 1)
        cv2.putText(display, name, (card_x + 50, ty + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.30, (160, 160, 160), 1)

    # 8. Slider Knob Handle (Synchronized with current focus!)
    knob_col = (0, 255, 255) if slider_dragging else ((0, 255, 0) if is_af else (255, 190, 0))
    cv2.rectangle(display, (slider_x - 20, fill_y - 12), (slider_x + 20, fill_y + 12), knob_col, -1)
    cv2.rectangle(display, (slider_x - 20, fill_y - 12), (slider_x + 20, fill_y + 12), (255, 255, 255), 2)
    cv2.putText(display, f"{clamped_focus}", (slider_x - 12, fill_y + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.40, (0, 0, 0), 2)

    # 9. Preset Buttons at Bottom
    btn_room_y1 = slider_y_bottom + 10
    btn_room_y2 = btn_room_y1 + 22
    cv2.rectangle(display, (card_x + 8, btn_room_y1), (card_x + card_w - 8, btn_room_y2), (40, 40, 40), -1)
    cv2.rectangle(display, (card_x + 8, btn_room_y1), (card_x + card_w - 8, btn_room_y2), (100, 100, 100), 1)
    cv2.putText(display, "[ROOM 15]", (card_x + 20, btn_room_y1 + 15), cv2.FONT_HERSHEY_SIMPLEX, 0.36, (0, 255, 255), 1)

    btn_desk_y1 = btn_room_y2 + 5
    btn_desk_y2 = btn_desk_y1 + 22
    cv2.rectangle(display, (card_x + 8, btn_desk_y1), (card_x + card_w - 8, btn_desk_y2), (40, 40, 40), -1)
    cv2.rectangle(display, (card_x + 8, btn_desk_y1), (card_x + card_w - 8, btn_desk_y2), (100, 100, 100), 1)
    cv2.putText(display, "[DESK 40]", (card_x + 22, btn_desk_y1 + 15), cv2.FONT_HERSHEY_SIMPLEX, 0.36, (0, 255, 255), 1)

    # 10. Resolution Switch Button [1080p HD] vs [720p 60F]
    btn_res_y1 = btn_desk_y2 + 5
    btn_res_y2 = btn_res_y1 + 22
    res_btn_col = (60, 40, 0) if current_res_mode == "1080p" else (30, 60, 30)
    cv2.rectangle(display, (card_x + 8, btn_res_y1), (card_x + card_w - 8, btn_res_y2), res_btn_col, -1)
    cv2.rectangle(display, (card_x + 8, btn_res_y1), (card_x + card_w - 8, btn_res_y2), (0, 200, 255), 1)
    res_label = "[RES: 1080p]" if current_res_mode == "1080p" else "[RES: 720p]"
    cv2.putText(display, res_label, (card_x + 14, btn_res_y1 + 15), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 255, 255), 1)

    # 11. Clean View Button [HUD: ON] / [HUD: OFF]
    btn_clean_y1 = btn_res_y2 + 5
    btn_clean_y2 = btn_clean_y1 + 22
    clean_col = (0, 100, 0) if clean_view else (40, 40, 40)
    cv2.rectangle(display, (card_x + 8, btn_clean_y1), (card_x + card_w - 8, btn_clean_y2), clean_col, -1)
    cv2.rectangle(display, (card_x + 8, btn_clean_y1), (card_x + card_w - 8, btn_clean_y2), (0, 255, 0) if clean_view else (100, 100, 100), 1)
    clean_label = "[HUD: ON]" if not clean_view else "[HUD: OFF]"
    cv2.putText(display, clean_label, (card_x + 22, btn_clean_y1 + 15), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 255, 255), 1)

    return {
        "card_x1": card_x,
        "card_x2": card_x + card_w,
        "card_y1": card_y,
        "card_y2": card_y + card_h,
        "slider_x": slider_x,
        "slider_y_top": slider_y_top,
        "slider_y_bottom": slider_y_bottom,
        "slider_h": slider_h,
        "btn_mode": (card_x + 8, btn_mode_y1, card_x + card_w - 8, btn_mode_y2),
        "btn_scan": (card_x + 8, btn_scan_y1, card_x + card_w - 8, btn_scan_y2),
        "btn_room": (card_x + 8, btn_room_y1, card_x + card_w - 8, btn_room_y2),
        "btn_desk": (card_x + 8, btn_desk_y1, card_x + card_w - 8, btn_desk_y2),
        "btn_res": (card_x + 8, btn_res_y1, card_x + card_w - 8, btn_res_y2),
        "btn_clean": (card_x + 8, btn_clean_y1, card_x + card_w - 8, btn_clean_y2),
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
    elif "chair" in n:
        return "Chair"
    elif "table" in n or "desk" in n:
        return "Table"
    elif "backpack" in n:
        return "Backpack"
    elif "camera" in n or "tripod" in n:
        return "Camera/Tripod"
    return raw_name.title()


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


# ==========================================
# GLOBAL MOUSE STATE
# ==========================================
mouse_state = {
    "dragging_slider": False,
    "pending_click_target": None,
    "last_ui_boxes": None,
    "wheel_delta": 0,
    "manual_lock": False,
    "last_click_visual": None
}

def on_mouse_event(event, x, y, flags, param):
    global mouse_state
    ui = mouse_state.get("last_ui_boxes")
    v4l2 = param["v4l2"]
    focus_engine = param["focus_engine"]

    # 1. Mouse Wheel Focus Adjustment
    if event == cv2.EVENT_MOUSEWHEEL:
        if flags > 0:
            mouse_state["wheel_delta"] += 2
        else:
            mouse_state["wheel_delta"] -= 2
        return

    # 2. Left Button Down
    if event == cv2.EVENT_LBUTTONDOWN:
        if ui is not None and ui["card_x1"] <= x <= ui["card_x2"] and ui["card_y1"] <= y <= ui["card_y2"]:
            # Check Mode Button: [AUTO AF] vs [MANUAL]
            mx1, my1, mx2, my2 = ui["btn_mode"]
            if mx1 <= x <= mx2 and my1 <= y <= my2:
                is_af = v4l2.get_autofocus()
                new_af = not is_af
                v4l2.set_autofocus(new_af)
                print(f"[Focus Mode] Toggled: {'AUTO AF' if new_af else 'MANUAL'}")
                return

            # Check Scan Button: [AUTO FOCUS NOW]
            sx1, sy1, sx2, sy2 = ui["btn_scan"]
            if sx1 <= x <= sx2 and sy1 <= y <= sy2:
                print("[Focus] Triggering Auto-Focus scan on target...")
                focus_engine.trigger_focus(param.get("active_target_bbox", (640-100, 360-100, 640+100, 360+100)), "Target")
                return

            # Check Preset 1: ROOM (15)
            bx1, by1, bx2, by2 = ui["btn_room"]
            if bx1 <= x <= bx2 and by1 <= y <= by2:
                v4l2.set_focus(15)
                focus_engine.cancel()
                print("[Manual Focus] Preset selected: ROOM (Focus=15)")
                return

            # Check Preset 2: DESK (40)
            dx1, dy1, dx2, dy2 = ui["btn_desk"]
            if dx1 <= x <= dx2 and dy1 <= y <= dy2:
                v4l2.set_focus(40)
                focus_engine.cancel()
                print("[Manual Focus] Preset selected: DESK (Focus=40)")
                return

            # Check Resolution Toggle Button
            if "btn_res" in ui:
                rx1, ry1, rx2, ry2 = ui["btn_res"]
                if rx1 <= x <= rx2 and ry1 <= y <= ry2:
                    param["request_res_toggle"] = True
                    return

            # Check Clean View Button
            if "btn_clean" in ui:
                cx1, cy1, cx2, cy2 = ui["btn_clean"]
                if cx1 <= x <= cx2 and cy1 <= y <= cy2:
                    param["request_clean_toggle"] = True
                    return

            # Clicked on Slider Track
            if ui["slider_y_top"] - 15 <= y <= ui["slider_y_bottom"] + 15:
                mouse_state["dragging_slider"] = True
                focus_engine.cancel()
                norm_pos = float(ui["slider_y_bottom"] - y) / float(ui["slider_h"])
                new_f = int(np.clip(norm_pos * 250.0, 0, 250))
                v4l2.set_focus(new_f)
                return
        else:
            # Clicked on Video Canvas -> Register Target Lock & Click-to-Focus
            mouse_state["pending_click_target"] = (x, y)
            mouse_state["last_click_visual"] = (x, y, time.time())

    # 3. Mouse Move while Dragging Slider
    elif event == cv2.EVENT_MOUSEMOVE:
        if mouse_state["dragging_slider"] and ui is not None:
            norm_pos = float(ui["slider_y_bottom"] - y) / float(ui["slider_h"])
            new_f = int(np.clip(norm_pos * 250.0, 0, 250))
            v4l2.set_focus(new_f)

    # 4. Left Button Up
    elif event == cv2.EVENT_LBUTTONUP:
        mouse_state["dragging_slider"] = False


# ==========================================
# MAIN APPLICATION LOOP
# ==========================================
def main():
    global mouse_state
    print("=" * 80)
    print("   AUV BENCH TEST v2.5: ULTRA-FIDELITY 1080p LOGITECH C922 EVALUATION   ")
    print("=" * 80)

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
    print(f"[Camera] Connecting to Logitech C922 on /dev/video{cam_index}...")

    # Hardware V4L2 Controller (Initializes at Focus=15, Sharpness=140)
    v4l2_ctrl = V4L2HardwareController(dev_index=cam_index)

    # Default to Native Full HD 1080p @ 30 FPS for razor-sharp optical clarity on 2.5K/4K displays
    current_res_mode = "1080p"
    current_cap_w, current_cap_h, current_cap_fps = 1920, 1080, 30
    stream = ThreadedWebcamCapture(src=cam_index, width=current_cap_w, height=current_cap_h, fps=current_cap_fps)
    if not stream.isOpened():
        print("[Error] Failed to open Logitech C922! Trying index 0...")
        cam_index = 0
        stream = ThreadedWebcamCapture(src=0, width=current_cap_w, height=current_cap_h, fps=current_cap_fps)

    # Enhanced Click-to-Focus Engine with monotonic frame synchronization
    focus_engine = ClickToFocusEngine(v4l2_ctrl, stream)

    # Kalman Filter initialization with AGILE tuning (qs=1.0 for instant zero-lag response)
    current_qs = 1.0
    kf = AUVVisualKalmanFilter(dt=1.0 / 30.0, mode="8D", qs=current_qs, r_var=0.15, gate_px=450.0)

    # Runtime toggles
    kalman_enabled = True
    split_screen_mode = False
    clean_view_mode = False
    clahe_enabled = False
    current_imgsz = 640
    conf_thresh = 0.12  # Sensitive threshold so all bench objects appear reliably

    # Target persistence state
    locked_label = None
    locked_bbox = None
    locked_center = None
    mouse_state["manual_lock"] = False

    # Jitter evaluation deques
    raw_history = deque(maxlen=20)
    kf_history = deque(maxlen=20)

    # Clean Qt Window without toolbar padding to ensure 1:1 pixel coordinate alignment
    WINDOW_NAME = "AUV Bench Test v2.5 (Logitech C922: Full HD 1080p & Focus Evaluation)"
    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL | cv2.WINDOW_GUI_NORMAL)
    cv2.resizeWindow(WINDOW_NAME, 1920, 1080)

    param_dict = {
        "v4l2": v4l2_ctrl,
        "focus_engine": focus_engine,
        "active_target_bbox": (current_cap_w // 2 - 120, current_cap_h // 2 - 120, current_cap_w // 2 + 120, current_cap_h // 2 + 120),
        "request_res_toggle": False,
        "request_clean_toggle": False
    }
    cv2.setMouseCallback(WINDOW_NAME, on_mouse_event, param_dict)

    print("\n" + "=" * 80)
    print("   INTERACTIVE CONTROLS:")
    print("   [Left-Click]    - CLICK ANY OBJECT (Laptop, Bottle, Tools) TO LOCK & AUTOFOCUS!")
    print("   [FOCUS SLIDER]  - Drag the vertical slider on the RIGHT (matches AF in real-time)")
    print("   [MODE: AUTO]    - Click button above slider to toggle Auto-Focus / Manual")
    print("   [AUTO FOCUS]    - Click to trigger synchronized autofocus sweep on current target")
    print("   [PRESETS]       - Click [ROOM 15] or [DESK 40] on screen for instant focus")
    print("   [RES BUTTON]    - Click [RES: 1080p] / [RES: 720p] to toggle resolution instantly")
    print("   [HUD BUTTON]    - Click [HUD: ON] / [HUD: OFF] or press [h] for Clean Optical View")
    print("   [MOUSE WHEEL]   - Scroll wheel anywhere to micro-adjust focus (+/- 2)")
    print("   [t]             - Reset target lock (Reverts to auto-tracking)")
    print("   [h]             - Toggle Clean View (Hide/Show all bounding boxes & HUD)")
    print("   [1] / [2]       - Toggle 1080p Full HD vs 720p 60 FPS")
    print("   [k]             - Toggle Kalman Filter ON / OFF")
    print("   [s]             - Toggle Split-Screen (Side-by-Side vs. Overlay)")
    print("   [a]             - Toggle Agility (Zero-Lag qs=1.0 vs Heavy Smooth qs=0.08)")
    print("   [+] / [-]       - Adjust Detection Confidence Threshold (+/- 0.02)")
    print("   [q]             - Exit cleanly")
    print("=" * 80 + "\n")

    t_prev = time.perf_counter()
    fps_smooth = 60.0

    while True:
        # Handle Mouse Wheel Focus Adjustments
        if mouse_state["wheel_delta"] != 0:
            delta = mouse_state["wheel_delta"]
            mouse_state["wheel_delta"] = 0
            curr_f = v4l2_ctrl.get_focus()
            new_f = max(0, min(250, curr_f + delta))
            v4l2_ctrl.set_focus(new_f)
            focus_engine.cancel()
            print(f"[Manual Focus] Wheel Nudged Focus: {new_f}/250")

        # Handle Mouse Button Toggle Events
        if param_dict.get("request_res_toggle", False):
            param_dict["request_res_toggle"] = False
            if current_res_mode == "1080p":
                print("[Resolution] Switching to High-Speed 720p Mode (1280x720 @ 60 FPS)...")
                current_res_mode = "720p"
                current_cap_w, current_cap_h, current_cap_fps = 1280, 720, 60
            else:
                print("[Resolution] Switching to Full HD 1080p Mode (1920x1080 @ 30 FPS)...")
                current_res_mode = "1080p"
                current_cap_w, current_cap_h, current_cap_fps = 1920, 1080, 30
            stream.set_resolution(current_cap_w, current_cap_h, current_cap_fps)
            cv2.resizeWindow(WINDOW_NAME, current_cap_w, current_cap_h)

        if param_dict.get("request_clean_toggle", False):
            param_dict["request_clean_toggle"] = False
            clean_view_mode = not clean_view_mode
            print(f"[HUD] Clean View Mode: {'ON (HUD Hidden)' if clean_view_mode else 'OFF (HUD Visible)'}")

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
                "area": max(1, (x2 - x1) * (y2 - y1))
            })

        # -------------------------------------------------------------
        # PRECISION CLICK SELECTION: SMALLEST-AREA-FIRST MATCHING
        # -------------------------------------------------------------
        if mouse_state["pending_click_target"] is not None:
            cx_click, cy_click = mouse_state["pending_click_target"]
            mouse_state["pending_click_target"] = None

            # 1. Find all candidates that geometrically enclose the click
            enclosing_cands = [c for c in candidate_boxes if c["bbox"][0] <= cx_click <= c["bbox"][2]
                               and c["bbox"][1] <= cy_click <= c["bbox"][3] and c["is_valid"]]

            selected_cand = None
            if len(enclosing_cands) > 0:
                # CRITICAL FIX: Sort by SMALLEST bounding box area first!
                # If Laptop sits on Table, Laptop is smaller than Table -> selects Laptop!
                # If Smartphone is held by Person, Smartphone is smaller than Person -> selects Smartphone!
                enclosing_cands.sort(key=lambda c: c["area"])
                selected_cand = enclosing_cands[0]
            else:
                # 2. Distance fallback: if clicked within 90px of an object center
                close_cands = []
                for c in candidate_boxes:
                    if c["is_valid"]:
                        dist = math.hypot(c["center"][0] - cx_click, c["center"][1] - cy_click)
                        if dist < 90:
                            close_cands.append((dist, c))
                if len(close_cands) > 0:
                    close_cands.sort(key=lambda item: item[0])
                    selected_cand = close_cands[0][1]

            if selected_cand is not None:
                locked_label = selected_cand["raw_name"]
                locked_bbox = selected_cand["bbox"]
                locked_center = selected_cand["center"]
                mouse_state["manual_lock"] = True

                # Snap Kalman Filter directly to the clicked object to eliminate lag
                bx, by = locked_center
                bw = float(locked_bbox[2] - locked_bbox[0])
                bh = float(locked_bbox[3] - locked_bbox[1])
                kf.init(float(bx), float(by), bw, bh)

                print(f"[Target Lock] MANUALLY LOCKED ONTO: {selected_cand['label']} (Area={selected_cand['area']}px)!")
                focus_engine.trigger_focus(selected_cand["bbox"], selected_cand["label"])
            else:
                # Clicked on empty space -> Trigger focus on clicked spot and return to auto-tracking
                print(f"[Target Lock] Clicked on background ({cx_click}, {cy_click}). Returning to auto-selection.")
                mouse_state["manual_lock"] = False
                locked_label = None
                locked_bbox = None
                locked_center = None
                roi_r = 90
                rx1, ry1 = max(0, cx_click - roi_r), max(0, cy_click - roi_r)
                rx2, ry2 = min(w, cx_click + roi_r), min(h, cy_click + roi_r)
                focus_engine.trigger_focus((rx1, ry1, rx2, ry2), "Clicked Spot")

        # -------------------------------------------------------------
        # IRONCLAD TARGET TRACKING (NEVER SWITCHES TO PERSON)
        # -------------------------------------------------------------
        best_cand = None

        if mouse_state["manual_lock"] and locked_label is not None:
            # MANUALLY LOCKED MODE: STRICTLY TRACK ONLY THE LOCKED OBJECT CLASS
            best_dist = 999999
            lcx, lcy = locked_center if locked_center is not None else (center_x, center_y)

            for cand in candidate_boxes:
                # Strict class match (e.g. only Laptop matches Laptop)
                if cand["raw_name"] == locked_label:
                    cx, cy = cand["center"]
                    dist = math.hypot(cx - lcx, cy - lcy)
                    if dist < best_dist and dist < 350:
                        best_dist = dist
                        best_cand = cand

            if best_cand is not None:
                locked_bbox = best_cand["bbox"]
                locked_center = best_cand["center"]
            else:
                # If the target is temporarily dropped (e.g. during focus sweep or occlusion),
                # DO NOT switch to Person or any other object! Hold lock and dead-reckon!
                pass

        else:
            # AUTO-SELECTION MODE: Uses OBJECT_PRIORITY (Bench electronics heavily favored over Person)
            max_score = 0.0
            for cand in candidate_boxes:
                if cand["is_valid"]:
                    weight = OBJECT_PRIORITY.get(cand["raw_name"], 1.5)
                    # Priority score: Area factor scaled by class priority weight
                    score = math.sqrt(cand["area"]) * cand["conf"] * weight
                    if score > max_score:
                        max_score = score
                        best_cand = cand

            if best_cand is not None:
                locked_label = best_cand["raw_name"]
                locked_bbox = best_cand["bbox"]
                locked_center = best_cand["center"]

        # Pass active target bbox to mouse param callback
        if best_cand is not None:
            param_dict["active_target_bbox"] = best_cand["bbox"]
        elif locked_bbox is not None:
            param_dict["active_target_bbox"] = locked_bbox
        else:
            param_dict["active_target_bbox"] = (center_x - 100, center_y - 100, center_x + 100, center_y + 100)

        # Process Target Coordinates & Kalman Update
        raw_x, raw_y = None, None
        kf_x, kf_y = None, None
        target_display_name = "None"
        is_occluded = False
        target_sharpness = 0.0

        if best_cand is not None:
            cx, cy = best_cand["center"]
            x1, y1, x2, y2 = best_cand["bbox"]
            conf = best_cand["conf"]
            raw_x, raw_y = cx, cy
            target_display_name = best_cand["label"]
            raw_history.append((raw_x, raw_y))

            # Compute optical sharpness of primary target
            target_roi = frame[max(0, y1):min(h, y2), max(0, x1):min(w, x2)]
            target_sharpness = calculate_sharpness(target_roi)

            if kalman_enabled:
                fx, fy = kf.update_bbox(x1, y1, x2, y2, conf=conf)
                kf_x, kf_y = int(fx), int(fy)
                kf_history.append((kf_x, kf_y))
        else:
            # Target is temporarily occluded or dropped during focus scan -> DEAD-RECKON ON LOCKED TARGET!
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

        # Physical Metric Velocity Computation
        vx_px, vy_px = kf.get_velocity() if (kalman_enabled and kf.initialized) else (0.0, 0.0)
        est_distance_cm = 80.0
        cm_per_px = (2.0 * est_distance_cm * math.tan(math.radians(35.2))) / float(w)
        vx_metric_cms = vx_px * cm_per_px
        vy_metric_cms = -vy_px * cm_per_px
        speed_total_cms = math.hypot(vx_metric_cms, vy_metric_cms)
        speed_total_ms = speed_total_cms / 100.0

        horiz_dir = "Right ->" if vx_metric_cms > 3.0 else ("Left <-" if vx_metric_cms < -3.0 else "Still")
        vert_dir = "Up ^" if vy_metric_cms > 3.0 else ("Down v" if vy_metric_cms < -3.0 else "Still")

        # Real-Time Focus & Autofocus Matching
        is_focus_scanning = focus_engine.is_scanning()
        focus_status_str, focus_target, af_peak_focus, focus_score = focus_engine.get_status()
        current_focus = v4l2_ctrl.get_focus()
        current_af = v4l2_ctrl.get_autofocus()

        # -------------------------------------------------------------
        # VISUALIZATION RENDERING
        # -------------------------------------------------------------
        if split_screen_mode:
            frame_raw = frame.copy()
            frame_kf = frame.copy()

            # Left: RAW YOLO (All detections)
            cv2.putText(frame_raw, f"[WITHOUT KALMAN: RAW YOLO ({len(candidate_boxes)} Objects)]", (20, 35),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2)
            for cand in candidate_boxes:
                x1, y1, x2, y2 = cand["bbox"]
                cv2.rectangle(frame_raw, (x1, y1), (x2, y2), (0, 0, 255), 1)
                cv2.putText(frame_raw, f"{cand['label']} {cand['conf']:.2f}", (x1, max(15, y1 - 4)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 0, 255), 1)

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
                tag = f"DEAD-RECKONING ({kf.missed_frames}f)" if is_occluded else f"{target_display_name} [SMOOTH]"
                cv2.putText(frame_kf, tag, (max(10, kf_x - 70), max(25, kf_y - 12)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, col, 2)

            half_w = w // 2
            display = np.hstack((cv2.resize(frame_raw, (half_w, h)), cv2.resize(frame_kf, (half_w, h))))
            cv2.line(display, (half_w, 0), (half_w, h), (255, 255, 255), 2)

        else:
            # === OVERLAY MODE ===
            display = frame

            if clean_view_mode:
                # Clean View Mode: Pure uncompressed optics with a minimal status badge
                cv2.rectangle(display, (10, 10), (490, 44), (15, 15, 15), -1)
                cv2.rectangle(display, (10, 10), (490, 44), (0, 255, 255), 1)
                cv2.putText(display, "[CLEAN VIEW: RAW OPTICS] Press 'h' to show overlays",
                            (20, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.44, (0, 255, 255), 1)
            else:
                # Center Crosshair
                cv2.line(display, (center_x - 12, center_y), (center_x + 12, center_y), (120, 120, 120), 1)
                cv2.line(display, (center_x, center_y - 12), (center_x, center_y + 12), (120, 120, 120), 1)

                # Draw visual click ripple if recently clicked
                if mouse_state["last_click_visual"] is not None:
                    lcx, lcy, t_click = mouse_state["last_click_visual"]
                    elapsed = time.time() - t_click
                    if elapsed < 0.8:
                        radius = int(12 + elapsed * 30)
                        alpha = max(0.0, 1.0 - elapsed / 0.8)
                        cv2.circle(display, (lcx, lcy), radius, (0, 255, 255), 2)
                        cv2.drawMarker(display, (lcx, lcy), (0, 255, 255), cv2.MARKER_CROSS, 16, 1)

                # 1. DRAW ALL RAW YOLO DETECTIONS IN CRISP RED
                for cand in candidate_boxes:
                    x1, y1, x2, y2 = cand["bbox"]
                    if cand["raw_name"] == "hand":
                        cv2.rectangle(display, (x1, y1), (x2, y2), (0, 140, 255), 1)
                        cv2.putText(display, f"Hand (Occluder) {cand['conf']:.2f}", (x1, max(15, y1 - 4)),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.40, (0, 140, 255), 1)
                    else:
                        is_main_target = (best_cand is not None and cand is best_cand)
                        cv2.rectangle(display, (x1, y1), (x2, y2), (0, 0, 255), 2 if is_main_target else 1)
                        cv2.circle(display, cand["center"], 3, (0, 0, 255), -1)
                        cv2.putText(display, f"{cand['label']} {cand['conf']:.2f}", (x1, max(15, y1 - 6)),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 0, 255), 1)

                # 2. DRAW 8D KALMAN FILTERED BOX ON ACTIVE TARGET (BRIGHT GREEN / CYAN)
                if kalman_enabled and kf_x is not None and kf_y is not None:
                    bbox_smooth = kf.get_bbox()
                    col = (255, 255, 0) if is_occluded else (0, 255, 0)
                    if bbox_smooth is not None:
                        sx1, sy1, sx2, sy2, _, _ = bbox_smooth
                        cv2.rectangle(display, (sx1, sy1), (sx2, sy2), col, 2)

                    # Pulsing Reticle if active autofocus scanning
                    if is_focus_scanning and best_cand is not None:
                        bx1, by1, bx2, by2 = best_cand["bbox"]
                        cv2.rectangle(display, (bx1 - 4, by1 - 4), (bx2 + 4, by2 + 4), (0, 255, 255), 2)
                        cv2.putText(display, "AUTOFOCUSING...", (bx1, min(h - 10, by2 + 20)),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)

                    cv2.circle(display, (kf_x, kf_y), 6, col, -1)

                    # Heading line from optical center
                    cv2.line(display, (center_x, center_y), (kf_x, kf_y), (0, 255, 255), 2)

                    # Velocity vector arrow
                    cv2.arrowedLine(display, (kf_x, kf_y), (int(kf_x + vx_px * 0.25), int(kf_y + vy_px * 0.25)), (0, 255, 255), 2)

                    # Target tag with live optical sharpness indicator
                    sharp_tag = f" | S: {target_sharpness:.0f}" if target_sharpness > 0 else ""
                    lock_indicator = "[LOCKED]" if mouse_state["manual_lock"] else "[AUTO]"
                    lbl = f"DEAD-RECKONING ({kf.missed_frames}f)" if is_occluded else f"8D KF: {target_display_name} {lock_indicator}{sharp_tag}"
                    cv2.putText(display, lbl, (max(10, kf_x - 70), max(25, kf_y - 10)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.55, col, 2)

                # Top Dashboard Banner
                kf_badge = "[KALMAN: ON]" if kalman_enabled else "[KALMAN: OFF]"
                sensor_badge = f"[{current_res_mode.upper()} {w}x{h} @ {fps_smooth:.0f}FPS]"
                det_count_badge = f"[{len(candidate_boxes)} OBJECTS DETECTED]"

                banner_w = max(1100, w - 145)
                cv2.rectangle(display, (10, 10), (banner_w, 95), (15, 15, 15), -1)

                # Header Line 1
                cv2.putText(display, f"LOGITECH C922 BENCH TEST v2.5 | {sensor_badge} {det_count_badge} {kf_badge}",
                            (20, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)

                # Header Line 2: Velocity & Jitter
                speed_txt = f"VELOCITY: {speed_total_cms:.1f} cm/s ({speed_total_ms:.2f} m/s) [{horiz_dir}, {vert_dir}]"
                jitter_txt = f"JITTER: RAW +/-{raw_jitter:.1f}px -> KF +/-{kf_jitter:.1f}px ({stabilization_pct:.0f}% STABLE)"
                cv2.putText(display, f"{speed_txt}  |  {jitter_txt}",
                            (20, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 255, 0) if stabilization_pct > 40 else (0, 200, 255), 2)

                # Header Line 3: Focus Status Readout
                f_col = (0, 255, 255) if is_focus_scanning else ((0, 255, 0) if "LOCKED" in focus_status_str else (180, 180, 180))
                focus_hud = f"FOCUS: {focus_status_str} | Click any Red Box to lock & autofocus | Drag Slider on right"
                cv2.putText(display, focus_hud, (20, 84), cv2.FONT_HERSHEY_SIMPLEX, 0.44, f_col, 1)

        # -------------------------------------------------------------
        # DRAW RIGHT-SIDE MANUAL FOCUS SLIDER (MATCHED TO AUTOFOCUS)
        # -------------------------------------------------------------
        ui_boxes = draw_focus_control_card(
            display=display,
            current_focus=current_focus,
            is_scanning=is_focus_scanning,
            is_af=current_af,
            af_target_f=af_peak_focus,
            target_sharpness=target_sharpness,
            slider_dragging=mouse_state["dragging_slider"],
            current_res_mode=current_res_mode,
            clean_view=clean_view_mode
        )
        mouse_state["last_ui_boxes"] = ui_boxes

        cv2.imshow(WINDOW_NAME, display)

        # -------------------------------------------------------------
        # KEYBOARD EVENT LOOP
        # -------------------------------------------------------------
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break

        # [1] Full HD 1080p Mode (1920x1080 @ 30 FPS)
        elif key == ord('1'):
            if current_res_mode != "1080p":
                print("[Resolution] Switching to Full HD 1080p Mode (1920x1080 @ 30 FPS)...")
                current_res_mode = "1080p"
                current_cap_w, current_cap_h, current_cap_fps = 1920, 1080, 30
                stream.set_resolution(1920, 1080, 30)
                cv2.resizeWindow(WINDOW_NAME, 1920, 1080)

        # [2] High-Speed 720p Mode (1280x720 @ 60 FPS)
        elif key == ord('2'):
            if current_res_mode != "720p":
                print("[Resolution] Switching to High-Speed 720p Mode (1280x720 @ 60 FPS)...")
                current_res_mode = "720p"
                current_cap_w, current_cap_h, current_cap_fps = 1280, 720, 60
                stream.set_resolution(1280, 720, 60)
                cv2.resizeWindow(WINDOW_NAME, 1280, 720)

        # [h] Toggle Clean View Mode (Hide / Show Overlays & HUD)
        elif key == ord('h'):
            clean_view_mode = not clean_view_mode
            print(f"[HUD] Clean View Mode: {'ON (HUD Hidden)' if clean_view_mode else 'OFF (HUD Visible)'}")

        # [f] Toggle Continuous Autofocus
        elif key == ord('f'):
            is_af = v4l2_ctrl.get_autofocus()
            new_af = not is_af
            v4l2_ctrl.set_autofocus(new_af)
            print(f"[Focus Mode] Continuous Auto-Focus: {'ON' if new_af else 'OFF (Manual/Locked)'}")

        # [[] Step Focus Closer (-5)
        elif key == ord('['):
            curr_f = v4l2_ctrl.get_focus()
            new_f = max(0, curr_f - 5)
            v4l2_ctrl.set_focus(new_f)
            focus_engine.cancel()
            print(f"[Manual Focus] Nudged Focus Closer: {new_f}/250")

        # []] Step Focus Farther (+5)
        elif key == ord(']'):
            curr_f = v4l2_ctrl.get_focus()
            new_f = min(250, curr_f + 5)
            v4l2_ctrl.set_focus(new_f)
            focus_engine.cancel()
            print(f"[Manual Focus] Nudged Focus Farther: {new_f}/250")

        # [<] or [,] Decrease Hardware Sharpness (-15)
        elif key in [ord('<'), ord(',')]:
            curr_s = v4l2_ctrl.get_sharpness()
            new_s = max(0, curr_s - 15)
            v4l2_ctrl.set_sharpness(new_s)
            print(f"[Hardware Sharpness] Decreased to {new_s}/255")

        # [>] or [.] Increase Hardware Sharpness (+15)
        elif key in [ord('>'), ord('.')]:
            curr_s = v4l2_ctrl.get_sharpness()
            new_s = min(255, curr_s + 15)
            v4l2_ctrl.set_sharpness(new_s)
            print(f"[Hardware Sharpness] Increased to {new_s}/255")

        # [r] Re-trigger autofocus optimization
        elif key == ord('r'):
            if best_cand is not None:
                print(f"[Auto-Focus] Re-optimizing optical focus for '{target_display_name}'...")
                focus_engine.trigger_focus(best_cand["bbox"], target_display_name)
            else:
                print("[Auto-Focus] Re-optimizing center region...")
                focus_engine.trigger_focus((center_x - 100, center_y - 100, center_x + 100, center_y + 100), "Center")

        # [t] Reset target lock (Revert to auto-tracking)
        elif key == ord('t'):
            locked_label = None
            locked_bbox = None
            locked_center = None
            mouse_state["manual_lock"] = False
            print("[Target Lock] Reset. Reverting to auto-tracking.")

        # [k] Toggle Kalman Filter
        elif key == ord('k'):
            kalman_enabled = not kalman_enabled
            print(f"[Toggle] Kalman Filter: {'ON' if kalman_enabled else 'OFF'}")

        # [s] Toggle Split-Screen
        elif key == ord('s'):
            split_screen_mode = not split_screen_mode
            print(f"[Toggle] View Mode: {'SPLIT SCREEN' if split_screen_mode else 'OVERLAY'}")

        # [a] Toggle Agility Mode
        elif key == ord('a'):
            if current_qs > 0.5:
                current_qs = 0.08
                print("[Agility] Switched to HEAVY SMOOTH Mode (qs = 0.08).")
            else:
                current_qs = 1.0
                print("[Agility] Switched to AGILE ZERO-LAG Mode (qs = 1.0).")
            kf = AUVVisualKalmanFilter(dt=1.0 / 60.0, mode="8D", qs=current_qs, r_var=0.15, gate_px=450.0)

        # [e] Toggle CLAHE
        elif key == ord('e'):
            clahe_enabled = not clahe_enabled
            print(f"[Toggle] CLAHE: {'ON' if clahe_enabled else 'OFF'}")

        # [+] / [-] Confidence threshold
        elif key in [ord('+'), ord('=')]:
            conf_thresh = min(0.90, conf_thresh + 0.02)
            print(f"[Confidence] Threshold: {conf_thresh:.2f}")
        elif key in [ord('-'), ord('_')]:
            conf_thresh = max(0.05, conf_thresh - 0.02)
            print(f"[Confidence] Threshold: {conf_thresh:.2f}")

    stream.release()
    v4l2_ctrl.close()
    cv2.destroyAllWindows()
    print("[Bench Test] Finished.")


if __name__ == "__main__":
    main()
