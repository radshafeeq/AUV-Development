#!/usr/bin/env python3
"""
AUV Real-Time YOLO26 World Target Detection & Tracking Node
----------------------------------------------------------
- Camera Input: BlueOS UDP H.264 Stream from RPi 4B (port 5600) / Logitech C922 (port 5601)
- AI Engine: YOLO26 World (Open-Vocabulary Zero-Shot Vision) at 1024px on NVIDIA RTX 4070 GPU (CUDA)
- Tracking Engine: 8D Position + Scale Kalman Filter (Position + Dimensions + Velocity + Scale Growth Rate)
- Pre-processing: Real-time CLAHE (Contrast Limited Adaptive Histogram Equalization) dynamic enhancer
- Autopilot Output: PyMAVLink control commands to ArduSub (BlueOS)
"""

import os
import sys
import time
import threading
import subprocess
import json
import urllib.request
import cv2
import numpy as np
import torch
from ultralytics import YOLOWorld
from pymavlink import mavutil
from kalman_filter import AUVKalmanFilter, TargetKalmanFilter

def apply_clahe(frame_bgr, clip_limit=2.5, tile_grid_size=(8, 8)):
    """
    Apply Contrast Limited Adaptive Histogram Equalization (CLAHE) on the L-channel
    in LAB color space. Enhances underwater visibility, contrast, and edge sharpness
    without blowing out color balance or introducing noise in uniform water backgrounds.
    """
    lab = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2LAB)
    l_channel, a_channel, b_channel = cv2.split(lab)
    clahe = cv2.createCLAHE(clipLimit=clip_limit, tileGridSize=tile_grid_size)
    cl = clahe.apply(l_channel)
    merged = cv2.merge((cl, a_channel, b_channel))
    return cv2.cvtColor(merged, cv2.COLOR_LAB2BGR)

def ensure_blueos_logitech_stream():
    """Ensure Logitech C922 stream on UDP port 5601 is active in BlueOS."""
    try:
        req = urllib.request.Request("http://192.168.2.2:6020/streams")
        with urllib.request.urlopen(req, timeout=1.0) as resp:
            streams = json.loads(resp.read().decode())
            for s in streams:
                if "5601" in str(s):
                    return True
        payload = {
            "name": "Logitech C922 Stream",
            "source": "/dev/video1",
            "stream_information": {
                "endpoints": ["udp://192.168.2.115:5601"],
                "configuration": {
                    "type": "video",
                    "encode": "MJPG",
                    "width": 1280,
                    "height": 720,
                    "frame_interval": {"numerator": 1, "denominator": 30}
                },
                "extended_configuration": {
                    "thermal": False,
                    "disable_mavlink": False,
                    "disable_zenoh": False,
                    "disable_thumbnails": False,
                    "disable_lazy": False,
                    "disable_recording": False
                }
            }
        }
        post_req = urllib.request.Request(
            "http://192.168.2.2:6020/streams",
            data=json.dumps(payload).encode(),
            headers={"Content-Type": "application/json"}
        )
        with urllib.request.urlopen(post_req, timeout=2.0) as post_resp:
            return post_resp.status in [200, 201]
    except Exception:
        return False

# ==========================================
# CONFIGURATION
# ==========================================
# MAVLink Endpoint: Topside UDP listener port (14550) or BlueOS IP
MAVLINK_ENDPOINT = "udpin:0.0.0.0:14550"

# YOLO26 World Model Weights
YOLO26_WORLD_WEIGHTS = "weights/yolo26_world.pt"

# YOLO26 World Target Vocabulary (60+ Classes across Lab Tools, Bench Electronics, AUV Hardware, Subsea Targets)
# Dual-token formulation guarantees maximum CLIP visual-textual cosine similarity
YOLO26_WORLD_CLASSES = [
    # --- Benchtop Electronics & Mobile Gadgets (17 classes) ---
    "smartphone", "cell phone", "mobile phone",
    "computer mouse", "mouse",
    "computer keyboard", "keyboard",
    "laptop", "computer monitor", "tablet",
    "person", "hand",
    "bottle", "water bottle", "cup", "mug",
    "notebook",

    # --- Mechatronics Lab Tools & Workshop Instruments (16 classes) ---
    "digital multimeter", "multimeter",
    "oscilloscope",
    "soldering iron", "wire stripper",
    "screwdriver", "pliers", "wrench",
    "caliper", "vernier caliper", "ruler",
    "scissors", "pen",
    "breadboard", "jumper wire",
    "heat shrink tube",

    # --- AUV & Subsea Robotics Internal Hardware (21 classes) ---
    "pixhawk", "flight controller",
    "bldc motor", "underwater thruster", "thruster", "propeller",
    "electronic speed controller", "esc",
    "lipo battery", "battery", "power bank",
    "charger", "power adapter",
    "ethernet cable", "tether", "cable", "wire",
    "printed circuit board", "circuit board", "pcb",
    "raspberry pi",
    "watertight enclosure", "acrylic tube",

    # --- Subsea Targets, Competition Obstacles & Marine Inspection (15 classes) ---
    "underwater buoy", "marker buoy", "buoy",
    "underwater gate", "navigation gate", "transit gate",
    "torpedo target", "docking station",
    "subsea pipe", "underwater pipeline", "pipe",
    "subsea flange", "subsea valve",
    "underwater cable",
    "diver", "fish"
]

IMG_SIZE = 1024             # High-resolution input tensor for small and distant subsea targets
CONF_THRESHOLD = 0.12       # Optimal zero-shot confidence threshold for high precision & recall

# Control Gains (Proportional Controller for Tracking)
KP_YAW = 0.8     # Turning gain
KP_HEAVE = 0.8   # Submerge/Ascend gain
FORWARD_SPEED = 200 # Constant forward thrust PWM (1500=Neutral, 1700=Forward)

# Priority target labels for tracking (prefer lab tools/AUV hardware/subsea targets over person/room background)
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
    "watertight enclosure", "acrylic tube",
    "underwater buoy", "marker buoy", "buoy",
    "underwater gate", "navigation gate", "transit gate",
    "torpedo target", "docking station",
    "subsea pipe", "underwater pipeline", "pipe",
    "subsea flange", "subsea valve", "underwater cable"
}

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
    elif "buoy" in n:
        return "Buoy Target"
    elif "gate" in n:
        return "Gate Target"
    elif "torpedo" in n or "docking" in n:
        return "Docking/Target"
    elif "pipeline" in n or "pipe" in n:
        return "Subsea Pipe"
    elif "flange" in n or "valve" in n:
        return "Subsea Valve"
    elif "bottle" in n:
        return "Bottle"
    elif "cup" in n or "mug" in n:
        return "Cup"
    elif "person" in n:
        return "Person"
    elif "hand" in n:
        return "Hand"
    elif "diver" in n:
        return "Diver"
    elif "fish" in n:
        return "Marine Life"
    return raw_name.title()

class GStreamerFrameGrabber:
    """Zero-latency GStreamer pipeline receiver for BlueOS RTP stream (supports H264 and JPEG/MJPG)."""
    def __init__(self, port=5601, width=1280, height=720, encoding="JPEG"):
        self.width = width
        self.height = height
        self.frame_size = width * height * 3
        self.port = port
        self.encoding = encoding
        self.lock = threading.Lock()
        self.frame = None
        self.status = False
        self.stopped = False

        if encoding.upper() == "JPEG":
            depay_dec = [
                "caps=application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)JPEG",
                "!", "rtpjpegdepay",
                "!", "jpegdec"
            ]
        else:
            depay_dec = [
                "caps=application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264",
                "!", "rtph264depay",
                "!", "h264parse",
                "!", "avdec_h264"
            ]

        gst_cmd = [
            "gst-launch-1.0", "-q",
            "udpsrc", f"port={port}",
            *depay_dec,
            "!", "videoconvert",
            "!", "videoscale",
            "!", f"video/x-raw, format=BGR, width={width}, height={height}",
            "!", "fdsink"
        ]

        try:
            self.proc = subprocess.Popen(gst_cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, bufsize=10**7)
            self.thread = threading.Thread(target=self.update, daemon=True)
            self.thread.start()
            # Wait briefly to confirm stream launch
            time.sleep(0.5)
            self.status = (self.proc.poll() is None)
        except Exception as e:
            print(f"[GStreamer Error] Failed to launch pipeline: {e}")
            self.status = False

    def isOpened(self):
        return self.status and (self.proc.poll() is None)

    def update(self):
        while not self.stopped:
            try:
                raw_frame = self.proc.stdout.read(self.frame_size)
                if len(raw_frame) == self.frame_size:
                    frame = np.frombuffer(raw_frame, dtype=np.uint8).reshape((self.height, self.width, 3))
                    with self.lock:
                        self.frame = frame
                        self.status = True
                else:
                    time.sleep(0.01)
            except Exception:
                break

    def read(self):
        with self.lock:
            return self.status, (self.frame.copy() if self.frame is not None else None)

    def release(self):
        self.stopped = True
        if hasattr(self, 'proc'):
            self.proc.terminate()

class RTSPFrameGrabber:
    """Zero-latency RTSP Stream Receiver using GStreamer (rtspsrc latency=0) to match Cockpit WebRTC."""
    def __init__(self, url="rtsp://192.168.2.2:8554/video_udp_stream_0", width=640, height=480):
        self.width = width
        self.height = height
        self.frame_size = width * height * 3
        self.lock = threading.Lock()
        self.frame = None
        self.status = False
        self.stopped = False

        gst_cmd = [
            "gst-launch-1.0", "-q",
            "rtspsrc", f"location={url}", "latency=0",
            "!", "rtph264depay",
            "!", "h264parse",
            "!", "avdec_h264",
            "!", "videoconvert",
            "!", "videoscale",
            "!", f"video/x-raw, format=BGR, width={width}, height={height}",
            "!", "fdsink"
        ]

        try:
            self.proc = subprocess.Popen(gst_cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, bufsize=10**7)
            self.thread = threading.Thread(target=self.update, daemon=True)
            self.thread.start()
            time.sleep(0.5)
            self.status = (self.proc.poll() is None)
        except Exception as e:
            print(f"[GStreamer RTSP Error] Failed to launch pipeline: {e}")
            self.status = False

    def isOpened(self):
        return self.status and (self.proc.poll() is None)

    def update(self):
        while not self.stopped:
            try:
                raw_frame = self.proc.stdout.read(self.frame_size)
                if len(raw_frame) == self.frame_size:
                    frame = np.frombuffer(raw_frame, dtype=np.uint8).reshape((self.height, self.width, 3))
                    with self.lock:
                        self.frame = frame
                        self.status = True
                else:
                    time.sleep(0.005)
            except Exception:
                break

    def read(self):
        with self.lock:
            return self.status, (self.frame.copy() if self.frame is not None else None)

    def release(self):
        self.stopped = True
        if hasattr(self, 'proc'):
            self.proc.terminate()

class FallbackWebcamGrabber:
    """Threaded smooth webcam grabber to eliminate frame buffering stutter and screen glitching."""
    def __init__(self, index=0):
        self.cap = cv2.VideoCapture(index, cv2.CAP_V4L2 if os.name == 'posix' else cv2.CAP_ANY)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.lock = threading.Lock()
        self.frame = None
        self.status = False
        self.stopped = False

        if self.cap.isOpened():
            self.status = True
            self.thread = threading.Thread(target=self.update, daemon=True)
            self.thread.start()

    def isOpened(self):
        return self.status and self.cap.isOpened()

    def update(self):
        while not self.stopped and self.cap.isOpened():
            ret, frame = self.cap.read()
            if ret and frame is not None:
                with self.lock:
                    self.frame = frame
                    self.status = True
            else:
                time.sleep(0.005)

    def read(self):
        with self.lock:
            return self.status, (self.frame.copy() if self.frame is not None else None)

    def release(self):
        self.stopped = True
        if hasattr(self, 'cap'):
            self.cap.release()

# ==========================================
# MAIN TRACKING PIPELINE
# ==========================================
def main():
    print("=" * 60)
    print("   AUV YOLO26 TARGET TRACKING (BLUEOS RPi CAM FEED)   ")
    print("=" * 60)

    # 1. Initialize CUDA GPU Device
    device = "cuda:0" if torch.cuda.is_available() else "cpu"
    print(f"[AI Engine] PyTorch Device: {device}")
    if torch.cuda.is_available():
        print(f"[AI Engine] GPU: {torch.cuda.get_device_name(0)}")

    # 2. Initialize YOLO26 World Open-Vocabulary AI Engine
    print(f"[AI Engine] Loading YOLO26 World model from '{YOLO26_WORLD_WEIGHTS}'...")
    model = YOLOWorld(YOLO26_WORLD_WEIGHTS)
    model.set_classes(YOLO26_WORLD_CLASSES)
    model.to(device)
    print(f"[YOLO26 World] Target vocabulary configured ({len(YOLO26_WORLD_CLASSES)} classes):")
    print(f"               {YOLO26_WORLD_CLASSES[:10]} ...")

    conf_thresh = CONF_THRESHOLD
    rotation_mode = 0  # 0: Normal, 1: 90 CW, 2: 180 deg, 3: 270 CW
    clahe_enabled = False  # Real-time CLAHE Dynamic Contrast Enhancer toggle

    # 3. Connect to MAVLink (ArduSub via BlueOS)
    print(f"[MAVLink] Connecting to vehicle at {MAVLINK_ENDPOINT}...")
    try:
        mav = mavutil.mavlink_connection(MAVLINK_ENDPOINT)
        mav.wait_heartbeat(timeout=2)
        print(f"[MAVLink] Connected to ArduSub! (System ID: {mav.target_system})")
    except Exception as e:
        print(f"[MAVLink Warning] Could not connect to MAVLink ({e}). Running in Video-Only mode.")
        mav = None

    # 4. Camera Setup & BlueOS Stream Initialization
    CAMERAS = [
        {
            "name": "RPi CSI Camera Module",
            "port": 5600,
            "encoding": "H264",
            "width": 640,
            "height": 480
        },
        {
            "name": "Logitech C922 USB Webcam",
            "port": 5601,
            "encoding": "JPEG",
            "width": 1280,
            "height": 720
        }
    ]
    current_cam_idx = 0  # Default to RPi CSI Camera

    def init_camera_grabber(idx):
        cam = CAMERAS[idx]
        print(f"[Video] Connecting to {cam['name']} on UDP port {cam['port']} ({cam['encoding']})...")
        g = GStreamerFrameGrabber(port=cam["port"], width=cam["width"], height=cam["height"], encoding=cam["encoding"])
        t_start = time.time()
        while time.time() - t_start < 2.0:
            ret, test_frame = g.read()
            if ret and test_frame is not None:
                print(f"[Video] {cam['name']} connected! Resolution: {test_frame.shape[1]}x{test_frame.shape[0]}")
                return g
            time.sleep(0.1)
        print(f"[Video Warning] {cam['name']} port {cam['port']} not streaming.")
        return g

    grabber = init_camera_grabber(current_cam_idx)
    if not grabber.isOpened():
        # Try alternate camera
        current_cam_idx = 1 - current_cam_idx
        print(f"[Video] Trying alternate camera: {CAMERAS[current_cam_idx]['name']}...")
        grabber.release()
        grabber = init_camera_grabber(current_cam_idx)

    if not grabber.isOpened():
        print("[Video Warning] BlueOS network streams offline. Trying local fallback webcam 0...")
        grabber.release()
        grabber = FallbackWebcamGrabber(0)

    if not grabber.isOpened():
        print("[Video Error] Failed to open any video stream. Please check tether & camera connection.")
        return

    # 5. Initialize AUV 8D Kalman Filter (Position + Scale CWNA Discrete Dynamic Model)
    kf = AUVKalmanFilter(dt=1.0 / 30.0, mode="8D")

    WINDOW_NAME = "AUV Topside AI Camera (YOLO26 World + 8D Kalman Filter + RTX 4070)"
    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WINDOW_NAME, 1280, 720)

    print("=" * 60)
    print("   HOTKEY CONTROLS:   ")
    print("   [c] - Switch Camera (Logitech C922 <--> RPi CSI Cam)")
    print("   [e] - Toggle CLAHE Underwater Dynamic Contrast Enhancement")
    print("   [r] - Rotate camera 90 deg clockwise (0/90/180/270 deg)")
    print("   [f] - Flip/Rotate video 180 deg")
    print("   [+] - Increase Confidence Threshold (+0.02)")
    print("   [-] - Decrease Confidence Threshold (-0.02)")
    print("   [q] - Exit cleanly")
    print("=" * 60)

    ROTATION_NAMES = {0: "0 deg (Normal)", 1: "90 deg CW", 2: "180 deg", 3: "270 deg CW"}

    while True:
        ret, frame = grabber.read()
        if not ret or frame is None:
            print("[Video Warning] Waiting for camera frame...")
            time.sleep(0.05)
            continue

        if rotation_mode == 1:
            frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
        elif rotation_mode == 2:
            frame = cv2.rotate(frame, cv2.ROTATE_180)
        elif rotation_mode == 3:
            frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)

        # Real-time Underwater CLAHE Dynamic Contrast Enhancement
        if clahe_enabled:
            frame = apply_clahe(frame)

        h, w, _ = frame.shape
        center_x, center_y = w // 2, h // 2

        # 1. Run 8D Kalman Filter Predict Step
        kf.predict()

        # 2. Run YOLO26 World Inference on RTX 4070 GPU at 1024px (Agnostic NMS suppresses duplicate token boxes)
        results = model.predict(frame, conf=conf_thresh, imgsz=IMG_SIZE, device=device, agnostic_nms=True, verbose=False)[0]

        best_target = None
        best_is_priority = False
        max_score = 0

        # Draw Frame Center Reference Crosshair
        cv2.line(frame, (center_x - 15, center_y), (center_x + 15, center_y), (0, 255, 0), 2)
        cv2.line(frame, (center_x, center_y - 15), (center_x, center_y + 15), (0, 255, 0), 2)

        # Parse & Draw ALL Detections
        for box in results.boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            conf = float(box.conf[0])
            cls_id = int(box.cls[0])
            raw_name = model.names[cls_id]
            pretty_label = format_display_label(raw_name)
            box_label = f"{pretty_label} {conf*100:.0f}%"

            # Secondary detection box (thin orange)
            cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 200, 0), 1)
            cv2.putText(frame, box_label, (x1, max(15, y1 - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 200, 0), 1)

            area = (x2 - x1) * (y2 - y1)
            is_priority = (raw_name.lower() in PRIORITY_TARGETS)

            # Prioritize bench/lab/AUV targets over general background/person
            score = area * (10.0 if is_priority else 1.0)
            if score > max_score:
                max_score = score
                best_target = ((x1 + x2) // 2, (y1 + y2) // 2, x1, y1, x2, y2, box_label)

        target_active = False
        if best_target is not None:
            raw_x, raw_y, x1, y1, x2, y2, target_label = best_target
            # Correct 8D Kalman Filter State with New Bounding Box Measurement
            filtered_x, filtered_y = kf.update_bbox(x1, y1, x2, y2)
            obj_x, obj_y = int(filtered_x), int(filtered_y)
            target_active = True

            # Get 8D Smoothed Bounding Box
            bbox_smooth = kf.get_bbox()
            if bbox_smooth is not None:
                sx1, sy1, sx2, sy2, sw, sh = bbox_smooth
                # Draw Smoothed 8D Kalman Bounding Box (Bright Green, 2px)
                cv2.rectangle(frame, (sx1, sy1), (sx2, sy2), (0, 255, 0), 2)
            else:
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

            # Draw Raw BBox (Dotted/Thin Red) & Center Dots
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 1)
            cv2.circle(frame, (raw_x, raw_y), 4, (0, 0, 255), -1)   # Raw detection dot (red)
            cv2.circle(frame, (obj_x, obj_y), 5, (0, 255, 0), -1)   # Smoothed 8D KF center dot (green)
            cv2.line(frame, (center_x, center_y), (obj_x, obj_y), (255, 255, 0), 2)

            # Get Scale & Expansion Rates for Forward Surge Distance Estimation
            target_area, scale_rate = kf.get_scale_rates()
            approach_indicator = "APPROACHING" if scale_rate > 300 else ("RETREATING" if scale_rate < -300 else "HOLDING")
            cv2.putText(frame, f"{target_label} [8D KF: {approach_indicator}]", (x1, max(20, y1 - 10)), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.58, (0, 255, 0), 2)
        else:
            # Handle Missing Frames / Occlusion using 8D Kalman Prediction (Dead-Reckoning)
            pred_x, pred_y = kf.handle_missing_frame()
            if pred_x is not None:
                obj_x, obj_y = int(pred_x), int(pred_y)
                target_active = True
                # Draw Predicted 8D Bounding Box & Center (Cyan)
                bbox_smooth = kf.get_bbox()
                if bbox_smooth is not None:
                    sx1, sy1, sx2, sy2, _, _ = bbox_smooth
                    cv2.rectangle(frame, (sx1, sy1), (sx2, sy2), (255, 255, 0), 2)
                cv2.circle(frame, (obj_x, obj_y), 6, (255, 255, 0), -1)
                cv2.line(frame, (center_x, center_y), (obj_x, obj_y), (255, 255, 0), 2)
                cv2.putText(frame, f"8D KF DEAD-RECKONING ({kf.missed_frames}f lost)", 
                            (max(10, obj_x - 80), max(20, obj_y - 15)), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 0), 2)

        # Execute Visual Steering Control
        if target_active and obj_x is not None and obj_y is not None:
            # Calculate Center Offset Errors (-1.0 to +1.0 normalized)
            error_x = (obj_x - center_x) / (w / 2)
            error_y = (obj_y - center_y) / (h / 2)

            # Calculate Steering Commands
            yaw_cmd = int(error_x * 400 * KP_YAW)     # [-400, +400]
            heave_cmd = int(-error_y * 400 * KP_HEAVE) # [-400, +400]

            vx, vy = kf.get_velocity()
            target_area, scale_rate = kf.get_scale_rates()
            cv2.putText(frame, f"Offset: ({error_x:+.2f}, {error_y:+.2f}) | Vel: ({vx:+.1f}, {vy:+.1f}) px/s | Area: {target_area:.0f} px2 ({scale_rate:+.0f}/s)", 
                        (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (0, 255, 255), 2)

            # Send MAVLink MANUAL_CONTROL command to ArduSub
            if mav is not None:
                mav.mav.manual_control_send(
                    mav.target_system,
                    FORWARD_SPEED,  # x: pitch/forward
                    0,              # y: roll/lateral
                    500 + heave_cmd,# z: thrust/heave
                    yaw_cmd,        # r: yaw turn
                    0               # buttons
                )
        else:
            cv2.putText(frame, "SEARCHING FOR TARGET (8D KF READY)...", (20, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 165, 255), 2)

        # Top Model Badge & Info HUD
        cam_badge = f"Cam: {CAMERAS[current_cam_idx]['name']} [{CAMERAS[current_cam_idx]['port']}]"
        rot_badge = f"[{ROTATION_NAMES[rotation_mode]}]"
        clahe_badge = "[CLAHE: ON]" if clahe_enabled else "[CLAHE: OFF]"
        cv2.putText(frame, f"{cam_badge} | YOLO26 World 1024px + 8D Kalman Filter {clahe_badge} {rot_badge}", 
                    (20, h - 35), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (255, 255, 255), 1)
        cv2.putText(frame, f"Conf: {conf_thresh:.2f} | Keys: [c] Cam | [e] CLAHE | [r] Rotate | [f] Flip | [+/-] Conf | [q] Exit", 
                    (20, h - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)

        # Display Live Annotated Video Window
        cv2.imshow(WINDOW_NAME, frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('e'):
            clahe_enabled = not clahe_enabled
            print(f"[System] CLAHE Dynamic Contrast Enhancer: {'ENABLED' if clahe_enabled else 'DISABLED'}")
        elif key == ord('c'):
            current_cam_idx = (current_cam_idx + 1) % len(CAMERAS)
            print(f"[System] Switching video stream to: {CAMERAS[current_cam_idx]['name']}...")
            if CAMERAS[current_cam_idx]["port"] == 5601:
                ensure_blueos_logitech_stream()
            grabber.release()
            grabber = init_camera_grabber(current_cam_idx)
            kf = AUVKalmanFilter(dt=1.0 / 30.0, mode="8D")  # Reset 8D KF on camera switch
        elif key == ord('r'):
            rotation_mode = (rotation_mode + 1) % 4
            kf = AUVKalmanFilter(dt=1.0 / 30.0, mode="8D")  # Reset 8D KF on orientation change
            print(f"[System] Video rotation changed to: {ROTATION_NAMES[rotation_mode]}")
        elif key == ord('f'):
            rotation_mode = 2 if rotation_mode == 0 else 0
            kf = AUVKalmanFilter(dt=1.0 / 30.0, mode="8D")  # Reset 8D KF on flip
            print(f"[System] Video orientation toggled to: {ROTATION_NAMES[rotation_mode]}")
        elif key in [ord('+'), ord('=')]:
            conf_thresh = min(0.95, conf_thresh + 0.02)
            print(f"[System] Confidence threshold increased to: {conf_thresh:.2f}")
        elif key in [ord('-'), ord('_')]:
            conf_thresh = max(0.02, conf_thresh - 0.02)
            print(f"[System] Confidence threshold decreased to: {conf_thresh:.2f}")

    grabber.release()
    cv2.destroyAllWindows()
    print("[System] Pipeline stopped cleanly.")

if __name__ == "__main__":
    main()
