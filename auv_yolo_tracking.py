#!/usr/bin/env python3
"""
AUV Real-Time YOLO26 World Target Detection & Tracking Node
----------------------------------------------------------
- Camera Input: BlueOS UDP H.264 Stream from RPi 4B (port 5600)
- AI Engine: YOLO26 World (Open-Vocabulary Zero-Shot Vision) on NVIDIA RTX 4070 GPU (CUDA)
- Tracking Engine: 4D Constant-Velocity Kalman Filter (Position + Velocity + Dead-Reckoning)
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

# High-Precision Target Vocabulary for YOLO26 World
# (Dual-token formulation ensures maximum CLIP visual cosine similarity)
YOLO26_WORLD_CLASSES = [
    # --- Desktop & Bench-Testing Gadgets ---
    "smartphone", "cell phone", "mobile phone",
    "computer mouse", "mouse",
    "computer keyboard", "keyboard",
    "laptop", "computer monitor",
    "person", "hand",
    "bottle", "water bottle", "cup",
    "scissors", "pen", "notebook",

    # --- Mechatronics & AUV Hardware Targets ---
    "pixhawk", "flight controller",
    "bldc motor", "motor", "propeller",
    "esc", "speed controller",
    "battery", "power bank",
    "charger", "power adapter",
    "cable", "wire", "tether",
    "circuit board", "pcb",

    # --- Underwater & Field Targets ---
    "underwater buoy", "buoy", "underwater gate", "pipe"
]

CONF_THRESHOLD = 0.12       # Optimal zero-shot confidence threshold for high precision & recall

# Control Gains (Proportional Controller for Tracking)
KP_YAW = 0.8     # Turning gain
KP_HEAVE = 0.8   # Submerge/Ascend gain
FORWARD_SPEED = 200 # Constant forward thrust PWM (1500=Neutral, 1700=Forward)

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

    # Priority target labels for tracking (prefer gadgets/tools over room background/person)
    PRIORITY_TARGETS = {
        "smartphone", "cell phone", "mobile phone",
        "computer mouse", "mouse",
        "computer keyboard", "keyboard",
        "laptop", "computer monitor",
        "pixhawk", "flight controller",
        "bldc motor", "motor", "propeller",
        "esc", "speed controller",
        "battery", "power bank",
        "charger", "power adapter",
        "cable", "wire", "tether",
        "bottle", "water bottle", "cup", "mug",
        "scissors", "pen", "notebook",
        "underwater buoy", "buoy", "underwater gate", "pipe"
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
        elif "pixhawk" in n or "flight controller" in n:
            return "Pixhawk"
        elif "motor" in n or "propeller" in n:
            return "BLDC Motor"
        elif "esc" in n or "speed controller" in n:
            return "ESC"
        elif "battery" in n or "power bank" in n:
            return "Battery"
        elif "charger" in n or "adapter" in n:
            return "Charger"
        elif "cable" in n or "wire" in n or "tether" in n:
            return "Cable"
        elif "buoy" in n:
            return "Buoy Target"
        elif "gate" in n or "pipe" in n:
            return "Gate Target"
        elif "bottle" in n:
            return "Bottle"
        elif "cup" in n or "mug" in n:
            return "Cup"
        elif "person" in n:
            return "Person"
        elif "hand" in n:
            return "Hand"
        return raw_name.title()

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
    # Ensure Logitech C922 stream on port 5601 is configured in BlueOS
    print("[BlueOS] Checking & ensuring Logitech C922 stream on UDP 5601...")
    ensure_blueos_logitech_stream()

    CAMERAS = [
        {
            "name": "Logitech C922 USB Webcam",
            "port": 5601,
            "encoding": "JPEG",
            "width": 1280,
            "height": 720
        },
        {
            "name": "RPi CSI Camera Module",
            "port": 5600,
            "encoding": "H264",
            "width": 640,
            "height": 480
        }
    ]
    current_cam_idx = 0  # Default to Logitech C922

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

    # 5. Initialize AUV Kalman Filter (4D Constant-Velocity CWNA Model)
    kf = AUVKalmanFilter(dt=1.0 / 30.0)

    WINDOW_NAME = "AUV Topside AI Camera (YOLO26 World + Kalman Filter + RTX 4070)"
    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WINDOW_NAME, 1280, 720)

    print("=" * 60)
    print("   HOTKEY CONTROLS:   ")
    print("   [c] - Switch Camera (Logitech C922 <--> RPi CSI Cam)")
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

        h, w, _ = frame.shape
        center_x, center_y = w // 2, h // 2

        # 1. Run Kalman Filter Predict Step
        kf.predict()

        # 2. Run YOLO26 World Inference on RTX 4070 GPU (with Agnostic NMS to suppress overlapping token duplicates)
        results = model.predict(frame, conf=conf_thresh, imgsz=640, device=device, agnostic_nms=True, verbose=False)[0]

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

            # Secondary detection box (thin cyan/orange)
            cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 200, 0), 1)
            cv2.putText(frame, box_label, (x1, max(15, y1 - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 200, 0), 1)

            area = (x2 - x1) * (y2 - y1)
            is_priority = (raw_name.lower() in PRIORITY_TARGETS)

            # Prioritize bench/AUV targets over general background/person
            score = area * (10.0 if is_priority else 1.0)
            if score > max_score:
                max_score = score
                best_target = ((x1 + x2) // 2, (y1 + y2) // 2, x1, y1, x2, y2, box_label)

        target_active = False
        if best_target is not None:
            raw_x, raw_y, x1, y1, x2, y2, target_label = best_target
            # Correct Kalman Filter State with New Detection
            filtered_x, filtered_y = kf.update(raw_x, raw_y)
            obj_x, obj_y = int(filtered_x), int(filtered_y)
            target_active = True

            # Draw Primary Tracked Target: Box (Red), Raw Center (Yellow), Filtered Center (Green)
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
            cv2.circle(frame, (raw_x, raw_y), 4, (0, 255, 255), -1) # Raw detection dot
            cv2.circle(frame, (obj_x, obj_y), 6, (0, 255, 0), -1)   # Smoothed KF dot
            cv2.line(frame, (center_x, center_y), (obj_x, obj_y), (255, 255, 0), 2)
            cv2.putText(frame, f"{target_label} [KF TRACK]", (x1, max(20, y1 - 10)), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        else:
            # Handle Missing Frames / Occlusion using Kalman Prediction
            pred_x, pred_y = kf.handle_missing_frame()
            if pred_x is not None:
                obj_x, obj_y = int(pred_x), int(pred_y)
                target_active = True
                # Draw Predicted Target Center & Vector (Cyan)
                cv2.circle(frame, (obj_x, obj_y), 6, (255, 255, 0), -1)
                cv2.line(frame, (center_x, center_y), (obj_x, obj_y), (255, 255, 0), 2)
                cv2.putText(frame, f"KF PREDICTING ({kf.missed_frames}f lost)", (max(10, obj_x - 60), max(20, obj_y - 15)), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

        # Execute Visual Steering Control
        if target_active and obj_x is not None and obj_y is not None:
            # Calculate Center Offset Errors (-1.0 to +1.0 normalized)
            error_x = (obj_x - center_x) / (w / 2)
            error_y = (obj_y - center_y) / (h / 2)

            # Calculate Steering Commands
            yaw_cmd = int(error_x * 400 * KP_YAW)     # [-400, +400]
            heave_cmd = int(-error_y * 400 * KP_HEAVE) # [-400, +400]

            vx, vy = kf.get_velocity()
            cv2.putText(frame, f"Target Offset: X={error_x:+.2f}, Y={error_y:+.2f} | KF Vel: ({vx:+.1f}, {vy:+.1f}) px/s", 
                        (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)

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
            cv2.putText(frame, "SEARCHING FOR TARGET (KF READY)...", (20, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 165, 255), 2)

        # Top Model Badge & Info HUD
        cam_badge = f"Cam: {CAMERAS[current_cam_idx]['name']} [{CAMERAS[current_cam_idx]['port']}]"
        rot_badge = f"[{ROTATION_NAMES[rotation_mode]}]"
        cv2.putText(frame, f"{cam_badge} | YOLO26 World + Kalman Filter {rot_badge}", 
                    (20, h - 35), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (255, 255, 255), 1)
        cv2.putText(frame, f"Conf: {conf_thresh:.2f} | Keys: [c] Cam | [r] Rotate | [f] Flip | [+/-] Conf | [q] Exit", 
                    (20, h - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)

        # Display Live Annotated Video Window
        cv2.imshow(WINDOW_NAME, frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('c'):
            current_cam_idx = (current_cam_idx + 1) % len(CAMERAS)
            print(f"[System] Switching video stream to: {CAMERAS[current_cam_idx]['name']}...")
            grabber.release()
            grabber = init_camera_grabber(current_cam_idx)
            kf = AUVKalmanFilter(dt=1.0 / 30.0)  # Reset KF on camera switch
        elif key == ord('r'):
            rotation_mode = (rotation_mode + 1) % 4
            kf = AUVKalmanFilter(dt=1.0 / 30.0)  # Reset KF on orientation change
            print(f"[System] Video rotation changed to: {ROTATION_NAMES[rotation_mode]}")
        elif key == ord('f'):
            rotation_mode = 2 if rotation_mode == 0 else 0
            kf = AUVKalmanFilter(dt=1.0 / 30.0)  # Reset KF on flip
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
