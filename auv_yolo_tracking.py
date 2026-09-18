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
import cv2
import numpy as np
import torch
from ultralytics import YOLOWorld
from pymavlink import mavutil
from kalman_filter import TargetKalmanFilter

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
    """Zero-latency GStreamer pipeline receiver for BlueOS RTP H.264 stream (Matches Cockpit performance)."""
    def __init__(self, port=5600, width=1280, height=720):
        self.width = width
        self.height = height
        self.frame_size = width * height * 3
        self.lock = threading.Lock()
        self.frame = None
        self.status = False
        self.stopped = False

        gst_cmd = [
            "gst-launch-1.0", "-q",
            "udpsrc", f"port={port}",
            "caps=application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264",
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

    # 4. Open BlueOS Camera Stream (Primary: UDP port 5600 zero-latency RTP H.264)
    print("[Video] Initializing GStreamer RTP H.264 stream receiver on UDP port 5600...")
    grabber = GStreamerFrameGrabber(port=5600, width=640, height=480)

    # Confirm frame reception within 2 seconds
    t_start = time.time()
    stream_ok = False
    while time.time() - t_start < 2.0:
        ret, test_frame = grabber.read()
        if ret and test_frame is not None:
            stream_ok = True
            print(f"[Video] UDP 5600 stream connected! Resolution: {test_frame.shape[1]}x{test_frame.shape[0]}")
            break
        time.sleep(0.1)

    if not stream_ok:
        print("[Video Warning] UDP port 5600 not streaming. Trying BlueOS RTSP stream...")
        grabber.release()
        rtsp_url = "rtsp://192.168.2.2:8554/video_udp_stream_0"
        grabber = RTSPFrameGrabber(rtsp_url)
        t_start = time.time()
        while time.time() - t_start < 2.0:
            ret, test_frame = grabber.read()
            if ret and test_frame is not None:
                stream_ok = True
                print(f"[Video] RTSP stream connected! Resolution: {test_frame.shape[1]}x{test_frame.shape[0]}")
                break
            time.sleep(0.1)

    if not stream_ok:
        print("[Video Warning] Network streams offline. Falling back to local webcam index 0...")
        grabber.release()
        grabber = FallbackWebcamGrabber(0)

    if not grabber.isOpened():
        print("[Video Error] Failed to open any video stream. Please check tether & camera connection.")
        return

    # 5. Initialize Target Kalman Filter
    kf = TargetKalmanFilter(dt=0.033)

    WINDOW_NAME = "AUV Topside AI Camera (YOLO26 World + Kalman Filter + RTX 4070)"
    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WINDOW_NAME, 1280, 720)

    print("=" * 60)
    print("   HOTKEY CONTROLS:   ")
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
        rot_badge = f"[{ROTATION_NAMES[rotation_mode]}]"
        cv2.putText(frame, f"Engine: YOLO26 World (Open Vocabulary) + Kalman Filter {rot_badge}", 
                    (20, h - 35), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (255, 255, 255), 1)
        cv2.putText(frame, f"Conf: {conf_thresh:.2f} | Keys: [r] Rotate 90 | [f] Flip 180 | [+/-] Conf | [q] Exit", 
                    (20, h - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)

        # Display Live Annotated Video Window
        cv2.imshow(WINDOW_NAME, frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('r'):
            rotation_mode = (rotation_mode + 1) % 4
            kf = TargetKalmanFilter(dt=0.033) # Reset KF on orientation change
            print(f"[System] Video rotation changed to: {ROTATION_NAMES[rotation_mode]}")
        elif key == ord('f'):
            rotation_mode = 2 if rotation_mode == 0 else 0
            kf = TargetKalmanFilter(dt=0.033) # Reset KF on flip
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
