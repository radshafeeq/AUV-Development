#!/usr/bin/env python3
"""
AUV Real-Time YOLOv8 Target Detection & Tracking Node
---------------------------------------------------
- Camera Input: BlueOS UDP H.264 Stream from RPi 4B (port 5600)
- AI Engine: Ultralytics YOLOv8 accelerated on NVIDIA RTX 4070 GPU (CUDA)
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
from ultralytics import YOLO
from pymavlink import mavutil

# ==========================================
# CONFIGURATION
# ==========================================
# MAVLink Endpoint: Topside UDP listener port (14550) or BlueOS IP
MAVLINK_ENDPOINT = "udpin:0.0.0.0:14550"

# Target Detection Settings
USE_YOLO_WORLD = True       # Zero-shot open-vocabulary detection without training
YOLO_WORLD_CLASSES = [
    # --- 1. Swimming Pool Competition & Test Tank Targets ---
    "white pvc navigation gate", "yellow pvc pipe gate", "red spherical buoy",
    "yellow cylindrical buoy", "green buoy", "black torpedo target mat",
    "submerged marker dropping bin", "bucket target", "underwater dock structure",
    "black pool lane line", "pool bottom drain grate", "pool ladder",
    "swimming pool wall", "blue pool tile floor", "scuba diver", "swimmer",

    # --- 2. Regular Open Water & Ocean Targets ---
    "underwater pipeline", "submerged metal frame", "underwater pipe flange",
    "submerged valve handle", "black rov tether cable", "underwater connector",
    "ship hull plating", "sunken vessel structure", "anchor chain",
    "underwater buoy", "underwater gate", "propeller", "rov frame",
    "fish", "coral reef", "sea floor sand", "underwater rock",

    # --- 3. Mechatronics & Robotics Lab ---
    "bldc motor", "stepper motor", "servo motor", "pixhawk", "flight controller",
    "esc", "electronic module", "circuit board", "pcb", "soldering iron",
    "multimeter", "oscilloscope", "power supply", "battery", "li-po battery",
    "cables", "wires", "breadboard", "microcontroller", "arduino", "raspberry pi",
    "3d printer", "caliper", "screwdriver", "pliers", "wrench", "lab bench", "fume hood",

    # --- 4. Office & Campus Workspace ---
    "laptop", "computer monitor", "desktop computer", "keyboard", "mouse",
    "office chair", "office desk", "stapler", "printer", "calculator",
    "water bottle", "whiteboard", "trash can", "fire extinguisher", "exit sign", "fire hydrant",

    # --- 5. Study Room, Classroom & Home ---
    "person", "textbook", "notebook", "pen", "pencil", "backpack", "reading lamp",
    "headphones", "tablet", "smartphone", "glasses", "projector", "podium",
    "classroom chair", "classroom desk", "wall clock", "sofa", "television", "charger",

    # --- 6. Room Architecture & Fixtures ---
    "window", "door", "air conditioner", "ceiling light", "ceiling fan", "speaker"
]

CUSTOM_WEIGHTS = "runs/detect/yolo26_combined_model/weights/best.pt" if os.path.exists("runs/detect/yolo26_combined_model/weights/best.pt") else ("runs/detect/combined_model/weights/best.pt" if os.path.exists("runs/detect/combined_model/weights/best.pt") else "runs/detect/office_model/weights/best.pt")
MODEL_NAME = "yolov8s-world.pt" if USE_YOLO_WORLD else (CUSTOM_WEIGHTS if os.path.exists(CUSTOM_WEIGHTS) else "yolov8n.pt")
CONF_THRESHOLD = 0.15       # Confidence threshold for open-vocabulary detection

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

    # 2. Load YOLO Model (YOLO-World Open Vocabulary or standard YOLO)
    print(f"[AI Engine] Loading model '{MODEL_NAME}'...")
    if USE_YOLO_WORLD:
        from ultralytics import YOLOWorld
        model = YOLOWorld(MODEL_NAME)
        model.set_classes(YOLO_WORLD_CLASSES)
        print(f"[YOLO-World] Target classes set: {YOLO_WORLD_CLASSES}")
    else:
        model = YOLO(MODEL_NAME)
    model.to(device)

    # 3. Connect to MAVLink (ArduSub via BlueOS)
    print(f"[MAVLink] Connecting to vehicle at {MAVLINK_ENDPOINT}...")
    try:
        mav = mavutil.mavlink_connection(MAVLINK_ENDPOINT)
        mav.wait_heartbeat(timeout=5)
        print(f"[MAVLink] Connected to ArduSub! (System ID: {mav.target_system})")
    except Exception as e:
        print(f"[MAVLink Warning] Could not connect to MAVLink ({e}). Running in Video-Only mode.")
        mav = None

    # 4. Open BlueOS 1.4.5 RTSP Camera Stream
    rtsp_url = "rtsp://192.168.2.2:8554/video_udp_stream_0"
    print(f"[Video] Connecting to BlueOS 1.4.5 RTSP Stream: {rtsp_url}...")
    grabber = RTSPFrameGrabber(rtsp_url)
    
    if not grabber.isOpened():
        print("[Video Warning] BlueOS RTSP stream not found. Trying GStreamer UDP port 5600...")
        grabber = GStreamerFrameGrabber(port=5600, width=640, height=480)

    if not grabber.isOpened():
        print("[Video Warning] BlueOS network stream not found. Falling back to local camera index 0...")
        grabber = FallbackWebcamGrabber(0)

    if not grabber.isOpened():
        print("[Video Error] Failed to open any video stream. Please check tether & BlueOS connection.")
        return

    engine_name = "YOLO-World (Open Vocabulary)" if USE_YOLO_WORLD else ("YOLO26 Combined" if "yolo26" in MODEL_NAME else "YOLOv8")
    WINDOW_NAME = f"AUV Topside AI Camera ({engine_name} + RTX 4070)"
    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WINDOW_NAME, 1280, 720)

    print("[System] Tracking active! Press 'q' in the display window to exit.")

    while True:
        ret, frame = grabber.read()
        if not ret or frame is None:
            print("[Video Warning] Waiting for camera frame...")
            time.sleep(0.05)
            continue

        h, w, _ = frame.shape
        center_x, center_y = w // 2, h // 2

        # Run YOLO Inference on GPU with fixed resolution (eliminates memory reallocation glitching)
        results = model.predict(frame, conf=CONF_THRESHOLD, imgsz=640, device=device, verbose=False)[0]

        best_target = None
        max_area = 0

        # Draw Frame Center Reference Crosshair
        cv2.line(frame, (center_x - 15, center_y), (center_x + 15, center_y), (0, 255, 0), 2)
        cv2.line(frame, (center_x, center_y - 15), (center_x, center_y + 15), (0, 255, 0), 2)

        # Parse Detections
        for box in results.boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            conf = float(box.conf[0])
            cls_id = int(box.cls[0])
            label = f"{model.names[cls_id]} {conf:.2f}"

            area = (x2 - x1) * (y2 - y1)
            if area > max_area:
                max_area = area
                best_target = ( (x1 + x2) // 2, (y1 + y2) // 2, x1, y1, x2, y2, label )

        # Execute Visual Tracking
        if best_target is not None:
            obj_x, obj_y, x1, y1, x2, y2, label = best_target

            # Draw Bounding Box & Target Vector
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
            cv2.circle(frame, (obj_x, obj_y), 5, (0, 0, 255), -1)
            cv2.line(frame, (center_x, center_y), (obj_x, obj_y), (255, 255, 0), 2)
            cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

            # Calculate Center Offset Errors (-1.0 to +1.0 normalized)
            error_x = (obj_x - center_x) / (w / 2)
            error_y = (obj_y - center_y) / (h / 2)

            # Calculate Steering Commands
            yaw_cmd = int(error_x * 400 * KP_YAW)     # [-400, +400]
            heave_cmd = int(-error_y * 400 * KP_HEAVE) # [-400, +400]

            cv2.putText(frame, f"Tracking Error: X={error_x:+.2f}, Y={error_y:+.2f}", 
                        (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

            # Send MAVLink MANUAL_CONTROL command to ArduSub
            if mav is not None:
                mav.mav.manual_control_send(
                    mav.target_system,
                    FORWARD_SPEED,  # x: pitch/forward (e.g. 200)
                    0,              # y: roll/lateral
                    500 + heave_cmd,# z: thrust/heave (0-1000)
                    yaw_cmd,        # r: yaw turn
                    0               # buttons
                )
        else:
            cv2.putText(frame, "SEARCHING FOR TARGET...", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)

        # Display Engine Badge Overlay
        cv2.putText(frame, f"Engine: {engine_name}", (20, h - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        # Display Live Annotated Video Window
        cv2.imshow(WINDOW_NAME, frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    grabber.release()
    cv2.destroyAllWindows()
    print("[System] Pipeline stopped cleanly.")

if __name__ == "__main__":
    main()
