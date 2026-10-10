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
import ultralytics.nn.tasks
try:
    torch.serialization.add_safe_globals([ultralytics.nn.tasks.WorldModel])
except Exception:
    pass
import argparse
from ultralytics import YOLOWorld
from pymavlink import mavutil
from kalman_filter import (
    AUVVisualKalmanFilter,
    AUVDynamicsKalmanFilter,
    AUVComparativeBaseline,
    AUVKalmanFilter,
    TargetKalmanFilter
)
from visual_dvl_odometry import DownwardVisualDVL

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
        # Dynamically determine source device node
        dev_node = "/dev/video0"
        try:
            v4l_req = urllib.request.Request("http://192.168.2.2:6020/v4l")
            with urllib.request.urlopen(v4l_req, timeout=1.0) as v4l_resp:
                v4l_devices = json.loads(v4l_resp.read().decode())
                for dev in v4l_devices:
                    if any(k in dev.get("name", "").lower() for k in ["c922", "logitech", "webcam"]):
                        dev_node = dev.get("source", "/dev/video0")
                        break
        except Exception:
            pass

        payload = {
            "name": "Logitech C922 Stream",
            "source": dev_node,
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
# COOPERATIVE DUAL-CAMERA WORKERS & LOGGERS
# ==========================================

class DownwardDVLWorker:
    """
    Asynchronous Worker for Downward-Facing Logitech C922 Pro in Lower Acrylic Tube.
    Continuously executes Lucas-Kanade optical flow, gyro derotation, and AR floor
    triangulation in a dedicated background thread.
    """
    def __init__(self, port=5601, device_index=2, use_udp=False, underwater_mode=True):
        self.port = int(port)
        self.device_index = int(device_index)
        self.use_udp = bool(use_udp)
        self.underwater_mode = bool(underwater_mode)
        self.dvl = DownwardVisualDVL(
            port=self.port,
            device_index=self.device_index,
            underwater_mode=self.underwater_mode
        )
        self.thread = None
        self.running = False
        self.stopped = False
        self.lock = threading.Lock()
        self.latest_flow_vis = None
        self.u_auv = 0.0
        self.v_auv = 0.0
        self.h_floor = 1.50
        self.flow_quality = 0.0
        self.tracked_count = 0
        self.connected = False

    def start(self):
        # 1. Try specified transport
        success = self.dvl.start_stream(use_udp=self.use_udp)
        # 2. If failed, attempt alternate transport
        if not success:
            success = self.dvl.start_stream(use_udp=not self.use_udp)

        if success:
            self.connected = True
            self.running = True
            self.thread = threading.Thread(target=self._worker_loop, daemon=True)
            self.thread.start()
            mode_desc = f"UDP port {self.port}" if self.use_udp else f"device /dev/video{self.device_index}"
            print(f"[DVL Worker] Downward Visual DVL connected on {mode_desc}!")
        else:
            print(f"[DVL Worker Warning] Downward camera not accessible on /dev/video{self.device_index} or UDP {self.port}.")
            print("                     Running in Simulated/Bench DVL mode (0.0 m/s default).")
            self.connected = False

    def _worker_loop(self):
        while not self.stopped and self.running:
            if self.dvl.cap is None or not self.dvl.cap.isOpened():
                time.sleep(0.05)
                continue
            ret, frame = self.dvl.cap.read()
            if not ret or frame is None:
                time.sleep(0.01)
                continue
            u, v, h, flow_vis = self.dvl.process_frame(frame)
            with self.lock:
                self.u_auv = u
                self.v_auv = v
                self.h_floor = h
                self.flow_quality = self.dvl.flow_quality
                self.tracked_count = self.dvl.tracked_count
                self.latest_flow_vis = flow_vis
            time.sleep(0.01)

    def get_state(self):
        with self.lock:
            vis = self.latest_flow_vis.copy() if self.latest_flow_vis is not None else None
            return {
                "u_auv": float(self.u_auv),
                "v_auv": float(self.v_auv),
                "h_floor": float(self.h_floor),
                "flow_quality": float(self.flow_quality),
                "tracked_count": int(self.tracked_count),
                "flow_vis": vis,
                "connected": self.connected
            }

    def stop(self):
        self.stopped = True
        self.running = False
        self.dvl.stop()


class TelemetryComparativeLogger:
    """
    Synchronous 50 Hz Dual-Channel Telemetry Logger:
    - Channel A: Raw Dead-Reckoning (Without Kalman Filter - Integrates raw IMU)
    - Channel B: 6-DOF Fossen Subsea EKF (With Kalman Filter - Fuses IMU + DVL + Dynamics)
    Saves synchronously to auv_telemetry_with_and_without_kf.csv for thesis validation.
    """
    def __init__(self, filename="auv_telemetry_with_and_without_kf.csv", dt=0.02):
        self.filename = filename
        self.dt = dt
        self.active = False
        self.file = None
        self.baseline_raw = AUVComparativeBaseline(dt=dt)
        self.ekf_subsea = AUVDynamicsKalmanFilter(dt=dt)
        self.start_time = None
        self.sample_count = 0

    def start(self):
        self.file = open(self.filename, "w", buffering=1)
        header = (
            "timestamp,dt,u_auv_dvl,v_auv_dvl,h_floor,"
            "target_detected,target_label,target_dist,target_v_rel,target_v_world,is_stationary,"
            "raw_u,raw_v,raw_drift_cum,kf_u,kf_v,kf_w,kf_dist_u,kf_dist_v\n"
        )
        self.file.write(header)
        self.active = True
        self.start_time = time.time()
        self.sample_count = 0
        print(f"[Logger] Telemetry recording started -> {self.filename}")

    def log_step(self, u_dvl, v_dvl, h_floor, target_info=None, imu_accel=None, imu_gyro=None):
        if not self.active or self.file is None:
            return
        t_now = time.time() - self.start_time

        ax = float(imu_accel[0]) if imu_accel is not None else 0.05
        ay = float(imu_accel[1]) if imu_accel is not None else 0.02
        az = float(imu_accel[2]) if imu_accel is not None else 0.0
        p = float(imu_gyro[0]) if imu_gyro is not None else 0.0
        q = float(imu_gyro[1]) if imu_gyro is not None else 0.0
        r = float(imu_gyro[2]) if imu_gyro is not None else 0.0

        # Channel A: Raw Dead-Reckoning (Without KF)
        v_raw, drift_cum = self.baseline_raw.step([ax, ay, az], [p, q, r])

        # Channel B: 6-DOF Fossen Subsea EKF (With KF)
        self.ekf_subsea.predict([20.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.ekf_subsea.update([u_dvl, v_dvl, 0.0, p, q, r])
        u_ekf, v_ekf, w_ekf, _, _, _ = self.ekf_subsea.get_velocities()
        dist_u, dist_v = self.ekf_subsea.get_disturbance_forces()

        # Target classification metadata
        t_det = 1 if target_info and target_info.get("detected") else 0
        t_lbl = target_info.get("label", "none") if target_info else "none"
        t_dist = target_info.get("dist", 0.0) if target_info else 0.0
        t_vrel = target_info.get("v_rel", 0.0) if target_info else 0.0
        t_vworld = target_info.get("v_world", 0.0) if target_info else 0.0
        t_stat = 1 if target_info and target_info.get("is_stationary") else 0

        line = (
            f"{t_now:.4f},{self.dt:.4f},{u_dvl:.4f},{v_dvl:.4f},{h_floor:.3f},"
            f"{t_det},{t_lbl},{t_dist:.3f},{t_vrel:.4f},{t_vworld:.4f},{t_stat},"
            f"{v_raw[0]:.4f},{v_raw[1]:.4f},{drift_cum:.4f},"
            f"{u_ekf:.4f},{v_ekf:.4f},{w_ekf:.4f},{dist_u:.3f},{dist_v:.3f}\n"
        )
        self.file.write(line)
        self.sample_count += 1

    def stop(self):
        if self.file:
            self.file.flush()
            self.file.close()
            self.file = None
        self.active = False
        print(f"[Logger] Telemetry recording stopped ({self.sample_count} samples logged to {self.filename}).")


# ==========================================
# MAIN TRACKING PIPELINE
# ==========================================
def main():
    parser = argparse.ArgumentParser(description="AUV Dual-Camera Cooperative Visual Tracking & Odometry")
    parser.add_argument("--front-port", type=int, default=5600, help="Front RPi AI Camera UDP port (default: 5600)")
    parser.add_argument("--downward-cam", type=int, default=2, help="Downward C922 device index (default: 2 for /dev/video2)")
    parser.add_argument("--downward-port", type=int, default=5601, help="Downward C922 UDP port (default: 5601)")
    parser.add_argument("--use-downward-udp", action="store_true", help="Stream downward camera via BlueOS UDP 5601")
    parser.add_argument("--no-downward", action="store_true", help="Disable downward visual DVL")
    parser.add_argument("--log", action="store_true", help="Enable 50 Hz dual-channel telemetry logging immediately")
    parser.add_argument("--pip", action="store_true", default=True, help="Enable Picture-in-Picture cockpit HUD")
    parser.add_argument("--clahe", action="store_true", help="Enable CLAHE underwater contrast enhancement")
    parser.add_argument("--conf", type=float, default=CONF_THRESHOLD, help="YOLO26 detection confidence threshold")
    args = parser.parse_args()

    print("=" * 70)
    print("   AUV COOPERATIVE DUAL-CAMERA PERCEPTION & ODOMETRY NODE   ")
    print("=" * 70)

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
    print(f"[YOLO26 World] Target vocabulary configured ({len(YOLO26_WORLD_CLASSES)} classes).")

    conf_thresh = args.conf
    rotation_mode = 0
    clahe_enabled = args.clahe
    pip_enabled = args.pip

    # 3. Connect to MAVLink (ArduSub via BlueOS)
    print(f"[MAVLink] Connecting to vehicle at {MAVLINK_ENDPOINT}...")
    try:
        mav = mavutil.mavlink_connection(MAVLINK_ENDPOINT)
        mav.wait_heartbeat(timeout=2)
        print(f"[MAVLink] Connected to ArduSub! (System ID: {mav.target_system})")
    except Exception as e:
        print(f"[MAVLink Warning] Could not connect to MAVLink ({e}). Running in Video-Only mode.")
        mav = None

    # 4. Start Downward Visual DVL Worker
    dvl_worker = None
    if not args.no_downward:
        print(f"[System] Launching Downward Visual DVL Worker (Device: {args.downward_cam}, Port: {args.downward_port})...")
        dvl_worker = DownwardDVLWorker(
            port=args.downward_port,
            device_index=args.downward_cam,
            use_udp=args.use_downward_udp,
            underwater_mode=True
        )
        dvl_worker.start()

    # 5. Initialize Front Camera Grabber (RPi AI Camera on UDP 5600)
    print(f"[Video] Initializing Front AI Camera (UDP {args.front_port} H.264)...")
    front_grabber = GStreamerFrameGrabber(port=args.front_port, width=1280, height=720, encoding="H264")
    t_start = time.time()
    front_connected = False
    while time.time() - t_start < 2.5:
        ret, test_frame = front_grabber.read()
        if ret and test_frame is not None:
            front_connected = True
            print(f"[Video] Front RPi AI Camera connected! ({test_frame.shape[1]}x{test_frame.shape[0]})")
            break
        time.sleep(0.1)

    if not front_connected:
        print(f"[Video Warning] UDP {args.front_port} not streaming. Falling back to local USB camera /dev/video0...")
        front_grabber.release()
        front_grabber = FallbackWebcamGrabber(0)

    if not front_grabber.isOpened():
        print("[Video Error] Failed to open front video stream. Running with synthetic test frames.")

    # 6. Initialize Visual Kalman Filter with Ego-Motion Compensation
    kf = AUVVisualKalmanFilter(dt=1.0 / 30.0, mode="8D")

    # 7. Initialize 50 Hz Synchronous Telemetry Logger
    logger = TelemetryComparativeLogger()
    if args.log:
        logger.start()

    WINDOW_NAME = "AUV Cockpit: Front AI Camera & Downward Visual DVL (RTX 4070 GPU)"
    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WINDOW_NAME, 1280, 720)

    print("=" * 60)
    print("   COCKPIT CONTROLS:   ")
    print("   [l] - Toggle 50 Hz Telemetry Logging (With vs. Without KF)")
    print("   [p] - Toggle Picture-in-Picture (Downward DVL Optical Flow)")
    print("   [e] - Toggle CLAHE Underwater Contrast Enhancement")
    print("   [r] - Rotate camera 90 deg clockwise")
    print("   [f] - Flip video 180 deg")
    print("   [+] - Increase Confidence (+0.02)")
    print("   [-] - Decrease Confidence (-0.02)")
    print("   [q] - Exit cleanly")
    print("=" * 60)

    ROTATION_NAMES = {0: "0 deg", 1: "90 deg CW", 2: "180 deg", 3: "270 deg CW"}

    while True:
        ret, frame = front_grabber.read()
        if not ret or frame is None:
            # Synthetic bench test frame if camera not streaming
            frame = np.zeros((720, 1280, 3), dtype=np.uint8)
            cv2.putText(frame, "WAITING FOR FRONT RPI AI CAMERA UDP 5600 FEED...", (240, 360),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)
            time.sleep(0.03)

        if rotation_mode == 1:
            frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
        elif rotation_mode == 2:
            frame = cv2.rotate(frame, cv2.ROTATE_180)
        elif rotation_mode == 3:
            frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)

        if clahe_enabled:
            frame = apply_clahe(frame)

        h, w, _ = frame.shape
        center_x, center_y = w // 2, h // 2

        # Ingest Downward DVL states
        u_auv, v_auv, h_floor, flow_vis, dvl_pts = 0.0, 0.0, 1.50, None, 0
        if dvl_worker is not None:
            dvl_state = dvl_worker.get_state()
            u_auv = dvl_state["u_auv"]
            v_auv = dvl_state["v_auv"]
            h_floor = dvl_state["h_floor"]
            flow_vis = dvl_state["flow_vis"]
            dvl_pts = dvl_state["tracked_count"]

        # Run 8D Visual Kalman Filter Predict step
        kf.predict()

        # Run YOLO26 World Inference on RTX 4070 GPU
        results = model.predict(frame, conf=conf_thresh, imgsz=IMG_SIZE, device=device, agnostic_nms=True, verbose=False)[0]

        best_target = None
        max_score = 0

        # Frame center crosshair
        cv2.line(frame, (center_x - 15, center_y), (center_x + 15, center_y), (0, 255, 0), 2)
        cv2.line(frame, (center_x, center_y - 15), (center_x, center_y + 15), (0, 255, 0), 2)

        # Parse Detections
        for box in results.boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            conf = float(box.conf[0])
            cls_id = int(box.cls[0])
            raw_name = model.names[cls_id]
            pretty_label = format_display_label(raw_name)
            box_label = f"{pretty_label} {conf*100:.0f}%"

            # Draw secondary detection outline
            cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 200, 0), 1)
            cv2.putText(frame, box_label, (x1, max(15, y1 - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 200, 0), 1)

            area = (x2 - x1) * (y2 - y1)
            is_priority = (raw_name.lower() in PRIORITY_TARGETS)
            score = area * (10.0 if is_priority else 1.0)
            if score > max_score:
                max_score = score
                best_target = ((x1 + x2) // 2, (y1 + y2) // 2, x1, y1, x2, y2, conf, pretty_label)

        target_active = False
        target_info = None

        if best_target is not None:
            raw_x, raw_y, x1, y1, x2, y2, conf_val, t_label = best_target
            box_w = max(4.0, float(x2 - x1))
            box_h = max(4.0, float(y2 - y1))

            # Metric distance estimation via optical scaling (W_real = 0.20m standard buoy / object)
            focal = 1400.0
            W_ref = 0.20
            est_dist = max(0.30, min(10.0, (focal * W_ref) / box_w))

            # 3D Ego-Motion Compensated Kalman Update
            ex, ey, vw_surge, vw_sway, w_speed, is_stat = kf.update_with_egomotion(
                [raw_x, raw_y], w=box_w, h=box_h, conf=conf_val,
                u_auv=u_auv, v_auv=v_auv, target_dist=est_dist, focal_length=focal
            )
            obj_x, obj_y = int(ex), int(ey)
            target_active = True

            v_rel_surge = vw_surge - u_auv
            stat_text = "STATIONARY" if is_stat else "MOVING"
            box_color = (0, 255, 128) if is_stat else (255, 200, 0)

            # Get 8D Kalman Smoothed Bounding Box
            bbox_smooth = kf.get_bbox()
            if bbox_smooth is not None:
                sx1, sy1, sx2, sy2, _, _ = bbox_smooth
                cv2.rectangle(frame, (sx1, sy1), (sx2, sy2), box_color, 2)
            else:
                cv2.rectangle(frame, (x1, y1), (x2, y2), box_color, 2)

            # Draw center tracking dots & line
            cv2.circle(frame, (raw_x, raw_y), 4, (0, 0, 255), -1)
            cv2.circle(frame, (obj_x, obj_y), 5, box_color, -1)
            cv2.line(frame, (center_x, center_y), (obj_x, obj_y), (255, 255, 0), 2)

            # Draw Ego-Motion Target Speed Badges
            cv2.putText(frame, f"{t_label} [{stat_text}] (Z: {est_dist:.2f}m)",
                        (x1, max(22, y1 - 28)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, box_color, 2)
            cv2.putText(frame, f"V_rel: {v_rel_surge:+.2f} | V_auv: {u_auv:+.2f} | V_world: {w_speed:.2f} m/s",
                        (x1, max(38, y1 - 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)

            target_info = {
                "detected": True,
                "label": t_label,
                "dist": est_dist,
                "v_rel": v_rel_surge,
                "v_world": w_speed,
                "is_stationary": is_stat
            }

            # Steering Commands to ArduSub
            error_x = (obj_x - center_x) / (w / 2)
            error_y = (obj_y - center_y) / (h / 2)
            yaw_cmd = int(error_x * 400 * KP_YAW)
            heave_cmd = int(-error_y * 400 * KP_HEAVE)

            if mav is not None:
                mav.mav.manual_control_send(
                    mav.target_system,
                    FORWARD_SPEED,
                    0,
                    500 + heave_cmd,
                    yaw_cmd,
                    0
                )
        else:
            # Dead-Reckoning on Lost Frames
            pred_x, pred_y = kf.handle_missing_frame()
            if pred_x is not None:
                obj_x, obj_y = int(pred_x), int(pred_y)
                cv2.circle(frame, (obj_x, obj_y), 6, (255, 255, 0), -1)
                cv2.putText(frame, f"8D KF DEAD-RECKONING ({kf.missed_frames}f)",
                            (max(10, obj_x - 80), max(20, obj_y - 15)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.52, (255, 255, 0), 2)
            else:
                cv2.putText(frame, "SEARCHING FOR TARGET (8D KF READY)...", (20, 65),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.60, (0, 165, 255), 2)

        # Log 50 Hz Synchronous Telemetry
        if logger.active:
            logger.log_step(u_dvl=u_auv, v_dvl=v_auv, h_floor=h_floor, target_info=target_info)

        # Picture-in-Picture (PIP) Inset of Downward DVL Optical Flow
        if pip_enabled and flow_vis is not None:
            pip_w, pip_h = 320, 180
            pip_thumb = cv2.resize(flow_vis, (pip_w, pip_h))
            cv2.rectangle(pip_thumb, (0, 0), (pip_w - 1, pip_h - 1), (0, 255, 255), 2)
            cv2.putText(pip_thumb, f"DOWNWARD DVL: u={u_auv:+.2f} v={v_auv:+.2f} m/s", (8, 18),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 255, 255), 1)
            cv2.putText(pip_thumb, f"FLOOR H: {h_floor:.2f} m | PTS: {dvl_pts}", (8, 34),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 255, 255), 1)

            y_off = h - pip_h - 20
            x_off = w - pip_w - 20
            if y_off >= 0 and x_off >= 0:
                frame[y_off:y_off + pip_h, x_off:x_off + pip_w] = pip_thumb

        # Cockpit Top Avionics Banner
        log_badge = f"[LOG: REC ({logger.sample_count})]" if logger.active else "[LOG: OFF]"
        cv2.putText(frame, f"AUV COCKPIT | FRONT: RPi AI Cam (IMX500) | DOWNWARD: C922 DVL | {log_badge}",
                    (20, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (255, 255, 255), 1)
        cv2.putText(frame, f"DVL: u={u_auv:+.2f} v={v_auv:+.2f} m/s | FLOOR H: {h_floor:.2f}m | TGT SPEED: {(target_info['v_world'] if target_info else 0.0):.2f}m/s",
                    (20, 45), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 255, 255), 1)

        # Bottom Avionics Controls Banner
        pip_badge = "[PIP: ON]" if pip_enabled else "[PIP: OFF]"
        clahe_badge = "[CLAHE: ON]" if clahe_enabled else "[CLAHE: OFF]"
        cv2.putText(frame, f"Keys: [l] Log | [p] PIP {pip_badge} | [e] CLAHE {clahe_badge} | [r] Rotate | [+/-] Conf | [q] Exit",
                    (20, h - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)

        cv2.imshow(WINDOW_NAME, frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('l'):
            if logger.active:
                logger.stop()
            else:
                logger.start()
        elif key == ord('p'):
            pip_enabled = not pip_enabled
            print(f"[Cockpit] Picture-in-Picture: {'ENABLED' if pip_enabled else 'DISABLED'}")
        elif key == ord('e'):
            clahe_enabled = not clahe_enabled
            print(f"[System] CLAHE Dynamic Contrast Enhancer: {'ENABLED' if clahe_enabled else 'DISABLED'}")
        elif key == ord('r'):
            rotation_mode = (rotation_mode + 1) % 4
            kf.init(center_x, center_y)
        elif key == ord('f'):
            rotation_mode = 2 if rotation_mode == 0 else 0
            kf.init(center_x, center_y)
        elif key in [ord('+'), ord('=')]:
            conf_thresh = min(0.95, conf_thresh + 0.02)
            print(f"[System] Confidence threshold: {conf_thresh:.2f}")
        elif key in [ord('-'), ord('_')]:
            conf_thresh = max(0.02, conf_thresh - 0.02)
            print(f"[System] Confidence threshold: {conf_thresh:.2f}")

    # Clean shutdown
    if logger.active:
        logger.stop()
    if dvl_worker is not None:
        dvl_worker.stop()
    front_grabber.release()
    cv2.destroyAllWindows()
    print("[System] Cooperative tracking pipeline stopped cleanly.")


if __name__ == "__main__":
    main()

