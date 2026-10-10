#!/usr/bin/env python3
"""
AUV Downward Visual DVL & AR Planar Floor Distance Engine
=========================================================
Author: Radhi Shafeeq
Affiliation: Hasanuddin University (Mechatronics Engineering)
Project: Over-Actuated 6-DOF, 8-Motor Autonomous Underwater Vehicle (AUV)

Functionality:
--------------
1. Captures downward-facing video from the lower acrylic hull:
   - Live network stream via BlueOS UVC (UDP port 5601), OR
   - Direct USB connection on topside bench test (e.g. /dev/video2).
2. Performs Pyramidal Lucas-Kanade optical flow on Shi-Tomasi feature points
   with CLAHE underwater contrast enhancement.
3. Derotates optical flow using Pixhawk gyroscope rates (p, q, r) to isolate
   pure linear translation from vehicle tilt and angular rotation.
4. Executes the AR (Augmented Reality) Planar Distance Engine:
   Fuses optical scale expansion/disparity, Pixhawk IMU gravity vector, and
   Bar30 hydrostatic depth (delta_z) to recover true metric floor distance (h).
5. Computes metric linear surge and sway velocities of the vehicle:
   u_AUV = -(dx_trans * h) / fx
   v_AUV = -(dy_trans * h) / fy  [m/s]
6. Compensates for optical refraction through the lower acrylic tube and water
   (f_water = 1.33 * f_air).
"""

import cv2
import numpy as np
import time
import math
import threading

class DownwardVisualDVL:
    """
    High-Performance Visual Doppler Velocity Log (Visual DVL) & AR Distance Estimator
    tailored for the AUV lower acrylic pod.
    """
    def __init__(self, port=5601, device_index=2, width=1920, height=1080,
                 underwater_mode=True, nominal_floor_dist=1.50):
        self.width = int(width)
        self.height = int(height)
        self.port = int(port)
        self.device_index = int(device_index)
        self.underwater_mode = bool(underwater_mode)
        
        # Optical calibration constants (Logitech C922 / Sonix 1080p sensor)
        # In air at 1080p: fx ~= 1400.0, fy ~= 1400.0, cx ~= 960.0, cy ~= 540.0
        self.refraction_index = 1.33 if self.underwater_mode else 1.00
        self.fx_air = 1400.0
        self.fy_air = 1400.0
        self.fx = self.fx_air * self.refraction_index
        self.fy = self.fy_air * self.refraction_index
        self.cx = self.width / 2.0
        self.cy = self.height / 2.0

        # Contrast Limited Adaptive Histogram Equalization (CLAHE) for murky water
        self.clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))

        # Optical flow configuration
        self.max_corners = 120
        self.feature_params = dict(
            maxCorners=self.max_corners,
            qualityLevel=0.03,
            minDistance=15,
            blockSize=7
        )
        self.lk_params = dict(
            winSize=(25, 25),
            maxLevel=3,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01)
        )

        # Vehicle & Floor states
        self.h_floor = float(nominal_floor_dist)
        self.u_auv = 0.0       # Surge velocity (forward, m/s)
        self.v_auv = 0.0       # Sway velocity (lateral, m/s)
        self.flow_quality = 0.0 # 0.0 to 1.0 confidence score
        self.tracked_count = 0
        
        # Historical tracking buffers
        self.prev_gray = None
        self.prev_pts = None
        self.last_depth = None
        self.last_time = time.time()
        self.feature_refresh_interval = 1.0 # seconds
        self.last_feature_refresh = time.time()
        
        # Thread safety & video capture
        self.lock = threading.Lock()
        self.running = False
        self.cap = None
        self.latest_frame = None
        self.latest_flow_vis = None

    def start_stream(self, use_udp=False):
        """Starts video capture either from BlueOS UDP port 5601 or local device."""
        if use_udp:
            # GStreamer pipeline for UDP H.264 / MJPG from BlueOS
            pipeline = (
                f"udpsrc port={self.port} caps=\"application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264\" ! "
                f"rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! appsink"
            )
            self.cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
            if not self.cap.isOpened():
                # Fallback to MJPG UDP
                pipeline_mjpg = f"udpsrc port={self.port} ! application/x-rtp,encoding-name=JPEG ! rtpjpegdepay ! jpegdec ! videoconvert ! appsink"
                self.cap = cv2.VideoCapture(pipeline_mjpg, cv2.CAP_GSTREAMER)
        
        if self.cap is None or not self.cap.isOpened():
            # Local USB fallback (/dev/video2)
            self.cap = cv2.VideoCapture(self.device_index, cv2.CAP_V4L2)
            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            self.cap.set(cv2.CAP_PROP_FPS, 30)

        if not self.cap.isOpened():
            return False

        self.running = True
        return True

    def process_frame(self, frame_bgr, p=0.0, q=0.0, r=0.0, bar30_depth=None, dt=None):
        """
        Process a single downward camera frame:
        - Computes Lucas-Kanade optical flow
        - Derotates using gyro rates (p, q, r in rad/s)
        - Updates AR floor distance (h)
        - Calculates metric linear velocities (u_auv, v_auv in m/s)
        """
        curr_time = time.time()
        actual_dt = float(dt) if (dt is not None and dt > 0) else max(0.001, curr_time - self.last_time)
        self.last_time = curr_time

        # Convert to grayscale & apply CLAHE contrast enhancement
        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
        gray = self.clahe.apply(gray)
        flow_vis = frame_bgr.copy()

        # Step 1: Feature detection / refreshing
        re_detect = (
            self.prev_pts is None or 
            len(self.prev_pts) < 20 or 
            (curr_time - self.last_feature_refresh) > self.feature_refresh_interval
        )
        if re_detect:
            pts = cv2.goodFeaturesToTrack(gray, mask=None, **self.feature_params)
            if pts is not None and len(pts) > 0:
                self.prev_pts = pts
                self.last_feature_refresh = curr_time

        if self.prev_gray is None or self.prev_pts is None or len(self.prev_pts) == 0:
            self.prev_gray = gray.copy()
            return self.u_auv, self.v_auv, self.h_floor, flow_vis

        # Step 2: Pyramidal Lucas-Kanade optical flow
        curr_pts, status, err = cv2.calcOpticalFlowPyrLK(
            self.prev_gray, gray, self.prev_pts, None, **self.lk_params
        )

        good_curr = []
        good_prev = []
        if curr_pts is not None and status is not None:
            for i, st in enumerate(status.flatten()):
                if st == 1:
                    good_curr.append(curr_pts[i].ravel())
                    good_prev.append(self.prev_pts[i].ravel())

        good_curr = np.array(good_curr, dtype=np.float32)
        good_prev = np.array(good_prev, dtype=np.float32)
        self.tracked_count = len(good_curr)

        if self.tracked_count >= 8:
            # Optical displacement per point (pixels)
            flow_vectors = (good_curr - good_prev) # [dx, dy]
            
            # Step 3: Gyroscope Derotation
            # For a camera pointing along +Z body (downward):
            # Rotational pixel displacement:
            # dx_rot = (x*y/fy)*p - (fx + x^2/fx)*q + y*r
            # dy_rot = (fy + y^2/fy)*p - (x*y/fx)*q - x*r
            trans_flow_x = []
            trans_flow_y = []
            
            for (p_old, p_new) in zip(good_prev, good_curr):
                x = p_old[0] - self.cx
                y = p_old[1] - self.cy
                
                dx_meas = (p_new[0] - p_old[0]) / actual_dt
                dy_meas = (p_new[1] - p_old[1]) / actual_dt
                
                # Derotation correction (pixels/second)
                dx_rot = ((x * y / self.fy) * p - (self.fx + (x**2 / self.fx)) * q + y * r)
                dy_rot = (((self.fy + (y**2 / self.fy)) * p) - (x * y / self.fx) * q - x * r)
                
                # Pure translational optical flow
                dx_trans = dx_meas - dx_rot
                dy_trans = dy_meas - dy_rot
                
                trans_flow_x.append(dx_trans)
                trans_flow_y.append(dy_trans)
                
                # Draw visual flow vector on HUD (green = translation)
                pt1 = (int(p_old[0]), int(p_old[1]))
                pt2 = (int(p_old[0] + dx_trans * 0.1), int(p_old[1] + dy_trans * 0.1))
                cv2.circle(flow_vis, pt1, 3, (0, 255, 0), -1)
                cv2.arrowedLine(flow_vis, pt1, pt2, (0, 255, 255), 1, tipLength=0.3)

            # Robust median translational flow
            med_flow_x = float(np.median(trans_flow_x))
            med_flow_y = float(np.median(trans_flow_y))
            self.flow_quality = min(1.0, self.tracked_count / 50.0)

            # Step 4: AR Planar Floor Distance Engine
            if bar30_depth is not None and self.last_depth is not None:
                delta_z = abs(bar30_depth - self.last_depth)
                if delta_z > 0.05: # Significant vertical motion
                    # Compute feature span ratio (scale expansion s2 / s1)
                    center_prev = np.mean(good_prev, axis=0)
                    center_curr = np.mean(good_curr, axis=0)
                    dist_prev = np.mean(np.linalg.norm(good_prev - center_prev, axis=1))
                    dist_curr = np.mean(np.linalg.norm(good_curr - center_curr, axis=1))
                    
                    if dist_prev > 10.0 and dist_curr > 10.0:
                        scale_ratio = dist_curr / dist_prev
                        if abs(scale_ratio - 1.0) > 0.02:
                            # AR Height Triangulation formula: h = delta_z * (s2 / (s2 - s1))
                            h_est = delta_z * (scale_ratio / abs(scale_ratio - 1.0))
                            if 0.30 <= h_est <= 10.0:
                                self.h_floor = 0.85 * self.h_floor + 0.15 * h_est

            if bar30_depth is not None:
                self.last_depth = float(bar30_depth)

            # Step 5: Metric Linear Velocity Conversion
            # u = surge (forward, along camera -Y or +X depending on frame alignment)
            # In standard body coordinate frame (X forward, Y starboard, Z downward):
            # Downward camera: Camera X = Body Y (Starboard), Camera Y = Body X (Forward)
            # u_AUV = -(flow_y * h) / fy
            # v_AUV = -(flow_x * h) / fx
            self.u_auv = - (med_flow_y * self.h_floor) / self.fy
            self.v_auv = - (med_flow_x * self.h_floor) / self.fx
            
            # Prepare next iteration
            self.prev_pts = good_curr.reshape(-1, 1, 2)
        else:
            # Low feature count fallback (smooth decay)
            self.u_auv *= 0.90
            self.v_auv *= 0.90
            self.flow_quality = 0.0
            self.prev_pts = None

        self.prev_gray = gray.copy()
        with self.lock:
            self.latest_flow_vis = flow_vis

        return self.u_auv, self.v_auv, self.h_floor, flow_vis

    def get_state(self):
        """Thread-safe accessor for vehicle velocity and floor distance."""
        return {
            "u_auv": float(self.u_auv),
            "v_auv": float(self.v_auv),
            "h_floor": float(self.h_floor),
            "flow_quality": float(self.flow_quality),
            "tracked_count": int(self.tracked_count),
            "underwater_mode": self.underwater_mode
        }

    def start_test(self, duration_sec=3):
        """Quick self-test to verify camera stream and pipeline health."""
        if not self.start_stream(use_udp=False):
            return False
        t_start = time.time()
        frames_processed = 0
        while time.time() - t_start < duration_sec:
            ret, frame = self.cap.read()
            if ret and frame is not None:
                self.process_frame(frame)
                frames_processed += 1
            time.sleep(0.03)
        self.stop()
        return frames_processed > 10

    def stop(self):
        """Release camera and resources."""
        self.running = False
        if self.cap is not None:
            self.cap.release()
            self.cap = None

if __name__ == "__main__":
    print("Testing DownwardVisualDVL...")
    dvl = DownwardVisualDVL(device_index=2, underwater_mode=False)
    if dvl.start_test(duration_sec=3):
        print("DownwardVisualDVL test PASSED! State:", dvl.get_state())
    else:
        print("DownwardVisualDVL test FAILED: Could not grab frames from device index 2.")
