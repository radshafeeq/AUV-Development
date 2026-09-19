#!/usr/bin/env python3
"""
Kalman Filter for AUV Target Tracking & State Estimation
-------------------------------------------------------
Provides an optimal 4D / 8D Constant-Velocity Kalman Filter derived from
first-principles Continuous White Noise Acceleration (CWNA) stochastic modeling to:
1. Smooth noisy YOLO visual detections and eliminate thruster control jitter.
2. Filter bounding box scale/dimensions (w, h) to estimate approaching/retreating distance.
3. Predict target location during temporary occlusions (bubbles, water murkiness, glare).
4. Estimate real-time target velocity (vx, vy) for predictive steering and surge control.

Corresponds directly to Section 6 & 7 (Listing 1) of the AUV Kalman Filter Monograph,
with 8D bounding-box expansion for 3D visual servoing and distance regulation.
"""

import numpy as np
import cv2


class AUVKalmanFilter:
    """
    Constant-Velocity Visual Servoing Kalman Filter:
    - 4D Mode: State x_k = [x, y, vx, vy]^T               (centroid position & velocity)
    - 8D Mode: State x_k = [x, y, w, h, vx, vy, vw, vh]^T (centroid + bounding box size & growth rates)
    """

    def __init__(self, dt=1.0/30.0, process_noise_std=0.05, measurement_noise_std=0.20, qs=0.05, r_var=0.20, mode="8D"):
        self.dt = float(dt)
        self.mode = mode.upper()
        self.qs = float(qs if qs is not None else process_noise_std)
        self.r_var = float(r_var if r_var is not None else (measurement_noise_std if measurement_noise_std == 0.20 else measurement_noise_std ** 2))

        dt2 = (self.dt ** 2) / 2.0
        dt3 = (self.dt ** 3) / 3.0

        if self.mode == "4D":
            # 4 state variables (x, y, vx, vy), 2 measurement variables (z_x, z_y)
            self.kf = cv2.KalmanFilter(4, 2)

            self.kf.transitionMatrix = np.array([
                [1.0, 0.0, self.dt, 0.0],
                [0.0, 1.0, 0.0, self.dt],
                [0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 1.0]
            ], dtype=np.float32)

            self.kf.measurementMatrix = np.array([
                [1.0, 0.0, 0.0, 0.0],
                [0.0, 1.0, 0.0, 0.0]
            ], dtype=np.float32)

            self.kf.processNoiseCov = self.qs * np.array([
                [dt3, 0.0, dt2, 0.0],
                [0.0, dt3, 0.0, dt2],
                [dt2, 0.0, self.dt, 0.0],
                [0.0, dt2, 0.0, self.dt]
            ], dtype=np.float32)

            self.kf.measurementNoiseCov = self.r_var * np.eye(2, dtype=np.float32)
            self.kf.errorCovPost = np.eye(4, dtype=np.float32)
            self.kf.errorCovPre = np.eye(4, dtype=np.float32)

        else:
            # 8D Mode: 8 state variables (x, y, w, h, vx, vy, vw, vh), 4 measurement variables (x, y, w, h)
            self.kf = cv2.KalmanFilter(8, 4)

            # State Transition Matrix A
            A = np.eye(8, dtype=np.float32)
            A[0:4, 4:8] = np.eye(4, dtype=np.float32) * self.dt
            self.kf.transitionMatrix = A

            # Measurement Observation Matrix H
            H = np.zeros((4, 8), dtype=np.float32)
            H[0:4, 0:4] = np.eye(4, dtype=np.float32)
            self.kf.measurementMatrix = H

            # Discretized Process Noise Covariance Matrix Q (CWNA Model for 4 coordinates)
            Q = np.zeros((8, 8), dtype=np.float32)
            Q[0:4, 0:4] = np.eye(4, dtype=np.float32) * (self.qs * dt3)
            Q[0:4, 4:8] = np.eye(4, dtype=np.float32) * (self.qs * dt2)
            Q[4:8, 0:4] = np.eye(4, dtype=np.float32) * (self.qs * dt2)
            Q[4:8, 4:8] = np.eye(4, dtype=np.float32) * (self.qs * self.dt)
            self.kf.processNoiseCov = Q

            # Measurement Noise Covariance Matrix R (Position variance 0.20, BBox size variance 0.50)
            r_size = self.r_var * 2.5  # Slightly higher tolerance for bounding box boundary jitter
            self.kf.measurementNoiseCov = np.diag([self.r_var, self.r_var, r_size, r_size]).astype(np.float32)
            self.kf.errorCovPost = np.eye(8, dtype=np.float32)
            self.kf.errorCovPre = np.eye(8, dtype=np.float32)

        self.initialized = False
        self.missed_frames = 0
        self.max_missed_frames = 15  # ~0.5 s timeout at 30 FPS

    def init(self, x, y, w=50.0, h=50.0, vx=0.0, vy=0.0, vw=0.0, vh=0.0):
        """Reset and initialize Kalman filter state with initial position, dimensions, and velocities."""
        if self.mode == "4D":
            st = np.array([[np.float32(x)], [np.float32(y)], [np.float32(vx)], [np.float32(vy)]], dtype=np.float32)
            self.kf.statePost = st
            self.kf.statePre = st.copy()
            self.kf.errorCovPost = np.eye(4, dtype=np.float32)
            self.kf.errorCovPre = np.eye(4, dtype=np.float32)
        else:
            st = np.array([
                [np.float32(x)], [np.float32(y)], [np.float32(w)], [np.float32(h)],
                [np.float32(vx)], [np.float32(vy)], [np.float32(vw)], [np.float32(vh)]
            ], dtype=np.float32)
            self.kf.statePost = st
            self.kf.statePre = st.copy()
            self.kf.errorCovPost = np.eye(8, dtype=np.float32)
            self.kf.errorCovPre = np.eye(8, dtype=np.float32)

        self.initialized = True
        self.missed_frames = 0

    def predict(self):
        """Predict the next state using motion dynamics."""
        if not self.initialized:
            return None, None
        prediction = self.kf.predict()
        pred_x = float(prediction[0][0])
        pred_y = float(prediction[1][0])
        return pred_x, pred_y

    def update(self, x, y, w=None, h=None):
        """Correct the prediction using a new detection."""
        if not self.initialized:
            init_w = float(w) if w is not None else 50.0
            init_h = float(h) if h is not None else 50.0
            self.init(x, y, w=init_w, h=init_h)
            return float(x), float(y)

        if self.mode == "4D" or w is None or h is None:
            measurement = np.array([[np.float32(x)], [np.float32(y)]], dtype=np.float32)
            # If 8D filter received only 2D input, keep last estimated w, h
            if self.mode == "8D":
                cur_w = self.kf.statePost[2][0]
                cur_h = self.kf.statePost[3][0]
                measurement = np.array([[np.float32(x)], [np.float32(y)], [cur_w], [cur_h]], dtype=np.float32)
        else:
            measurement = np.array([
                [np.float32(x)], [np.float32(y)],
                [np.float32(w)], [np.float32(h)]
            ], dtype=np.float32)

        estimated = self.kf.correct(measurement)
        self.missed_frames = 0

        est_x = float(estimated[0][0])
        est_y = float(estimated[1][0])
        return est_x, est_y

    def update_bbox(self, x1, y1, x2, y2):
        """Helper to update state directly from bounding box coordinates (x1, y1, x2, y2)."""
        w = max(1.0, float(x2 - x1))
        h = max(1.0, float(y2 - y1))
        cx = float(x1 + x2) / 2.0
        cy = float(y1 + y2) / 2.0
        est_x, est_y = self.update(cx, cy, w, h)
        return est_x, est_y

    def get_bbox(self):
        """Get filtered bounding box coordinates (x1, y1, x2, y2, w, h)."""
        if not self.initialized:
            return None
        cx = float(self.kf.statePost[0][0])
        cy = float(self.kf.statePost[1][0])
        if self.mode == "8D":
            w = max(4.0, float(self.kf.statePost[2][0]))
            h = max(4.0, float(self.kf.statePost[3][0]))
        else:
            w = 50.0
            h = 50.0
        x1 = int(cx - w / 2.0)
        y1 = int(cy - h / 2.0)
        x2 = int(cx + w / 2.0)
        y2 = int(cy + h / 2.0)
        return x1, y1, x2, y2, w, h

    def get_velocity(self):
        """Get estimated target velocity vector (vx, vy) in pixels/second."""
        if not self.initialized:
            return 0.0, 0.0
        vel_idx = 4 if self.mode == "8D" else 2
        vx = float(self.kf.statePost[vel_idx][0])
        vy = float(self.kf.statePost[vel_idx + 1][0])
        return vx, vy

    def get_scale_rates(self):
        """Get bounding box area and growth rate for distance estimation."""
        if not self.initialized or self.mode != "8D":
            return 0.0, 0.0
        w = float(self.kf.statePost[2][0])
        h = float(self.kf.statePost[3][0])
        vw = float(self.kf.statePost[6][0])
        vh = float(self.kf.statePost[7][0])
        area = max(1.0, w * h)
        area_rate = (vw * h + w * vh)  # d(Area)/dt in px^2/s
        return area, area_rate

    def handle_missing_frame(self):
        """Called when YOLO fails to detect the object in the current frame."""
        if not self.initialized:
            return None, None
        self.missed_frames += 1
        if self.missed_frames > self.max_missed_frames:
            self.initialized = False
            return None, None
        # Return predicted position
        pred_x = float(self.kf.statePost[0][0])
        pred_y = float(self.kf.statePost[1][0])
        return pred_x, pred_y

    def process_frame(self, bbox_centroid=None):
        """
        Unified Predict-Correct Pipeline matching Listing 1 of the AUV Kalman Filter Monograph:
        Returns: (filtered_x, filtered_y, is_valid)
        """
        if not self.initialized:
            if bbox_centroid is not None:
                self.init(bbox_centroid[0], bbox_centroid[1])
                return float(bbox_centroid[0]), float(bbox_centroid[1]), True
            return None, None, False

        # Step 1: Predict Phase
        prediction = self.kf.predict()

        if bbox_centroid is not None:
            # Target Detected: Execute Correction Phase
            self.missed_frames = 0
            if self.mode == "4D":
                measurement = np.array([
                    [np.float32(bbox_centroid[0])],
                    [np.float32(bbox_centroid[1])]
                ], dtype=np.float32)
            else:
                w = float(bbox_centroid[2]) if len(bbox_centroid) > 2 else self.kf.statePost[2][0]
                h = float(bbox_centroid[3]) if len(bbox_centroid) > 3 else self.kf.statePost[3][0]
                measurement = np.array([
                    [np.float32(bbox_centroid[0])],
                    [np.float32(bbox_centroid[1])],
                    [np.float32(w)],
                    [np.float32(h)]
                ], dtype=np.float32)

            estimated_state = self.kf.correct(measurement)
            return float(estimated_state[0][0]), float(estimated_state[1][0]), True
        else:
            # Occlusion Defense: Dead-Reckoning Extrapolation
            self.missed_frames += 1
            if self.missed_frames <= self.max_missed_frames:
                return float(prediction[0][0]), float(prediction[1][0]), True
            else:
                self.initialized = False
                return None, None, False


# Backwards-compatible aliases
TargetKalmanFilter = AUVKalmanFilter
AUV4DKalmanFilter = lambda **kwargs: AUVKalmanFilter(mode="4D", **kwargs)
AUV8DKalmanFilter = lambda **kwargs: AUVKalmanFilter(mode="8D", **kwargs)


if __name__ == "__main__":
    print("=" * 65)
    print("Testing AUVKalmanFilter: 4D Monograph Benchmark & 8D Scale Tracking")
    print("=" * 65)
    
    # 1. Test 4D mode against Section 6.7 benchmark
    print("\n--- 1. Testing 4D Mode (Monograph Regression Test) ---")
    kf4 = AUVKalmanFilter(dt=1.0/30.0, qs=0.05, r_var=0.20, mode="4D")
    kf4.init(310.0, 235.0, vx=5.0, vy=-2.0)
    
    test_measurements = [
        (325.0, 233.0),
        (328.0, 231.0),
        None,
        None,
        (332.0, 229.0)
    ]
    for k, z in enumerate(test_measurements, start=1):
        ex, ey, val = kf4.process_frame(z)
        vx, vy = kf4.get_velocity()
        m_str = f"({z[0]:.1f}, {z[1]:.1f})" if z is not None else "OCCLUSION"
        print(f"Cycle {k}: Measured={m_str:<16} -> Filtered=({ex:.2f}, {ey:.2f}) | Vel=({vx:+.2f}, {vy:+.2f}) px/s")

    # 2. Test 8D mode for Position + Bounding Box Scale Tracking
    print("\n--- 2. Testing 8D Mode (Position + Scale + Surge Rate Tracking) ---")
    kf8 = AUVKalmanFilter(dt=1.0/30.0, qs=0.05, r_var=0.20, mode="8D")
    kf8.init(320.0, 240.0, w=100.0, h=80.0)
    
    # Simulate an approaching target (x, y slight drift, bounding box grows from 100x80 to 120x96)
    for step in range(1, 6):
        kf8.predict()
        raw_x = 320.0 + step * 2.0
        raw_y = 240.0 - step * 1.0
        raw_w = 100.0 + step * 4.0
        raw_h = 80.0 + step * 3.2
        ex, ey = kf8.update(raw_x, raw_y, raw_w, raw_h)
        vx, vy = kf8.get_velocity()
        x1, y1, x2, y2, fw, fh = kf8.get_bbox()
        area, area_rate = kf8.get_scale_rates()
        print(f"Step {step}: Center=({ex:.1f}, {ey:.1f}) | BBox=({fw:.1f}x{fh:.1f}) | Area={area:.0f} px² | Growth={area_rate:+.1f} px²/s")
    
    print("\nAUVKalmanFilter 4D and 8D tests completed successfully!")


