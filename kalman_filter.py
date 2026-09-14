#!/usr/bin/env python3
"""
Kalman Filter for AUV Target Tracking & State Estimation
-------------------------------------------------------
Provides a 4D Constant-Velocity Kalman Filter [x, y, vx, vy]^T to:
1. Smooth noisy YOLO visual detections and eliminate thruster control jitter.
2. Predict target location during temporary occlusions (bubbles, water murkiness, glare).
3. Estimate target velocity (vx, vy) for predictive steering.
"""

import numpy as np
import cv2


class TargetKalmanFilter:
    """
    2D Position + 2D Velocity Kalman Filter:
    State vector X = [x, y, vx, vy]^T
    Measurement Z = [z_x, z_y]^T
    """

    def __init__(self, dt=0.033, process_noise_std=0.05, measurement_noise_std=0.2):
        self.dt = dt

        # Initialize OpenCV Kalman Filter (4 state variables: x, y, vx, vy; 2 measurement variables: z_x, z_y)
        self.kf = cv2.KalmanFilter(4, 2)

        # State Transition Matrix A: x_k = x_{k-1} + vx * dt
        self.kf.transitionMatrix = np.array([
            [1, 0, self.dt, 0],
            [0, 1, 0, self.dt],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ], dtype=np.float32)

        # Measurement Matrix H: We only measure position (z_x, z_y)
        self.kf.measurementMatrix = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0]
        ], dtype=np.float32)

        # Process Noise Covariance Q (Uncertainty in constant-velocity model)
        self.kf.processNoiseCov = np.eye(4, dtype=np.float32) * (process_noise_std ** 2)

        # Measurement Noise Covariance R (YOLO detection bounding box noise)
        self.kf.measurementNoiseCov = np.eye(2, dtype=np.float32) * (measurement_noise_std ** 2)

        # Posteriori Error Covariance P
        self.kf.errorCovPost = np.eye(4, dtype=np.float32)

        self.initialized = False
        self.missed_frames = 0
        self.max_missed_frames = 15  # Keep predicting for up to 15 missing frames (~0.5 seconds)

    def init(self, x, y):
        """Reset and initialize Kalman filter state with initial position (x, y)."""
        self.kf.statePost = np.array([[x], [y], [0.0], [0.0]], dtype=np.float32)
        self.kf.errorCovPost = np.eye(4, dtype=np.float32)
        self.initialized = True
        self.missed_frames = 0

    def predict(self):
        """Predict the next state [x, y, vx, vy]^T using motion dynamics."""
        if not self.initialized:
            return None, None
        prediction = self.kf.predict()
        pred_x = float(prediction[0][0])
        pred_y = float(prediction[1][0])
        return pred_x, pred_y

    def update(self, x, y):
        """Correct the prediction using a new YOLO detection (z_x, z_y)."""
        if not self.initialized:
            self.init(x, y)
            return x, y

        measurement = np.array([[np.float32(x)], [np.float32(y)]], dtype=np.float32)
        estimated = self.kf.correct(measurement)
        self.missed_frames = 0

        est_x = float(estimated[0][0])
        est_y = float(estimated[1][0])
        return est_x, est_y

    def get_velocity(self):
        """Get estimated target velocity vector (vx, vy)."""
        if not self.initialized:
            return 0.0, 0.0
        vx = float(self.kf.statePost[2][0])
        vy = float(self.kf.statePost[3][0])
        return vx, vy

    def handle_missing_frame(self):
        """Called when YOLO fails to detect the object in the current frame."""
        self.missed_frames += 1
        if self.missed_frames > self.max_missed_frames:
            self.initialized = False
            return None, None
        # Return predicted position
        pred_x = float(self.kf.statePost[0][0])
        pred_y = float(self.kf.statePost[1][0])
        return pred_x, pred_y


if __name__ == "__main__":
    print("Testing TargetKalmanFilter module...")
    kf = TargetKalmanFilter(dt=0.033)
    kf.init(320, 240)
    for i in range(5):
        px, py = kf.predict()
        # Simulate noisy measurements moving right
        mx, my = 320 + i * 10 + np.random.normal(0, 2), 240 + np.random.normal(0, 2)
        ex, ey = kf.update(mx, my)
        vx, vy = kf.get_velocity()
        print(f"Step {i+1}: Measured=({mx:.1f}, {my:.1f}) -> Filtered=({ex:.1f}, {ey:.1f}) | Velocity=({vx:.2f}, {vy:.2f})")
    print("Kalman Filter test complete!")
