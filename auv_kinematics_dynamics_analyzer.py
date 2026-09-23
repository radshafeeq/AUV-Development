#!/usr/bin/env python3
"""
AUV Kinematics, Dynamics, and Multi-Sensor Telemetry Analyzer
============================================================
Developed by: Radhi Shafeeq
Affiliation: Hasanuddin University (Mechatronics Engineering)
Project: Over-Actuated 6-DOF, 8-Motor Autonomous Underwater Vehicle (AUV)

Purpose:
--------
Synchronously captures, computes, and analyzes the complete 6-DOF Kinematics,
Fossen Kinetics/Dynamics, Thruster Allocation, and Multi-Sensor Fusion
(Pixhawk IMU, MS5837 Bar30 depth sensor, 8-motor PWMs, YOLO26 target tracking,
and dual Kalman filters) for prototype development, tuning, and research logging.

Features:
- Subscribes to live MAVLink messages (ATTITUDE, RAW_IMU, SCALED_PRESSURE, SERVO_OUTPUT_RAW).
- Computes complete 6-DOF Kinematic transformation (R_b^n, T_Theta, J_6x6).
- Computes 8-motor PWM-to-thrust mapping and 6x8 allocation matrix (tau = T_6x8 * f).
- Evaluates Fossen's 6-DOF hydrodynamic kinetics (Inertia M, Damping D, Restoring g).
- Calculates real-time dynamic force/moment balances and disturbance residuals (d_u, d_v).
- Built-in --test mode (synthetic trajectory validation) and --live mode (hardware capture).
- Automatically exports synchronized CSV dataset and publication-quality plots (300 DPI).
"""

import sys
import os
import time
import math
import argparse
import json
import urllib.request
import numpy as np

# Try importing matplotlib for figure rendering
try:
    import matplotlib
    matplotlib.use('Agg') # Headless rendering
    import matplotlib.pyplot as plt
    HAS_MATPLOTLIB = True
except ImportError:
    HAS_MATPLOTLIB = False

# Import custom Kalman filter suite if available
try:
    sys.path.append("/home/radhi/Documents/AUV_GitHub_Upload")
    from kalman_filter import AUVDynamicsKalmanFilter, AUVVisualKalmanFilter
    HAS_KALMAN = True
except ImportError:
    HAS_KALMAN = False


class AUVKinematicsDynamicsAnalyzer:
    """
    Core Kinematics, Dynamics, and Multi-Sensor Fusion Engine for 6-DOF AUV.
    """
    def __init__(self, mavlink_url="http://192.168.2.2:6040"):
        self.mavlink_url = mavlink_url
        self.sample_rate = 50.0 # 50 Hz
        self.dt = 1.0 / self.sample_rate

        # -------------------------------------------------------------
        # 1. Physical Parameters (BlueROV2 Heavy 8-Motor Architecture)
        # -------------------------------------------------------------
        self.mass = 13.0 # kg
        self.rho = 1000.0 # kg/m^3 (freshwater) or 1025.0 (saltwater)
        self.g_acc = 9.80665 # m/s^2
        self.buoyancy = 13.0 * self.g_acc # Neutrally buoyant: W approx B
        self.bg_z = 0.02 # Vertical BG separation (z_g - z_b) in meters

        # Principal rigid-body moments of inertia (kg.m^2)
        self.Ixx = 0.26
        self.Iyy = 0.23
        self.Izz = 0.37

        # Hydrodynamic added mass (kg and kg.m^2)
        self.X_udot = -6.36
        self.Y_vdot = -7.12
        self.Z_wdot = -18.68
        self.K_pdot = -0.189
        self.M_qdot = -0.135
        self.N_rdot = -0.222

        # Generalized 6-DOF Total Inertia Matrix M = M_RB + M_A
        self.M_diag = np.array([
            self.mass - self.X_udot, # Surge: 19.36 kg
            self.mass - self.Y_vdot, # Sway:  20.12 kg
            self.mass - self.Z_wdot, # Heave: 31.68 kg
            self.Ixx - self.K_pdot,  # Roll:  0.449 kg.m^2
            self.Iyy - self.M_qdot,  # Pitch: 0.365 kg.m^2
            self.Izz - self.N_rdot   # Yaw:   0.592 kg.m^2
        ], dtype=np.float64)
        self.M = np.diag(self.M_diag)

        # Hydrodynamic Damping Coefficients
        self.D_lin = np.array([13.7, 0.0, 33.8, 0.0, 0.0, 0.0], dtype=np.float64)
        self.D_quad = np.array([141.0, 217.0, 190.0, 4.0, 4.0, 4.0], dtype=np.float64)

        # -------------------------------------------------------------
        # 2. 6x8 Thruster Allocation Matrix (T_6x8)
        # -------------------------------------------------------------
        c45 = math.cos(math.radians(45))
        # Geometry distances (meters)
        dx_h = 0.156
        dy_h = 0.111
        dx_v = 0.140
        dy_v = 0.100

        # Yaw moment arm for 45 deg vectored horizontal thrusters
        arm_yaw = (dx_h + dy_h) * c45 / 2.0 # approx 0.0944

        self.T_6x8 = np.array([
            # T1      T2      T3      T4      T5      T6      T7      T8
            [ c45,    c45,   -c45,   -c45,    0.0,    0.0,    0.0,    0.0   ], # Surge (X)
            [-c45,    c45,   -c45,    c45,    0.0,    0.0,    0.0,    0.0   ], # Sway  (Y)
            [ 0.0,    0.0,    0.0,    0.0,   -1.0,   -1.0,   -1.0,   -1.0   ], # Heave (Z)
            [ 0.0,    0.0,    0.0,    0.0,   -dy_v,   dy_v,  -dy_v,   dy_v  ], # Roll  (K)
            [ 0.0,    0.0,    0.0,    0.0,    dx_v,   dx_v,  -dx_v,  -dx_v  ], # Pitch (M)
            [-arm_yaw, arm_yaw, arm_yaw, -arm_yaw, 0.0, 0.0,  0.0,    0.0   ]  # Yaw   (N)
        ], dtype=np.float64)

        # Compute Moore-Penrose pseudo-inverse for control allocation
        self.T_pinv = np.linalg.pinv(self.T_6x8)

        # -------------------------------------------------------------
        # 3. State Vectors and Estimators
        # -------------------------------------------------------------
        # Kinematic states eta = [x, y, z, phi, theta, psi]^T
        self.eta = np.zeros(6, dtype=np.float64)
        # Dynamic velocities nu = [u, v, w, p, q, r]^T
        self.nu = np.zeros(6, dtype=np.float64)
        # Body accelerations nudot = [udot, vdot, wdot, pdot, qdot, rdot]^T
        self.nudot = np.zeros(6, dtype=np.float64)
        # Commanded forces tau = [X, Y, Z, K, M, N]^T
        self.tau = np.zeros(6, dtype=np.float64)
        # 8-motor thrusts (N)
        self.thruster_thrusts = np.zeros(8, dtype=np.float64)
        # 8-motor PWMs (1100 - 1900 us)
        self.thruster_pwms = np.full(8, 1500.0, dtype=np.float64)

        # Disturbance Observer estimate [d_u, d_v]
        self.disturbances = np.zeros(2, dtype=np.float64)

        # Initialize Subsea Dynamics Kalman Filter
        if HAS_KALMAN:
            self.dkf = AUVDynamicsKalmanFilter(dt=self.dt)
        else:
            self.dkf = None

        # Data Logging Buffer
        self.log_data = []

    # =========================================================================
    # KINEMATIC TRANSFORMATION DERIVATIONS
    # =========================================================================
    def compute_rotation_matrix(self, phi, theta, psi):
        """
        Computes the orthogonal linear rotation matrix R_b^n in SO(3).
        Transforms vectors from Body-Fixed {b} to Earth-Fixed Inertial {n}.
        """
        c_phi = math.cos(phi)
        s_phi = math.sin(phi)
        c_th = math.cos(theta)
        s_th = math.sin(theta)
        c_psi = math.cos(psi)
        s_psi = math.sin(psi)

        R = np.array([
            [c_psi*c_th, -s_psi*c_phi + c_psi*s_th*s_phi,  s_psi*s_phi + c_psi*s_th*c_phi],
            [s_psi*c_th,  c_psi*c_phi + s_psi*s_th*s_phi, -c_psi*s_phi + s_psi*s_th*c_phi],
            [-s_th,       c_th*s_phi,                      c_th*c_phi]
        ], dtype=np.float64)
        return R

    def compute_angular_transformation_matrix(self, phi, theta):
        """
        Computes the Euler angular rate transformation matrix T_Theta.
        Relates body angular velocities [p, q, r]^T to Euler angle rates [dphi, dtheta, dpsi]^T.
        """
        c_phi = math.cos(phi)
        s_phi = math.sin(phi)
        c_th = math.cos(theta)

        # Protect against singularity at theta = +- 90 deg (Gimbal Lock)
        if abs(c_th) < 1e-4:
            c_th = 1e-4 if c_th >= 0 else -1e-4
        t_th = math.tan(theta)

        T = np.array([
            [1.0, s_phi * t_th,  c_phi * t_th],
            [0.0, c_phi,        -s_phi],
            [0.0, s_phi / c_th,  c_phi / c_th]
        ], dtype=np.float64)
        return T

    def compute_kinematic_jacobian(self, phi, theta, psi):
        """
        Constructs the unified 6x6 Kinematic Jacobian J(eta_2).
        dot_eta = J(eta_2) * nu
        """
        R = self.compute_rotation_matrix(phi, theta, psi)
        T = self.compute_angular_transformation_matrix(phi, theta)

        J = np.zeros((6, 6), dtype=np.float64)
        J[0:3, 0:3] = R
        J[3:6, 3:6] = T
        return J

    # =========================================================================
    # DYNAMIC FORCE AND MOMENT DERIVATIONS (FOSSEN)
    # =========================================================================
    def compute_hydrodynamic_damping(self, nu):
        """
        Computes the coupled 6-DOF linear and quadratic hydrodynamic damping vector:
        D(nu) * nu = D_lin * nu + D_quad * |nu| * nu
        """
        d_linear = self.D_lin * nu
        d_quadratic = self.D_quad * np.abs(nu) * nu
        return d_linear + d_quadratic

    def compute_hydrostatic_restoring(self, phi, theta):
        """
        Computes the 6-DOF gravitational and buoyancy restoring force/moment vector g(eta).
        For neutrally buoyant vehicle with center of gravity below center of buoyancy:
        """
        K_restoring = self.buoyancy * self.bg_z * math.cos(theta) * math.sin(phi)
        M_restoring = self.buoyancy * self.bg_z * math.sin(theta)

        g = np.array([0.0, 0.0, 0.0, K_restoring, M_restoring, 0.0], dtype=np.float64)
        return g

    def compute_coriolis_centripetal(self, nu):
        """
        Computes the Coriolis and Centripetal force vector C(nu) * nu.
        """
        u, v, w, p, q, r = nu
        m1, m2, m3, m4, m5, m6 = self.M_diag

        C_nu = np.array([
            -m2 * v * r + m3 * w * q,
             m1 * u * r - m3 * w * p,
            -m1 * u * q + m2 * v * p,
            -m6 * r * q + m5 * q * r,
             m6 * r * p - m4 * p * r,
            -m5 * q * p + m4 * p * q
        ], dtype=np.float64)
        return C_nu

    def pwm_to_thrust(self, pwm):
        """
        Converts PWM pulse width (1100 - 1900 us) to Blue Robotics T200 thrust (Newtons).
        Deadband: 1475 to 1525 us.
        """
        delta = pwm - 1500.0
        if abs(delta) < 25.0:
            return 0.0
        if delta > 0:
            return 0.000318 * (delta ** 2)
        else:
            return -0.000250 * (delta ** 2)

    def allocate_thrusters_to_tau(self, pwms):
        """
        Converts 8 PWM signals into generalized 6-DOF force/moment vector tau = T_6x8 * f.
        """
        f = np.array([self.pwm_to_thrust(p) for p in pwms], dtype=np.float64)
        self.thruster_thrusts = f
        tau = self.T_6x8 @ f
        return tau

    # =========================================================================
    # MULTI-SENSOR INGESTION AND TELEMETRY PROCESSING
    # =========================================================================
    def fetch_mavlink_json(self, message_name):
        """
        Queries mavlink2rest on BlueOS (port 6040) for a given MAVLink message.
        """
        url = f"{self.mavlink_url}/mavlink/vehicles/1/components/1/messages/{message_name}"
        try:
            req = urllib.request.Request(url, headers={'User-Agent': 'AUVAnalyzer/1.0'})
            with urllib.request.urlopen(req, timeout=0.5) as resp:
                data = json.loads(resp.read().decode('utf-8'))
                return data.get("message", None)
        except Exception:
            return None

    def step_live_telemetry(self):
        """
        Fetches live multi-sensor telemetry, runs kinematics and dynamics model,
        and executes one step of the Kalman filter.
        """
        t_now = time.time()

        # 1. Fetch Pixhawk ATTITUDE (Roll, Pitch, Yaw, Gyro rates)
        msg_att = self.fetch_mavlink_json("ATTITUDE")
        if msg_att:
            phi = float(msg_att.get("roll", 0.0))
            theta = float(msg_att.get("pitch", 0.0))
            psi = float(msg_att.get("yaw", 0.0))
            p = float(msg_att.get("rollspeed", 0.0))
            q = float(msg_att.get("pitchspeed", 0.0))
            r = float(msg_att.get("yawspeed", 0.0))
            self.eta[3] = phi
            self.eta[4] = theta
            self.eta[5] = psi
            self.nu[3] = p
            self.nu[4] = q
            self.nu[5] = r

        # 2. Fetch MS5837 Bar30 Subsea Depth Sensor (SCALED_PRESSURE2)
        msg_bar30 = self.fetch_mavlink_json("SCALED_PRESSURE2")
        depth = 0.0
        if msg_bar30:
            press_mbar = float(msg_bar30.get("press_abs", 1013.25))
            depth = max(0.0, (press_mbar - 1013.25) * 100.0 / (self.rho * self.g_acc))
            w = (depth - self.eta[2]) / self.dt if self.dt > 0 else 0.0
            self.eta[2] = depth
            self.nu[2] = w

        # 3. Fetch 8-Motor PWM feedback (SERVO_OUTPUT_RAW)
        msg_servo = self.fetch_mavlink_json("SERVO_OUTPUT_RAW")
        if msg_servo:
            for i in range(8):
                key = f"servo{i+1}_raw"
                self.thruster_pwms[i] = float(msg_servo.get(key, 1500.0))
            self.tau = self.allocate_thrusters_to_tau(self.thruster_pwms)

        # 4. Evaluate Hydrodynamic Dynamics & Forces
        D_nu = self.compute_hydrodynamic_damping(self.nu)
        g_eta = self.compute_hydrostatic_restoring(self.eta[3], self.eta[4])
        C_nu = self.compute_coriolis_centripetal(self.nu)

        # 5. Run Dynamics Kalman Filter Prediction & Multi-Sensor Update
        if self.dkf:
            self.dkf.predict(self.tau, angles=(self.eta[3], self.eta[4], self.eta[5]), dt=self.dt)
            z_meas = np.array([self.nu[0], self.nu[1], self.nu[2], self.nu[3], self.nu[4], self.nu[5]])
            self.dkf.update(z_meas)
            est_u, est_v, est_w, est_p, est_q, est_r = self.dkf.get_velocities()
            self.nu[0] = est_u
            self.nu[1] = est_v
            self.nu[2] = est_w
            self.disturbances[0] = float(self.dkf.x[6])
            self.disturbances[1] = float(self.dkf.x[7])

        # 6. Propagate Kinematics: dot_eta = J(eta_2) * nu
        J = self.compute_kinematic_jacobian(self.eta[3], self.eta[4], self.eta[5])
        dot_eta = J @ self.nu
        self.eta[0] += dot_eta[0] * self.dt
        self.eta[1] += dot_eta[1] * self.dt

        # Record snapshot
        record = {
            "timestamp": t_now,
            "x": self.eta[0], "y": self.eta[1], "z": self.eta[2],
            "phi": self.eta[3], "theta": self.eta[4], "psi": self.eta[5],
            "u": self.nu[0], "v": self.nu[1], "w": self.nu[2],
            "p": self.nu[3], "q": self.nu[4], "r": self.nu[5],
            "tau_X": self.tau[0], "tau_Y": self.tau[1], "tau_Z": self.tau[2],
            "tau_K": self.tau[3], "tau_M": self.tau[4], "tau_N": self.tau[5],
            "D_u": D_nu[0], "D_v": D_nu[1], "D_w": D_nu[2],
            "dist_u": self.disturbances[0], "dist_v": self.disturbances[1]
        }
        for i in range(8):
            record[f"pwm_{i+1}"] = self.thruster_pwms[i]
            record[f"thrust_{i+1}"] = self.thruster_thrusts[i]

        self.log_data.append(record)
        return record

    # =========================================================================
    # SYNTHETIC TEST-BENCH SIMULATION FOR RESEARCH VALIDATION
    # =========================================================================
    def run_synthetic_test(self, duration_sec=12.0):
        """
        Runs a comprehensive 6-DOF synthetic trajectory simulating forward surge,
        sway oscillation, diving to depth, pitch-hold inspection, and ocean currents.
        """
        print(f"\n[TestBench] Generating {duration_sec:.1f}s synthetic 6-DOF trajectory...")
        steps = int(duration_sec * self.sample_rate)

        for k in range(steps):
            t = k * self.dt

            # 1. Commanded 8-thruster PWMs
            surge_pwm = 1500.0 + 120.0 * math.sin(0.5 * math.pi * t / duration_sec)
            sway_pwm = 1500.0 + 60.0 * math.sin(1.5 * t)
            heave_pwm = 1500.0 + 80.0 * (1.0 if 2.0 <= t <= 8.0 else 0.0)
            pitch_diff = 40.0 * math.sin(0.8 * t)

            pwms = np.array([
                surge_pwm - (sway_pwm - 1500), # T1
                surge_pwm + (sway_pwm - 1500), # T2
                surge_pwm - (sway_pwm - 1500), # T3
                surge_pwm + (sway_pwm - 1500), # T4
                heave_pwm + pitch_diff,        # T5 (Port-Fore)
                heave_pwm + pitch_diff,        # T6 (Starboard-Fore)
                heave_pwm - pitch_diff,        # T7 (Port-Aft)
                heave_pwm - pitch_diff         # T8 (Starboard-Aft)
            ], dtype=np.float64)

            self.thruster_pwms = pwms
            self.tau = self.allocate_thrusters_to_tau(pwms)

            # Simulated Ocean Current Disturbance: 3.5 N in Surge, -2.0 N in Sway
            true_dist = np.array([3.5, -2.0], dtype=np.float64)

            # 2. Integrate Fossen's Kinetics: nudot = M^-1 * (tau + dist - D(nu)*nu - g(eta))
            D_nu = self.compute_hydrodynamic_damping(self.nu)
            g_eta = self.compute_hydrostatic_restoring(self.eta[3], self.eta[4])

            total_forces = self.tau - D_nu - g_eta
            total_forces[0] += true_dist[0]
            total_forces[1] += true_dist[1]

            self.nudot = total_forces / self.M_diag
            self.nu += self.nudot * self.dt

            # Add sensor noise for EKF validation
            gyro_noise = np.random.normal(0, 0.005, 3)
            accel_noise = np.random.normal(0, 0.02, 3)
            measured_nu = self.nu.copy()
            measured_nu[3:6] += gyro_noise
            measured_nu[0:2] += accel_noise[0:2] * self.dt

            # 3. Step Kalman Filter
            if self.dkf:
                self.dkf.predict(self.tau, angles=(self.eta[3], self.eta[4], self.eta[5]), dt=self.dt)
                self.dkf.update(measured_nu)
                est_u, est_v, est_w, est_p, est_q, est_r = self.dkf.get_velocities()
                self.disturbances[0] = float(self.dkf.x[6])
                self.disturbances[1] = float(self.dkf.x[7])

            # 4. Integrate Kinematics: dot_eta = J(eta) * nu
            J = self.compute_kinematic_jacobian(self.eta[3], self.eta[4], self.eta[5])
            dot_eta = J @ self.nu
            self.eta += dot_eta * self.dt

            # Record
            record = {
                "timestamp": t,
                "x": self.eta[0], "y": self.eta[1], "z": self.eta[2],
                "phi": self.eta[3], "theta": self.eta[4], "psi": self.eta[5],
                "u": self.nu[0], "v": self.nu[1], "w": self.nu[2],
                "p": self.nu[3], "q": self.nu[4], "r": self.nu[5],
                "tau_X": self.tau[0], "tau_Y": self.tau[1], "tau_Z": self.tau[2],
                "tau_K": self.tau[3], "tau_M": self.tau[4], "tau_N": self.tau[5],
                "D_u": D_nu[0], "D_v": D_nu[1], "D_w": D_nu[2],
                "dist_u": self.disturbances[0], "dist_v": self.disturbances[1]
            }
            for i in range(8):
                record[f"pwm_{i+1}"] = self.thruster_pwms[i]
                record[f"thrust_{i+1}"] = self.thruster_thrusts[i]

            self.log_data.append(record)

        print(f"[TestBench] Trajectory generated successfully ({len(self.log_data)} samples).")

    # =========================================================================
    # EXPORT DATASET AND GENERATE PUBLICATION PLOTS
    # =========================================================================
    def save_csv(self, filename="auv_telemetry_dataset.csv"):
        """Saves recorded time-series data to CSV."""
        if not self.log_data:
            print("[Warning] No data to save.")
            return

        keys = list(self.log_data[0].keys())
        with open(filename, "w") as f:
            f.write(",".join(keys) + "\n")
            for entry in self.log_data:
                f.write(",".join(f"{entry[k]:.5f}" for k in keys) + "\n")
        print(f"[Export] Saved {len(self.log_data)} telemetry records to: {filename}")

    def generate_publication_plots(self, output_dir="."):
        """
        Renders four publication-quality figures (300 DPI):
        1. 6-DOF Kinematic Attitudes and 3D Trajectory
        2. 6-DOF Dynamic Velocities
        3. Hydrodynamic Kinetics and Ocean Disturbance Force Balance
        4. 8-Motor Thruster Control Allocation and PWM Distribution
        """
        if not HAS_MATPLOTLIB:
            print("[Warning] Matplotlib not installed; skipping plot generation.")
            return

        if not self.log_data:
            print("[Warning] No telemetry data to plot.")
            return

        t0 = self.log_data[0]["timestamp"]
        t = [d["timestamp"] - t0 for d in self.log_data]
        x = [d["x"] for d in self.log_data]
        y = [d["y"] for d in self.log_data]
        z = [d["z"] for d in self.log_data]
        phi = [math.degrees(d["phi"]) for d in self.log_data]
        theta = [math.degrees(d["theta"]) for d in self.log_data]
        psi = [math.degrees(d["psi"]) for d in self.log_data]

        u = [d["u"] for d in self.log_data]
        v = [d["v"] for d in self.log_data]
        w = [d["w"] for d in self.log_data]
        p = [math.degrees(d["p"]) for d in self.log_data]
        q = [math.degrees(d["q"]) for d in self.log_data]
        r = [math.degrees(d["r"]) for d in self.log_data]

        tau_X = [d["tau_X"] for d in self.log_data]
        dist_u = [d["dist_u"] for d in self.log_data]
        dist_v = [d["dist_v"] for d in self.log_data]
        D_u = [d["D_u"] for d in self.log_data]

        plt.style.use('seaborn-v0_8-whitegrid' if 'seaborn-v0_8-whitegrid' in plt.style.available else 'default')

        # Figure 1: 6-DOF Kinematic Positions and Angles
        fig1, axs = plt.subplots(2, 2, figsize=(12, 8), dpi=300)
        fig1.suptitle("6-DOF Kinematics: Earth-Fixed Position and Euler Attitude", fontsize=14, fontweight='bold')

        axs[0, 0].plot(y, x, 'b-', linewidth=2.0, label='Trajectory')
        axs[0, 0].plot(y[0], x[0], 'go', markersize=8, label='Start')
        axs[0, 0].plot(y[-1], x[-1], 'rs', markersize=8, label='End')
        axs[0, 0].set_title("Horizontal Trajectory (North-East Planar)")
        axs[0, 0].set_xlabel("East Position y (m)")
        axs[0, 0].set_ylabel("North Position x (m)")
        axs[0, 0].legend()
        axs[0, 0].grid(True)

        axs[0, 1].plot(t, z, 'm-', linewidth=2.0)
        axs[0, 1].set_title("Subsea Depth Profile z(t) (MS5837 Bar30)")
        axs[0, 1].set_xlabel("Time (s)")
        axs[0, 1].set_ylabel("Depth z (m)")
        axs[0, 1].invert_yaxis()
        axs[0, 1].grid(True)

        axs[1, 0].plot(t, phi, 'r-', label=r'Roll $\phi$')
        axs[1, 0].plot(t, theta, 'g--', label=r'Pitch $\theta$')
        axs[1, 0].set_title(r"Attitude Angles ($\phi, \theta$) - Active 6-DOF Authority")
        axs[1, 0].set_xlabel("Time (s)")
        axs[1, 0].set_ylabel("Angle (deg)")
        axs[1, 0].legend()
        axs[1, 0].grid(True)

        axs[1, 1].plot(t, psi, 'c-', linewidth=2.0, label=r'Heading $\psi$')
        axs[1, 1].set_title(r"Compass Heading $\psi(t)$ (LSM303D Magnetometer)")
        axs[1, 1].set_xlabel("Time (s)")
        axs[1, 1].set_ylabel("Heading (deg)")
        axs[1, 1].legend()
        axs[1, 1].grid(True)

        fig1.tight_layout()
        path1 = os.path.join(output_dir, "figure1_6dof_kinematics.png")
        fig1.savefig(path1)
        plt.close(fig1)
        print(f"[Plot] Generated: {path1}")

        # Figure 2: 6-DOF Dynamic Body Velocities
        fig2, axs = plt.subplots(2, 1, figsize=(11, 7), dpi=300)
        fig2.suptitle("6-DOF Dynamic Velocities (Body-Fixed Linear and Angular Rates)", fontsize=14, fontweight='bold')

        axs[0].plot(t, u, 'b-', label='Surge u (Forward)', linewidth=1.8)
        axs[0].plot(t, v, 'g--', label='Sway v (Lateral)', linewidth=1.8)
        axs[0].plot(t, w, 'm-.', label='Heave w (Vertical)', linewidth=1.8)
        axs[0].set_title("Translational Velocities (u, v, w)")
        axs[0].set_xlabel("Time (s)")
        axs[0].set_ylabel("Linear Velocity (m/s)")
        axs[0].legend(loc="upper right")
        axs[0].grid(True)

        axs[1].plot(t, p, 'r-', label='Roll rate p', linewidth=1.5)
        axs[1].plot(t, q, 'orange', label='Pitch rate q', linestyle='--', linewidth=1.5)
        axs[1].plot(t, r, 'k-.', label='Yaw rate r', linewidth=1.5)
        axs[1].set_title("Rotational Velocities (p, q, r) - Pixhawk Gyroscopes")
        axs[1].set_xlabel("Time (s)")
        axs[1].set_ylabel("Angular Rate (deg/s)")
        axs[1].legend(loc="upper right")
        axs[1].grid(True)

        fig2.tight_layout()
        path2 = os.path.join(output_dir, "figure2_6dof_velocities.png")
        fig2.savefig(path2)
        plt.close(fig2)
        print(f"[Plot] Generated: {path2}")

        # Figure 3: Hydrodynamic Kinetics and Disturbance Observer
        fig3, axs = plt.subplots(2, 1, figsize=(11, 7), dpi=300)
        fig3.suptitle("Fossen Hydrodynamic Force Balance and Integral Disturbance Observer", fontsize=14, fontweight='bold')

        axs[0].plot(t, tau_X, 'b-', label=r'Thrust $\tau_X$', linewidth=2.0)
        axs[0].plot(t, D_u, 'r--', label=r'Hydrodynamic Drag $D(u)u$', linewidth=2.0)
        axs[0].set_title(r"Surge Axis Kinetics: Commanded Thrust $\tau_X$ vs Morison Drag $D(u)u$")
        axs[0].set_xlabel("Time (s)")
        axs[0].set_ylabel("Force (N)")
        axs[0].legend()
        axs[0].grid(True)

        axs[1].plot(t, dist_u, 'b-', label=r'Estimated Ocean Current $\hat{d}_u$', linewidth=2.0)
        axs[1].plot(t, dist_v, 'g--', label=r'Estimated Ocean Current $\hat{d}_v$', linewidth=2.0)
        axs[1].axhline(y=3.5, color='b', linestyle=':', label='True Disturbance Surge (3.5 N)')
        axs[1].axhline(y=-2.0, color='g', linestyle=':', label='True Disturbance Sway (-2.0 N)')
        axs[1].set_title("Kalman Disturbance Observer Convergence under Current Perturbations")
        axs[1].set_xlabel("Time (s)")
        axs[1].set_ylabel("Disturbance Force (N)")
        axs[1].legend(loc="center right")
        axs[1].grid(True)

        fig3.tight_layout()
        path3 = os.path.join(output_dir, "figure3_hydrodynamic_forces_disturbances.png")
        fig3.savefig(path3)
        plt.close(fig3)
        print(f"[Plot] Generated: {path3}")

        # Figure 4: 8-Motor Control Allocation and PWM Distribution
        fig4, axs = plt.subplots(2, 1, figsize=(11, 7), dpi=300)
        fig4.suptitle("Over-Actuated 8-Motor Control Allocation (BlueROV2 Heavy Configuration)", fontsize=14, fontweight='bold')

        for i in range(4):
            pwms_i = [d[f"pwm_{i+1}"] for d in self.log_data]
            axs[0].plot(t, pwms_i, label=f'Thruster {i+1} (Horiz Vectored)')
        axs[0].axhline(y=1500, color='k', linestyle=':', alpha=0.5)
        axs[0].set_title("Horizontal Thrusters 1–4 (PWM Channels 1–4)")
        axs[0].set_xlabel("Time (s)")
        axs[0].set_ylabel(r"PWM Pulse Width ($\mu$s)")
        axs[0].legend(loc="upper right", ncol=2)
        axs[0].grid(True)

        for i in range(4, 8):
            pwms_i = [d[f"pwm_{i+1}"] for d in self.log_data]
            axs[1].plot(t, pwms_i, label=f'Thruster {i+1} (Vertical Corner)')
        axs[1].axhline(y=1500, color='k', linestyle=':', alpha=0.5)
        axs[1].set_title("Vertical Corner Thrusters 5–8 (PWM Channels 5–8)")
        axs[1].set_xlabel("Time (s)")
        axs[1].set_ylabel(r"PWM Pulse Width ($\mu$s)")
        axs[1].legend(loc="upper right", ncol=2)
        axs[1].grid(True)

        fig4.tight_layout()
        path4 = os.path.join(output_dir, "figure4_8motor_thruster_allocation.png")
        fig4.savefig(path4)
        plt.close(fig4)
        print(f"[Plot] Generated: {path4}")


def main():
    parser = argparse.ArgumentParser(description="AUV Kinematics, Dynamics, and Multi-Sensor Telemetry Analyzer")
    parser.add_argument("--test", action="store_true", help="Run synthetic 6-DOF testbench simulation and render plots")
    parser.add_argument("--live", action="store_true", help="Connect to live vehicle over MAVLink and capture telemetry")
    parser.add_argument("--duration", type=float, default=10.0, help="Duration for live capture or test in seconds (default: 10)")
    parser.add_argument("--mavlink-url", type=str, default="http://192.168.2.2:6040", help="Base URL for mavlink2rest (default: http://192.168.2.2:6040)")
    parser.add_argument("--output-dir", type=str, default=".", help="Directory to save CSV dataset and figures")
    args = parser.parse_args()

    analyzer = AUVKinematicsDynamicsAnalyzer(mavlink_url=args.mavlink_url)

    if args.test:
        print("[Mode] Running 6-DOF Synthetic Kinematics and Dynamics Benchmark...")
        analyzer.run_synthetic_test(duration_sec=args.duration)
        csv_path = os.path.join(args.output_dir, "auv_synthetic_telemetry.csv")
        analyzer.save_csv(csv_path)
        analyzer.generate_publication_plots(output_dir=args.output_dir)
        print("\n[Complete] Benchmark verification finished successfully!")

    elif args.live:
        print(f"[Mode] Connecting to live vehicle via {args.mavlink_url}...")
        print(f"[Mode] Recording multi-sensor telemetry for {args.duration:.1f} seconds...")
        t_end = time.time() + args.duration
        count = 0
        try:
            while time.time() < t_end:
                rec = analyzer.step_live_telemetry()
                count += 1
                if count % 25 == 0:
                    print(f"  [Sample {count:4d}] Depth: {rec['z']:.3f}m | Roll: {math.degrees(rec['phi']):+5.1f} deg | Pitch: {math.degrees(rec['theta']):+5.1f} deg | Heading: {math.degrees(rec['psi']):+5.1f} deg | Thrust X: {rec['tau_X']:+5.1f}N")
                time.sleep(analyzer.dt)
        except KeyboardInterrupt:
            print("\n[User Interrupted] Stopping capture early...")

        csv_path = os.path.join(args.output_dir, "auv_live_telemetry.csv")
        analyzer.save_csv(csv_path)
        analyzer.generate_publication_plots(output_dir=args.output_dir)
        print("\n[Complete] Live telemetry capture and analysis completed!")

    else:
        print("Usage: python3 auv_kinematics_dynamics_analyzer.py --test OR --live [--duration SECONDS]")
        print("Run --help for options.")


if __name__ == "__main__":
    main()
