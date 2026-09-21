#!/usr/bin/env python3
"""
AUV Hydrodynamic Dynamics Hardware-In-The-Loop (HIL) Companion Node
===================================================================
Author: Radhi Shafeeq (Hasanuddin University - Mechatronics Engineering)
Project: 5-DOF AUV Autonomous State Estimation & Visual Servoing

Description:
  Executes the 4-DOF Non-linear Hydrodynamic Extended Kalman Filter &
  Disturbance Observer (AUVDynamicsKalmanFilter) live with real vehicle
  hardware: Pixhawk 2.4.8 (ArduSub) + MS5837 Depth/Pressure Sensor.

Execution Modes:
  1. Onboard Subsea (on Raspberry Pi 4B):
     python3 auv_dynamics_hil_node.py --host 127.0.0.1 --freq 50
  2. Topside Remote HIL (on Laptop via Ethernet):
     python3 auv_dynamics_hil_node.py --host 192.168.2.2 --freq 50
"""

import sys
import os
import time
import argparse
import json
import urllib.request
import numpy as np

# Ensure local modules are found
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from kalman_filter import AUVDynamicsKalmanFilter

def parse_args():
    parser = argparse.ArgumentParser(description="AUV Hydrodynamic Dynamics HIL Estimator")
    parser.add_argument("--host", type=str, default="127.0.0.1",
                        help="BlueOS IP address (127.0.0.1 if running on Pi, 192.168.2.2 if on Laptop)")
    parser.add_argument("--port", type=int, default=6040,
                        help="BlueOS mavlink2rest port (default: 6040)")
    parser.add_argument("--freq", type=float, default=50.0,
                        help="Filter execution frequency in Hz (default: 50.0)")
    parser.add_argument("--water", type=str, default="fresh", choices=["fresh", "salt"],
                        help="Water density preset: 'fresh' (1000 kg/m3) or 'salt' (1025 kg/m3)")
    return parser.parse_args()

class SubseaTelemetryBridge:
    """Fast, low-latency REST telemetry bridge to BlueOS mavlink2rest."""
    def __init__(self, host="127.0.0.1", port=6040):
        self.base_url = f"http://{host}:{port}/mavlink/vehicles/1/components/1/messages"
        self.prev_depth = 0.0
        self.prev_depth_time = time.perf_counter()
        self.p_atm = None  # Calibrated atmospheric pressure (hPa)
        
    def fetch_all(self):
        try:
            req = urllib.request.Request(self.base_url)
            with urllib.request.urlopen(req, timeout=0.25) as resp:
                return json.loads(resp.read().decode())
        except Exception:
            return None

def main():
    args = parse_args()
    dt = 1.0 / args.freq
    rho = 1000.0 if args.water == "fresh" else 1025.0
    g = 9.80665

    print("=" * 70)
    print("   AUV HYDRODYNAMIC DYNAMICS EXTENDED KALMAN FILTER (HIL NODE)    ")
    print("=" * 70)
    print(f" Target Host : {args.host}:{args.port}")
    print(f" Frequency   : {args.freq:.1f} Hz (dt = {dt*1000:.1f} ms)")
    print(f" Water Type  : {args.water.upper()} water (rho = {rho:.1f} kg/m³)")
    print(" Connecting to subsea telemetry bridge...")

    bridge = SubseaTelemetryBridge(host=args.host, port=args.port)
    dkf = AUVDynamicsKalmanFilter(dt=dt, mass=11.5)

    # 1. Calibrate Surface Atmospheric Pressure
    print("[Calibrating] Sampling MS5837 pressure sensor for atmospheric baseline...")
    samples = []
    t_start = time.time()
    while time.time() - t_start < 2.0:
        msgs = bridge.fetch_all()
        if msgs and "SCALED_PRESSURE2" in msgs:
            p_meas = msgs["SCALED_PRESSURE2"].get("message", {}).get("press_abs")
            if p_meas is not None:
                samples.append(p_meas)
        time.sleep(0.05)

    if samples:
        p_atm = float(np.median(samples))
        print(f"[Calibrated] Atmospheric Baseline Pressure P_atm = {p_atm:.2f} hPa")
    else:
        p_atm = 1013.25
        print(f"[Warning] Could not read MS5837; defaulting P_atm = {p_atm:.2f} hPa")

    print("\n[Running] Dynamics EKF Active! Streaming live state estimates:")
    print("Surge(u) | Sway(v) | Heave(w) | YawRate(r) | Depth(z) | Dist(du,dv) | Temp")
    print("-" * 75)

    step_count = 0
    t_prev = time.perf_counter()
    prev_z = 0.0

    while True:
        cycle_start = time.perf_counter()
        msgs = bridge.fetch_all()

        if msgs:
            # A. Depth & Vertical Velocity from MS5837
            p_msg = msgs.get("SCALED_PRESSURE2", {}).get("message", {})
            p_abs = p_msg.get("press_abs", p_atm)
            temp_c = p_msg.get("temperature", 0) / 100.0

            # Hydrostatic pressure equation: P = P_atm + rho * g * z
            delta_p_pa = max(0.0, (p_abs - p_atm)) * 100.0  # hPa -> Pa
            depth_m = delta_p_pa / (rho * g)

            # Heave velocity from depth differentiation
            w_meas = (depth_m - prev_z) / dt
            prev_z = depth_m

            # B. Angular Rate & Attitude from Pixhawk
            att = msgs.get("ATTITUDE", {}).get("message", {})
            r_meas = att.get("yawspeed", 0.0)
            roll_deg = np.degrees(att.get("roll", 0.0))
            pitch_deg = np.degrees(att.get("pitch", 0.0))
            yaw_deg = np.degrees(att.get("yaw", 0.0))

            # C. Linear Acceleration / Surmise velocities
            u_meas = 0.0
            v_meas = 0.0

            # D. Commanded Thrust from Servo Outputs / Manual Control
            # M1-M4 horizontal vectored thrusters, M5-M6 vertical
            tau_cmd = [0.0, 0.0, 0.0, 0.0]  # [surge, sway, heave, yaw]

            # E. EKF Predict and Update
            dkf.predict(tau_cmd)
            dkf.update([u_meas, v_meas, w_meas, r_meas])

            u_est, v_est, w_est, r_est = dkf.get_velocities()
            du_est, dv_est = dkf.get_disturbance_forces()

            step_count += 1
            if step_count % 5 == 0:  # Print every 5 cycles (10 Hz terminal refresh)
                print(f" {u_est:+5.2f}m/s | {v_est:+5.2f}m/s | {w_est:+5.2f}m/s | {r_est:+5.2f}r/s | "
                      f"{depth_m*100:6.1f}cm | {du_est:+4.1f},{dv_est:+4.1f}N | {temp_c:4.1f}°C",
                      end="\r", flush=True)

        elapsed = time.perf_counter() - cycle_start
        sleep_time = max(0.0, dt - elapsed)
        time.sleep(sleep_time)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n[Stopped] Dynamics HIL node terminated cleanly.")
