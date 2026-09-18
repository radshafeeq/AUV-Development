#!/usr/bin/env python3
"""
AUV Real-Time Velocity & State Dashboard
Reads Gazebo Harmonic odometry topic and displays human-readable, formatted 6-DOF velocity
with visual gauges, units, and thesis notation (u, v, w, p, q, r).
"""

import sys
import os
import re
import math
import time
import subprocess
import argparse

# ANSI color codes for clean terminal UI
CYAN = "\033[96m"
GREEN = "\033[92m"
YELLOW = "\033[93m"
RED = "\033[91m"
BLUE = "\033[94m"
BOLD = "\033[1m"
RESET = "\033[0m"
CLEAR_SCREEN = "\033[2J\033[H"
HIDE_CURSOR = "\033[?25l"
SHOW_CURSOR = "\033[?25h"

def quat_to_euler(x, y, z, w):
    """Converts quaternion to Euler angles (Roll, Pitch, Yaw in degrees)."""
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)

def create_bar(val, max_val=1.0, length=20):
    """Creates a centered visual bi-directional bar: [    <===|====>    ]"""
    val = max(-max_val, min(max_val, val))
    center = length // 2
    chars = [" "] * length
    chars[center] = "|"

    ratio = val / max_val
    offset = int(abs(ratio) * center)

    if ratio > 0:
        for i in range(center + 1, min(length, center + 1 + offset)):
            chars[i] = "="
        if center + offset < length:
            chars[center + offset] = ">"
    elif ratio < 0:
        for i in range(max(0, center - offset), center):
            chars[i] = "="
        if center - offset >= 0:
            chars[center - offset] = "<"

    bar_str = "".join(chars)
    if abs(val) < 0.02:
        color = RESET
    elif abs(val) < 0.5 * max_val:
        color = GREEN
    else:
        color = YELLOW
    return f"[{color}{bar_str}{RESET}]"

def main():
    parser = argparse.ArgumentParser(description="Human-Readable AUV Velocity Monitor")
    parser.add_argument("--topic", default="/model/bluerov2_heavy/odometry", help="Gazebo odometry topic")
    args = parser.parse_args()

    # Launch gz topic -e as a subprocess
    cmd = ["gz", "topic", "-e", "-t", args.topic]
    try:
        proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)
    except Exception as e:
        print(f"Failed to run gz topic: {e}")
        return

    # Vehicle state dictionary
    state = {
        "pos_x": 0.0, "pos_y": 0.0, "pos_z": 0.0,
        "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0,
        "roll": 0.0, "pitch": 0.0, "yaw": 0.0,
        "u": 0.0, "v": 0.0, "w": 0.0,
        "p": 0.0, "q": 0.0, "r": 0.0,
        "speed": 0.0
    }

    current_section = None
    sub_section = None
    last_render = time.time()
    msg_count = 0

    print(HIDE_CURSOR, end="")
    try:
        for line in proc.stdout:
            line_str = line.strip()
            
            # Detect section transitions
            if line_str.startswith("pose {"):
                current_section = "pose"
                sub_section = None
            elif line_str.startswith("twist {"):
                current_section = "twist"
                sub_section = None
            elif line_str.startswith("position {"):
                sub_section = "position"
            elif line_str.startswith("orientation {"):
                sub_section = "orientation"
            elif line_str.startswith("linear {"):
                sub_section = "linear"
            elif line_str.startswith("angular {"):
                sub_section = "angular"
            elif line_str == "}":
                if sub_section:
                    sub_section = None
                elif current_section:
                    current_section = None
                    msg_count += 1
            
            # Parse key-values
            m = re.match(r"([xyz|w]):\s*([-+]?[0-9]*\.?[0-9]+(?:[eE][-+]?[0-9]+)?)", line_str)
            if m:
                axis, val = m.group(1), float(m.group(2))
                if current_section == "pose":
                    if sub_section == "position":
                        if axis == "x": state["pos_x"] = val
                        elif axis == "y": state["pos_y"] = val
                        elif axis == "z": state["pos_z"] = val
                    elif sub_section == "orientation":
                        if axis == "x": state["qx"] = val
                        elif axis == "y": state["qy"] = val
                        elif axis == "z": state["qz"] = val
                        elif axis == "w": state["qw"] = val
                elif current_section == "twist":
                    if sub_section == "linear":
                        if axis == "x": state["u"] = val
                        elif axis == "y": state["v"] = val
                        elif axis == "z": state["w"] = val
                    elif sub_section == "angular":
                        if axis == "x": state["p"] = math.degrees(val)
                        elif axis == "y": state["q"] = math.degrees(val)
                        elif axis == "z": state["r"] = math.degrees(val)

            # Render at max 10 Hz to prevent eye fatigue
            now = time.time()
            if now - last_render >= 0.1:
                last_render = now
                roll, pitch, yaw = quat_to_euler(state["qx"], state["qy"], state["qz"], state["qw"])
                state["roll"], state["pitch"], state["yaw"] = roll, pitch, yaw
                state["speed"] = math.sqrt(state["u"]**2 + state["v"]**2 + state["w"]**2)

                # Format human readable output
                out = [
                    CLEAR_SCREEN,
                    f"{BOLD}{CYAN}========================================================================{RESET}",
                    f"{BOLD}{CYAN}               AUV REAL-TIME VELOCITY & STATE DASHBOARD                 {RESET}",
                    f"{BOLD}{CYAN}========================================================================{RESET}",
                    f"Topic: {GREEN}{args.topic}{RESET}  |  Sample Rate: {GREEN}50 Hz (Output: 10 Hz){RESET}",
                    "",
                    f"{BOLD}1. LINEAR VELOCITY (Translational Motion in Body-Fixed Frame){RESET}",
                    f"   Surge (u, Forward)  : {BOLD}{state['u']:+6.3f} m/s{RESET} ({state['u']*100:+6.1f} cm/s)  {create_bar(state['u'], 1.0)}",
                    f"   Sway  (v, Lateral)  : {BOLD}{state['v']:+6.3f} m/s{RESET} ({state['v']*100:+6.1f} cm/s)  {create_bar(state['v'], 1.0)}",
                    f"   Heave (w, Vertical) : {BOLD}{state['w']:+6.3f} m/s{RESET} ({state['w']*100:+6.1f} cm/s)  {create_bar(state['w'], 1.0)}",
                    f"   Total Speed (||V||) : {BOLD}{GREEN}{state['speed']:6.3f} m/s{RESET} ({state['speed']*100:6.1f} cm/s)",
                    "",
                    f"{BOLD}2. ANGULAR VELOCITY (Rotational Rates in Body-Fixed Frame){RESET}",
                    f"   Roll Rate  (p)      : {BOLD}{state['p']:+6.2f} °/s{RESET}  {create_bar(state['p'], 45.0)}",
                    f"   Pitch Rate (q)      : {BOLD}{state['q']:+6.2f} °/s{RESET}  {create_bar(state['q'], 45.0)}",
                    f"   Yaw Rate   (r)      : {BOLD}{state['r']:+6.2f} °/s{RESET}  {create_bar(state['r'], 45.0)}",
                    "",
                    f"{BOLD}3. VEHICLE POSE & DEPTH (World Frame){RESET}",
                    f"   Depth (Submerged)   : {BOLD}{YELLOW}{-state['pos_z']:6.2f} m{RESET} (Position Z: {state['pos_z']:+6.2f} m)",
                    f"   Position (X, Y)     : ({state['pos_x']:+6.2f} m, {state['pos_y']:+6.2f} m)",
                    f"   Attitude (R, P, Y)  : Roll: {state['roll']:+5.1f}°, Pitch: {state['pitch']:+5.1f}°, Heading: {state['yaw']:+5.1f}°",
                    f"{BOLD}{CYAN}------------------------------------------------------------------------{RESET}",
                    f"{BLUE}Press Ctrl+C in this window to stop monitoring.{RESET}"
                ]
                sys.stdout.write("\n".join(out) + "\n")
                sys.stdout.flush()

    except KeyboardInterrupt:
        pass
    finally:
        print(SHOW_CURSOR)
        if proc.poll() is None:
            proc.terminate()

if __name__ == "__main__":
    main()
