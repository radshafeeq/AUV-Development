# Custom 4-DOF AUV Simulation
This repository contains the simulation files for a custom 6-motor, 4-DOF Autonomous Underwater Vehicle (AUV). This project is being developed as part of an undergraduate Mechatronics Engineering thesis at Hasanuddin University.

The simulation integrates a custom 3D AUV frame design with Gazebo Harmonic (for physics and 3D rendering) and ArduSub SITL (for the flight controller and vehicle dynamics).

## Architecture Overview
The simulation is split into two main components that communicate over a local network:

**Gazebo Sim:** Handles the 3D graphics, water physics, buoyancy, and collisions using the custom 3D .obj models.

**ArduSub SITL (Software In The Loop):** Acts as the "brain" of the robot. It runs the ArduSub firmware, calculates motor thrusts based on the 6-motor geometry, and provides a MAVLink interface via MAVProxy.

## 📚 Master Academic Reference Monographs (Thesis Documentation)
This repository includes two publication-grade, unabridged theoretical monographs grounded in the master research library:
- **Monograph 1 — Visual Servoing & State Estimation**: [`AUV_Kalman_Filter_Comprehensive_Explanation.md`](AUV_Kalman_Filter_Comprehensive_Explanation.md)  
  *Exhaustive theoretical derivation of discrete Kalman filtering (DKF, EKF, UKF, EIF, RHKF), CWNA process noise covariance discretization ($\mathbf{Q}$), dual-filter architecture (`AUVVisualKalmanFilter` on Topside Laptop + `AUVDynamicsKalmanFilter` on Raspberry Pi 4B), zero-allocation optimization ($13.49\text{ \mu s}$ / $20.99\text{ \mu s}$), Fossen (2021) 4-DOF hydrodynamic plant model, subsea current disturbance observer, and Hardware-in-the-Loop (HIL) dry bench test methodology.*
- **Monograph 2 — Kinematic and Dynamic Modeling**: [`AUV_Kinematics_and_Dynamics_Comprehensive_Derivation.md`](AUV_Kinematics_and_Dynamics_Comprehensive_Derivation.md)  
  *Exhaustive first-principles derivation of 6-DOF kinematics ($SO(3)$ rotation matrix $\mathbf{R}_b^n$, $\mathbf{T}_\Theta$ matrix inversion, quaternions), Fossen's 6-DOF kinetics plant model (mass, Coriolis, damping, hydrostatics, 8-thruster allocation), variable-by-variable 4-DOF reduction, and the first-principles proof of the destabilizing hydrodynamic Munk Moment.*

## How to Run the Simulation

You can launch either your **Custom 4-DOF AUV (6 Thrusters)** or the **BlueROV2 Heavy (8 Thrusters / 6-DOF)** simulation environment.

### Option A: Custom 4-DOF AUV Simulation - "Poseidon AUV" (6 Thrusters)
1. **Launch Gazebo Physics World**:
   ```bash
   ./start_gazebo.sh
   ```
2. **Launch ArduSub SITL Flight Controller**:
   ```bash
   ./start_ardusub.sh
   ```
3. **Launch Blue Robotics Cockpit GCS**:
   ```bash
   ./start_cockpit.sh
   ```

---

### Option B: BlueROV2 Heavy Simulation (8 Thrusters / 6-DOF)
1. **Launch Gazebo Physics World**:
   ```bash
   ./start_bluerov2_heavy_gazebo.sh
   ```
2. **Launch ArduSub SITL Flight Controller**:
   ```bash
   ./start_bluerov2_heavy_ardusub.sh
   ```
3. **Launch Blue Robotics Cockpit GCS**:
   ```bash
   ./start_cockpit.sh
   ```

---

## 📊 Real-Time 6-DOF Velocity & State Dashboard (Ground Truth)

To fulfill academic thesis requirements for comparing state estimations (Kalman Filter) against exact physics simulator measurements, a real-time **Odometry & Velocity Dashboard** (`display_velocity.py`) is integrated directly into the Gazebo simulation.

### ⚙️ How It Works:
1. **Gazebo System Plugin (`OdometryPublisher`)**:
   - The Gazebo model files (`model.sdf` and `model.sdf.in` for both BlueROV2 standard and BlueROV2 Heavy) include the `gz::sim::systems::OdometryPublisher` plugin.
   - It silently samples the true vehicle state at **50 Hz** from the Gazebo physics engine without altering any rigid-body mass, inertia, or hydrodynamic drag properties.
   - Ground truth data is published to `/model/bluerov2_heavy/odometry` (or `/model/bluerov2/odometry`).

2. **Human-Readable HUD (`display_velocity.py`)**:
   - The raw Gazebo protobuf stream outputs 50 messages per second with 16 decimal places, which causes severe visual fatigue and cannot be monitored during manual piloting.
   - `display_velocity.py` intercepts this stream, formats the data, and renders an in-place terminal dashboard refreshed smoothly at **10 Hz**.
   - Features include:
     - **Body-Fixed Linear Velocity**: Surge ($u$), Sway ($v$), and Heave ($w$) in both $\text{m/s}$ and $\text{cm/s}$, plus Total Speed ($||V|| = \sqrt{u^2 + v^2 + w^2}$).
     - **Body-Fixed Angular Rates**: Roll rate ($p$), Pitch rate ($q$), and Yaw rate ($r$) in $\text{deg/s}$.
     - **World Pose & Attitude**: Real-time submerged depth in meters and Euler angles (Roll, Pitch, Heading in degrees) converted from quaternions.
     - **Visual Direction Gauges**: Centered bidirectional indicator bars `[   <===|===>   ]`.

### 🚀 Launching the Dashboard:
The dashboard is automatically launched by `start_bluerov2_heavy_gazebo.sh` and `start_gazebo.sh`. To launch or restart it independently:

```bash
# For BlueROV2 Heavy (8 thrusters):
python3 display_velocity.py --topic /model/bluerov2_heavy/odometry

# For Custom / BlueROV2 Standard (6 thrusters):
python3 display_velocity.py --topic /model/bluerov2/odometry
```

---

## 🎯 Real-Time YOLO26 & Open-Vocabulary AI Target Tracking

This repository includes a zero-latency real-time AI computer vision & closed-loop visual servoing tracking node accelerated on **NVIDIA RTX GPUs** (CUDA).

### 🚀 How to Run the Detection Camera & Target Tracking:
To launch the real-time AI vision camera node with live overlay, bounding boxes, tracking vectors, and automatic MAVLink control:

```bash
python3 auv_yolo_tracking.py
```

### ⚙️ Switching Between Detection Modes in `auv_yolo_tracking.py`:

Open `auv_yolo_tracking.py` and set `USE_YOLO_WORLD`:

1. **YOLO26 Custom Model (Fine-Tuned 11-Class Model)**:
   ```python
   USE_YOLO_WORLD = False  # Loads runs/detect/yolo26_combined_model/weights/best.pt
   ```
   - **Trained Classes**: `bldc_motor`, `pixhawk`, `esc`, `battery`, `charger`, `cable`, `soldering_iron`, `underwater_buoy`, `underwater_gate`, `exit sign`, `fire hydrant`

2. **YOLO-World (Open-Vocabulary Zero-Shot Detection)**:
   ```python
   USE_YOLO_WORLD = True   # Zero-shot recognition for ANY text prompt
   YOLO_WORLD_CLASSES = ["person", "laptop", "bldc motor", "window", "air conditioner", "water bottle"]
   ```
   - Instantly detects 120+ lab, office, campus, and room objects without dataset training!

---

### 📈 Dual Kalman Filter Suite: Visual Estimation & Hydrodynamic Observer

The autonomy stack incorporates a high-performance **Dual Kalman Filter Suite** implemented in [`kalman_filter.py`](file:///home/radhi/Documents/AUV_GitHub_Upload/kalman_filter.py), dividing state estimation responsibilities between the topside GPU workstation and the subsea companion computer:

```
                            ETHERNET TETHER (192.168.2.x)
     TOPSIDE WORKSTATION (LAPTOP)                 │             SUBSEA VEHICLE (AUV HULL)
┌────────────────────────────────────────────┐    │    ┌─────────────────────────────────────────┐
│ • RTSP H.264 Video Ingestion (50-60 FPS)   │◄───┼────│ • RPi 4B: BlueOS System & Camera Server │
│ • YOLO26 World Neural Detection (RTX GPU)  │    │    │ • AUVDynamicsKalmanFilter (4-DOF EKF)   │
│ • AUVVisualKalmanFilter (8D Position/Scale)│    │    │   Estimates [u, v, w, r] & currents     │
│ • Visual Servoing Guidance Law             │────┼───►│ • Autonomous Tracking Setpoints         │
└────────────────────────────────────────────┘    │    └────────────────────┬────────────────────┘
                                                  │                         │ USB MAVLink (/dev/ttyACM0)
                                                  │                         ▼
                                                  │    ┌─────────────────────────────────────────┐
                                                  │    │ • Pixhawk 2.4.8: Stock ArduSub Firmware │
                                                  │    │   400 Hz EKF3 IMU/Depth Attitude PID    │
                                                  │    │   6x T200 PWM Motor Mixing              │
                                                  └────┴─────────────────────────────────────────┘
```

#### 1. Topside Visual Kalman Filter (`AUVVisualKalmanFilter` on Laptop GPU/CPU)
- **Role**: Smooths noisy YOLO detections, predicts trajectories through occlusions/turbidity, and computes monocular surge approach rates.
- **State Vector**: 8D $\mathbf{x}_{\text{vis}} = [x, y, w, h, v_x, v_y, v_w, v_h]^T$ *(Centroid + Dimensions + Velocities)*
- **Measurement Vector**: $\mathbf{z}_k = [x_m, y_m, w_m, h_m]^T$ *(Raw YOLO bounding box)*
- **Zero-Allocation Optimization**: Built using Python `__slots__` and pre-allocated NumPy contiguous memory buffers—eliminates garbage collection pauses during high-speed tracking.
- **Adaptive Time-Step ($\Delta t$)**: Automatically measures elapsed hardware time (`time.perf_counter()`), adjusting the state transition matrix $\mathbf{A}(\Delta t)$ to prevent velocity distortion during network frame jitters.
- **Confidence-Scaled Measurement Covariance**: Dynamically weights $\mathbf{R}(\text{conf}) = \mathbf{R}_0 / \max(\text{conf}, 0.15)^2$. High-confidence detections are tightly tracked; lower-confidence detections in murky water rely more on model dead-reckoning.
- **Innovation Outlier Gating**: Mahalanobis distance gating rejects sudden water surface reflections, bubbles, and false positive detections ($> 300\text{ px}$).
- **Scale Rate & Monocular Standoff**: Computes analytical projected area expansion rate $\frac{d\mathcal{A}}{dt} = v_w h + w v_h$, providing closed-loop feedback for forward distance holding without an acoustic DVL.
- **Performance Benchmark**: **$13.49\text{ \mu s}$** execution time per step (>74,000 Hz throughput).

#### 2. Subsea Hydrodynamic Dynamics Kalman Filter (`AUVDynamicsKalmanFilter` on Raspberry Pi 4B)
- **Role**: Non-linear Extended Kalman Filter and Disturbance Observer fusing Pixhawk IMU/depth telemetry with thruster command efforts.
- **Physical Model**: Fossen's (2021) 4-DOF marine craft equations of motion:
  $$\mathbf{M} \dot{\boldsymbol{\nu}} + \mathbf{C}(\boldsymbol{\nu})\boldsymbol{\nu} + \mathbf{D}(\boldsymbol{\nu})\boldsymbol{\nu} = \boldsymbol{\tau} + \mathbf{d}$$
- **State Vector**: 6D $\mathbf{x}_{\text{dyn}} = [u, v, w, r, d_u, d_v]^T$ *(Surge, Sway, Heave, Yaw rate, plus Ocean Current Disturbance Forces $d_u, d_v$)*
- **Inertia Matrix**: $\mathbf{M} = \text{diag}[17.86, 18.62, 30.18, 0.25]$ kg, kg$\cdot\text{m}^2$ (including hydrodynamic added mass).
- **Coupled Quadratic Damping**: $\mathbf{D}_{\text{lin}} = [13.7, 0, 33.8, 0]^T$, $\mathbf{D}_{\text{quad}} = [141.0, 217.0, 190.0, 1.5]^T$.
- **Disturbance Observer**: Uncouples vehicle thrust from external environmental currents, providing direct current force estimates ($d_u, d_v$ in Newtons) for active trim compensation.
- **Performance Benchmark**: **$20.99\text{ \mu s}$** execution time per step (>47,000 Hz throughput on Raspberry Pi ARM Cortex-A72).

---

### 🏋️‍♂️ How to Train YOLO26 Model on Combined Dataset:
To fine-tune YOLO26 on the combined Mechatronics + Office dataset on your GPU:

```bash
python3 train_yolo26.py 25
```

---

## Basic Control Commands
Once the ArduSub terminal is running, you will see a `MANUAL>` prompt. You can use the following MAVLink commands to control the AUV:

- **Arm the thrusters:** `arm throttle`
- **Change flight mode:** `mode alt_hold` (Depth hold mode)
- **Apply vertical thrust (heave):** `rc 3 1600` (1500 is neutral/stop, 1600 is up, 1400 is down)
- **Stop thrust:** `rc 3 1500`
- **Disarm the AUV:** `disarm`

---

# Real-Time AI Camera Detection & Target Tracking

This repository includes a high-performance **Real-Time AI Computer Vision & Autonomous Tracking Node** powered by **YOLO26 World** open-vocabulary zero-shot detection and an **8D Position + Scale Kalman Filter**, designed for topside GPU acceleration (NVIDIA RTX 4070 Laptop GPU) connected to the AUV via BlueOS / Ethernet tether (`192.168.2.2`).

## Overview & Architecture
- **Dual Camera Feeds via BlueOS 1.4.5 on Raspberry Pi 4B**:
  - **Camera 1 (Primary)**: **Raspberry Pi CSI Camera Module** (`mmal service 16.1`) streaming `640x480` @ 30 FPS via zero-latency RTP H.264 on UDP port **`5600`**.
  - **Camera 2 (Secondary / Bench Test)**: **Logitech C922 USB Webcam** streaming 720p HD (`1280x720` @ 30 FPS) via zero-latency RTP JPEG on UDP port **`5601`**.
  - Switch between cameras in real-time by pressing **`[c]`**.
- **Real-Time CLAHE Dynamic Contrast Enhancer**:
  - Contrast Limited Adaptive Histogram Equalization applied dynamically to the $L$-channel in LAB color space (`apply_clahe`).
  - Penetrates turbid, dark, and backscatter-heavy underwater scenes; toggled instantly in real-time via hotkey **`[e]`**.
- **High-Resolution AI Inference Engine**:
  - Ultralytics **YOLO26 World** (`weights/yolo26_world.pt`) running at **`imgsz=1024`** on NVIDIA RTX 4070 GPU (CUDA).
  - Configured with a 70+ class open-vocabulary dictionary spanning bench electronics, mechatronics lab tools, AUV hardware, and subsea inspection targets.
- **8D Target Tracking & State Estimation**:
  - Continuous-Velocity 8D Kalman Filter ($[x, y, w, h, v_x, v_y, v_w, v_h]^T$) with CWNA covariance modeling.
  - Generates smoothed bounding boxes and optical expansion rates ($\dot{\mathcal{A}}$) for forward surge standoff regulation.
- **Closed-Loop Visual Servoing**:
  - Computes normalized centering error $(e_x, e_y)$ and dispatches PyMAVLink `MANUAL_CONTROL` commands to ArduSub at 30 Hz.

---

## How to Run the AI Topside Camera Tracker

### Quick Start (Terminal Command)
Open a terminal on your topside computer and run:

```bash
cd ~/Documents/AUV_GitHub_Upload
/home/radhi/venv-ardupilot/bin/python3 auv_yolo_tracking.py
```

### Hotkey Controls in Live Display Window
| Key | Action | Description |
|---|---|---|
| **`c`** | **Switch Camera** | Toggles between **RPi CSI Cam (Port 5600)** and **Logitech C922 (Port 5601)** |
| **`e`** | **Toggle CLAHE** | Enables/disables real-time underwater adaptive contrast enhancement |
| **`r`** | **Rotate 90° CW** | Cycles camera orientation (0° -> 90° CW -> 180° -> 270° CW) |
| **`f`** | **Flip 180°** | Toggles 180° upside-down flip |
| **`+` / `=`** | **Confidence Up** | Increases detection confidence threshold (+0.02) |
| **`-` / `_`** | **Confidence Down** | Decreases detection confidence threshold (-0.02) |
| **`q`** | **Exit** | Closes camera stream and shuts down cleanly |

---

## Future Underwater Target Fine-Tuning Workflow

To fine-tune YOLO26 specifically on custom underwater targets (competition buoys, gates, markers):

```bash
cd ~/Documents/AUV_GitHub_Upload
/home/radhi/venv-ardupilot/bin/python3 train_yolo26.py 25
```
- Trains YOLO26 on custom underwater annotated datasets.
- Saves custom weights to `runs/detect/yolo26_combined_model/weights/best.pt`.

---

## 🔬 Hardware-in-the-Loop (HIL) Dry Bench Testing Guide

You can validate the complete autonomous tracking and control pipeline on a workbench **without building the full waterproof hull, mounting thrusters, or testing in water**.

```
┌───────────────────────────┐                        ┌─────────────────────────────────────────┐
│     Topside Laptop        │                        │      Raspberry Pi 4B (BlueOS 1.4.5)     │
│  • auv_yolo_tracking.py   │◄─── Ethernet Tether ──►│  • RTSP/RTP Video Streamer              │
│  • AUVVisualKalmanFilter  │    (192.168.2.x)       │  • MAVLink Router (Endpoints on 14550)  │
│  • Cockpit / QGC HUD      │                        │  • AUVDynamicsKalmanFilter Observer     │
└───────────────────────────┘                        └────────────────────┬────────────────────┘
                                                                          │ USB Telemetry Cable
                                                                          ▼
                                                     ┌─────────────────────────────────────────┐
                                                     │         Pixhawk 2.4.8 (ArduSub)         │
                                                     │  • 400 Hz Internal Attitude EKF3        │
                                                     │  • Servo PWM Output Mixer (Ch 1 - 6)    │
                                                     └─────────────────────────────────────────┘
```

### 1. Hardware Connections:
1. Connect the **Raspberry Pi 4B** to the **Pixhawk 2.4.8** using a standard micro-USB to USB-A cable (`/dev/ttyACM0`).
2. Plug the **Pi Camera module** (CSI flat cable) and/or the **Logitech C922 USB webcam** into the Raspberry Pi 4B.
3. Connect the **Topside Laptop** to the Raspberry Pi 4B with an Ethernet cable. Set your laptop's Ethernet IPv4 address to `192.168.2.1` (netmask `255.255.255.0`).
4. Power the Raspberry Pi (5V USB-C) and Pixhawk (via USB or Power Module).

### 2. Bypass Pre-Arming Checks for Dry Benchtop Testing:
On a dry desk, ArduSub will block arming because the external Bar30 / MS5837 underwater pressure sensor is absent. To allow bench testing:
1. Open the BlueOS web interface at `http://192.168.2.2` or launch **Cockpit** / **QGroundControl**.
2. Go to **Parameters** and set:
   ```text
   ARMING_CHECK = 0
   ```
3. Reboot the Pixhawk or restart ArduSub.

### 3. Step-by-Step Bench Test Verification:
1. **Attitude / IMU Tracking Verification**:
   - Open **Cockpit** (`./start_cockpit.sh`) or check the GCS artificial horizon.
   - Pick up the Pixhawk and rotate it along pitch, roll, and yaw.
   - Confirm that the artificial horizon and 3D vehicle orientation in Cockpit respond instantly to physical movements.
2. **Launch Topside Autonomous AI Tracking**:
   ```bash
   cd ~/Documents/AUV_GitHub_Upload
   /home/radhi/venv-ardupilot/bin/python3 auv_yolo_tracking.py
   ```
3. **Arm Thrusters in MANUAL Mode**:
   - In Cockpit or your MAVLink terminal, arm the vehicle (`arm throttle`). Keep mode in `MANUAL`.
4. **Visual Closed-Loop Servoing & PWM Output Verification**:
   - Place a test object (e.g., a BLDC motor, electronics, or water bottle) in front of the camera.
   - Watch the HUD: YOLO26 World detects the object, the green `AUVVisualKalmanFilter` box locks onto target centroid, and the tracking error vectors display current pixel offsets $(e_x, e_y)$.
   - Move the target object to the left or right:
     - The script transmits MAVLink `MANUAL_CONTROL` yaw packets at 30 Hz.
     - Observe the **Servo Outputs** (`SERVO_OUTPUT_RAW`) in Cockpit or MAVProxy. The PWM signals on thruster channels 1–4 will dynamically deviate from neutral ($1500\text{ \mu s}$) in proportion to target offset.
   - Move the target closer or further away:
     - The Kalman filter computes projected area expansion rate $\frac{d\mathcal{A}}{dt}$, driving forward/backward surge channels accordingly.

> [!TIP]
> This dry HIL bench test confirms your entire vision-to-control pipeline—from camera photon capture through neural inference, Kalman filtering, MAVLink packet transmission, and Pixhawk PWM motor mixing—before immersing any hardware into water!

### 5. Direct Laptop USB Bench Camera Evaluation (`test_detection_camera.py` v2.3)
If your webcam (Logitech C922) is plugged directly into your laptop via USB rather than through BlueOS:
```bash
cd ~/Documents/AUV_GitHub_Upload
/home/radhi/venv-ardupilot/bin/python3 test_detection_camera.py
```
- **Simultaneous Multi-Object Detection & Real-Time Tracking**:
  - **Red Boxes (Raw YOLO Detections)**: ALL detected objects in view (Laptop, Bottle, Mouse, Keyboard, Smartphone, Multimeter, Cables, etc.) are enclosed in crisp **Red Bounding Boxes** displaying their class name and confidence score simultaneously.
  - **Green Box (8D Kalman Filter)**: The primary tracked object is highlighted with the bold **Bright Green / Cyan Kalman Box**, with smoothed coordinates, optical center crosshair line, and velocity vector arrows ($v_x, v_y$).
  - **Smart Auto-Relocking**: Prevents signal loss or dead locks. If a target is moved or occluded, the tracker dynamically tracks active objects with zero signal dropouts.
  - **Cyan Box**: Dead-reckoning trajectory bridging when the object is temporarily occluded (e.g., placing hands in front of the camera).
- **Manual Focus Slider Matched with Auto-Focus**:
  - **Real-Time Slider Sync**: The slider knob dynamically glides along the vertical track to reflect the camera's true optical focus position during autofocus sweeps or manual tuning.
  - **`[MODE: AUTO]` / `[MODE: MANUAL]` Button**: Toggle continuous autofocus directly from the on-screen card.
  - **`[AUTO FOCUS]` Button**: Execute a synchronized contrast-maximization autofocus sweep on the current target with a single click.
  - **`AF*` Target Notch**: Shows the autofocus convergence point directly on the slider track.
  - **Quick Presets**: Click `[ROOM 15]` to snap to verified razor-sharp room standoff ($~1.5\text{m}$ to $\infty$) or `[DESK 40]` for close desk inspection ($~40\text{cm}$).
  - **Mouse Wheel Tuning**: Scroll the mouse wheel anywhere over the window to nudge focus by $\pm 2$.
- **Interactive Controls & Hotkeys**:
  - **`[Right Slider]`**: Drag knob with mouse to adjust focus in real time.
  - **`[Left-Click]`**: Click any object on screen to lock target AND trigger optical autofocus.
  - `[1]`: Switch to **Full HD 1080p Mode** (`1920x1080` @ 30 FPS).
  - `[2]`: Switch to **High-Speed 720p Mode** (`1280x720` @ 60 FPS, default).
  - `[t]`: Reset target lock (reverts to auto-tracking).
  - `[k]`: Toggle Kalman Filter ON / OFF for instant before-and-after comparison.
  - `[s]`: Toggle **Side-by-Side Split-Screen** mode (Left = Raw YOLO, Right = 8D Kalman Filter).
  - `[a]`: Toggle Agility: Agile Zero-Lag ($q_s = 1.0$) vs. Heavy Smooth ($q_s = 0.08$).
  - `[+]` / `[-]`: Adjust detection confidence threshold ($\pm 0.02$).
  - `[q]`: Exit cleanly.
- **Live Jitter & Sharpness HUD**: Real-time stabilization percentage ($>75\%$ reduction in jitter) and numerical optical sharpness readout displayed on the target bounding box.

---

# Fresh Install Guide — Setting Up on a New Device

This section explains how to install all the required software from scratch on a new Ubuntu machine and reproduce this simulation environment.

## System Requirements

| Component | Required Version |
|---|---|
| **OS** | Ubuntu 24.04 LTS (Noble Numbat) |
| **ROS 2** | Jazzy Jalisco |
| **Gazebo** | Harmonic (gz-harmonic) |
| **ArduSub** | Latest (from ArduPilot source) |
| **Python** | 3.12+ |
| **Git** | Any recent version |

> **Important:** All versions must match exactly. Mixing Gazebo versions (e.g., Fortress vs Harmonic) will cause plugin incompatibilities.

---

## Step 1 - Install Ubuntu 24.04

Download Ubuntu 24.04 LTS from https://ubuntu.com/download/desktop and install it.

After installation, update your system:

```bash
sudo apt update && sudo apt upgrade -y
```

---

## Step 2 - Install ROS 2 Jazzy

```bash
# Set up locale
sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# Add ROS 2 repository
sudo apt install -y software-properties-common curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
  sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Jazzy
sudo apt update
sudo apt install -y ros-jazzy-desktop

# Source ROS 2 - add to .bashrc so it loads automatically in every terminal
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

Verify:
```bash
ros2 --version
```

---

## Step 3 - Install Gazebo Harmonic

```bash
# Add Gazebo repository
sudo curl https://packages.osrfoundation.org/gazebo.gpg \
  --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
  http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | \
  sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# Install Gazebo Harmonic
sudo apt update
sudo apt install -y gz-harmonic
```

Verify:
```bash
gz sim --version
```

---

## Step 4 - Install ArduPilot and ArduSub

```bash
# Clone ArduPilot
cd ~
git clone https://github.com/ArduPilot/ardupilot.git
cd ardupilot

# Install dependencies
Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile

# Build ArduSub SITL (Software In The Loop)
./waf configure --board sitl
./waf sub
```

Install MAVProxy:
```bash
pip install MAVProxy
```

---

## Step 5 - Install ardupilot_gazebo Plugin

This plugin bridges ArduPilot SITL with Gazebo Harmonic.

```bash
# Install build dependencies
sudo apt install -y libgz-sim8-dev rapidjson-dev

# Clone and build the plugin
cd ~
git clone https://github.com/ArduPilot/ardupilot_gazebo.git
cd ardupilot_gazebo
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
make -j4
```

---

## Step 6 - Clone This Repository

```bash
cd ~
git clone https://github.com/radshafeeq/AUV-Development.git
cd AUV-Development
```

---

## Step 7 - Set Up Environment Paths

This is the most important step. Gazebo needs to know WHERE to find the models, worlds,
and plugins. Without this, Gazebo will launch but show an empty world with no AUV.

Add these lines to your ~/.bashrc to make them permanent:

```bash
echo 'export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/auv_ws/firmware/ardupilot_gazebo/build:$GZ_SIM_SYSTEM_PLUGIN_PATH' >> ~/.bashrc
echo 'export GZ_SIM_RESOURCE_PATH=$HOME/auv_ws/firmware/ardupilot_gazebo/models:$HOME/auv_ws/firmware/ardupilot_gazebo/worlds:$GZ_SIM_RESOURCE_PATH' >> ~/.bashrc
echo 'export GZ_SIM_RESOURCE_PATH=$HOME/auv_ws/simulation/bluerov2_gz/models:$HOME/auv_ws/simulation/bluerov2_gz/worlds:$GZ_SIM_RESOURCE_PATH' >> ~/.bashrc
echo 'export GZ_SIM_RESOURCE_PATH=$HOME/auv_ws/simulation/my_robot_model/models:$GZ_SIM_RESOURCE_PATH' >> ~/.bashrc
source ~/.bashrc
```

---

## Step 8 - Add the Custom AUV Model to Gazebo

Copy the custom AUV model to Gazebo's local model directory:

```bash
mkdir -p ~/.gz/sim/models
cp -r ~/AUV-Development/my_robot_model/models/my_custom_auv ~/.gz/sim/models/
```

After completing all steps, go back to the **How to Run the Simulation** section at the top of this README to launch the simulation.

---

## Repository Structure

```
AUV-Development/
├── README.md                              <- This file
├── SIMULATION_REQUIREMENTS.md             <- Detailed simulation documentation
├── AUV_Kalman_Filter_Comprehensive_Explanation.md <- Comprehensive kinematic/dynamic & Kalman filter guide
├── kalman_filter.py                       <- 1D/2D Kalman Filter & Extended Kalman Filter (EKF) module
├── auv_yolo_tracking.py                   <- Real-time YOLO26 World tracking & MAVLink guidance node
├── train_yolo26.py                        <- YOLO26 custom training pipeline
├── weights/
│   ├── yolo26_world.pt                    <- Zero-shot open-vocabulary YOLO26 World model weights
│   └── yolo26n.pt                         <- Base YOLO26 neural network weights
├── start_gazebo.sh                        <- Quick Gazebo launch script (Custom 4-DOF AUV)
├── start_ardusub.sh                       <- Quick ArduSub launch script (Custom 4-DOF AUV)
├── start_bluerov2_heavy_gazebo.sh        <- Quick Gazebo launch script (BlueROV2 Heavy 8-Thruster)
├── start_bluerov2_heavy_ardusub.sh       <- Quick ArduSub launch script (BlueROV2 Heavy 8-Thruster)
├── start_cockpit.sh                       <- Quick Cockpit GCS launch script
│
├── my_robot_model/                        <- Custom AUV Gazebo model
│   └── models/my_custom_auv/
│       ├── model.config                   <- Model metadata
│       ├── model.sdf                      <- Full physics and visual model
│       └── meshes/
│           ├── The_Hull.obj               <- Custom AUV hull mesh
│           └── The_Hull_Rotated.obj       <- Hull mesh (rotated orientation)
│
├── bluerov2_gz/                           <- Base simulation framework
│   ├── CMakeLists.txt / package.xml       <- ROS 2 build files
│   ├── setup.bash                         <- Environment path setup
│   ├── models/                            <- BlueROV2 reference models and seabed
│   ├── worlds/                            <- Underwater Gazebo world files
│   ├── scripts/                           <- Thruster control shell scripts
│   └── params/                            <- Sensor parameter files
│
├── My_AUV_Simulation/                     <- Webots simulation
│   ├── worlds/
│   │   ├── auv_test_world.wbt             <- Webots world file
│   │   └── *.obj                          <- All propeller and hull meshes
│   └── controllers/
│       └── ardupilot_vehicle_controller/  <- ArduPilot controller (Python)
│
└── docs/
    └── SIMULATION_REQUIREMENTS.md         <- Thesis simulation documentation
```

---

## References

- Gazebo Harmonic Installation: https://gazebosim.org/docs/harmonic/install
- ardupilot_gazebo Plugin: https://github.com/ArduPilot/ardupilot_gazebo
- ArduSub Documentation: https://www.ardusub.com/
- ArduPilot SITL Setup: https://ardupilot.org/dev/docs/setting-up-sitl-on-linux.html
- MAVProxy Documentation: https://ardupilot.org/mavproxy/docs/getting_started/download_and_installation.html
- ROS 2 Jazzy Installation: https://docs.ros.org/en/jazzy/Installation.html
- BlueROV2 Gazebo base package: https://github.com/clydemcqueen/bluerov2_gz

---

---

# Adapting the BlueROV2 Simulation for a Custom AUV

This simulation was **not built from scratch**. It is based on an existing open-source
BlueROV2 Gazebo simulation package found on the internet, which was then adapted to use
a custom 3D hull model with modified physical properties to match the real-world AUV built
for this thesis.

## Origin of the Base Simulation

The base simulation package used is **bluerov2_gz** by Clyde McQueen:
- Source: https://github.com/clydemcqueen/bluerov2_gz
- It provides: underwater world files, buoyancy plugin, hydrodynamics plugin, thruster plugin, and the BlueROV2 Gazebo model

### Install the Original bluerov2_gz

```bash
# Create a ROS 2 colcon workspace (if you don't have one)
mkdir -p ~/colcon_ws/src
cd ~/colcon_ws/src

# Clone the original BlueROV2 simulation
git clone https://github.com/clydemcqueen/bluerov2_gz.git

# Build it
cd ~/colcon_ws
colcon build
source install/setup.bash
```

To run the **original unmodified** BlueROV2 simulation:
```bash
gz sim -v 3 -r bluerov2_underwater.world
```

---

## How the Custom AUV Model Was Created

Instead of using the default BlueROV2 3D model, a custom model folder was created at:

```
~/my_robot_model/models/my_custom_auv/
├── model.config        <- Model name and metadata
├── model.sdf           <- All physics, visual, and plugin definitions
└── meshes/
    └── The_Hull.obj    <- Custom 3D hull designed for this thesis
```

The `model.sdf` file is the core file that was edited. Open it with:

```bash
nano ~/my_robot_model/models/my_custom_auv/model.sdf
```

---

## What Was Changed in model.sdf

### 1. Visual Mesh — Replaced with Custom Hull

The original BlueROV2 `.dae` mesh was replaced with the custom `.obj` hull:

```xml
<visual name="base_link_visual">
  <pose>0 0 0 0 0 1.57</pose>
  <geometry>
    <mesh>
      <uri>model://my_custom_auv/meshes/The_Hull.obj</uri>
      <scale>0.01 0.01 0.01</scale>   <!-- OBJ was designed in cm, scaled to meters -->
    </mesh>
  </geometry>
</visual>
```

> The scale of 0.01 is required because the OBJ file was designed in centimetres,
> but Gazebo interprets all units as metres.

### 2. Collision Box — Simplified for Physics Stability

Instead of using the complex OBJ mesh for collision (which causes physics instability),
a simple box approximating the hull's bounding box was used:

```xml
<collision name="base_link_collision">
  <geometry>
    <box>
      <size>0.45 0.25 0.13</size>   <!-- Length x Width x Height in metres -->
    </box>
  </geometry>
</collision>
```

### 3. Mass and Inertia — Set to Match Real AUV

```xml
<inertial>
  <pose>0 0 -0.05 0 0 0</pose>     <!-- Center of mass slightly below center -->
  <mass>14.2</mass>                 <!-- Total mass in kg -->
  <inertia>
    <ixx>0.13</ixx>
    <iyy>0.13</iyy>
    <izz>0.13</izz>
  </inertia>
</inertial>
```

### 4. Buoyancy — Tuned for Neutral Buoyancy

The buoyancy volume was adjusted until the AUV achieved near-neutral buoyancy
(hovering without sinking or floating):

```xml
<plugin filename="gz-sim-buoyancy-system" name="gz::sim::systems::Buoyancy">
  <uniform_fluid_density>998</uniform_fluid_density>   <!-- Water density kg/m3 -->
  <link_name>base_link</link_name>
  <volume>0.011</volume>                               <!-- Displaced volume in m3 -->
  <center_of_volume>0 0 0.1</center_of_volume>         <!-- Slightly above CoM for stability -->
</plugin>
```

### 5. Thruster Positions — Repositioned for Custom Frame Geometry

The 6 thrusters were repositioned to match the custom AUV frame. Thrusters 1-4 are
the horizontal thrusters (for surge and yaw), and thrusters 5-6 are the vertical
thrusters (for heave):

```xml
<!-- Thruster 1 - Front Right Horizontal -->
<link name="thruster1">
  <pose>0.23 -0.147 0.0 -1.571 1.571 -0.785</pose>
</link>

<!-- Thruster 2 - Front Left Horizontal -->
<link name="thruster2">
  <pose>0.23 0.147 0.0 -1.571 1.571 -2.356</pose>
</link>

<!-- Thruster 3 - Rear Right Horizontal -->
<link name="thruster3">
  <pose>-0.23 -0.147 0.0 -1.571 1.571 0.785</pose>
</link>

<!-- Thruster 4 - Rear Left Horizontal -->
<link name="thruster4">
  <pose>-0.23 0.147 0.0 -1.571 1.571 2.356</pose>
</link>

<!-- Thruster 5 - Left Vertical -->
<link name="thruster5">
  <pose>0.0 -0.119 0.007 0 0 0</pose>
</link>

<!-- Thruster 6 - Right Vertical -->
<link name="thruster6">
  <pose>0.0 0.119 0.007 0 0 0</pose>
</link>
```

> Pose format is: `X Y Z Roll Pitch Yaw` (all in metres and radians)

### 6. Hydrodynamic Drag Coefficients

Drag coefficients were kept from the original BlueROV2 model as a reasonable
approximation for a similarly-sized AUV:

```xml
<xUabsU>-33.732</xUabsU>   <!-- Surge drag -->
<yVabsV>-54.16</yVabsV>    <!-- Sway drag -->
<zWabsW>-73.225</zWabsW>   <!-- Heave drag -->
```

---

## Summary of Adaptation Workflow

```
1. Clone bluerov2_gz (original BlueROV2 simulation)
         |
2. Create new model folder: my_robot_model/models/my_custom_auv/
         |
3. Edit model.sdf:
   - Replace visual mesh  --> custom The_Hull.obj
   - Adjust scale         --> 0.01 (cm to meters)
   - Set mass             --> 14.2 kg
   - Tune buoyancy volume --> 0.011 m3
   - Reposition thrusters --> match physical frame measurements
   - Simplify collision   --> box primitive for stability
         |
4. Set GZ_SIM_RESOURCE_PATH to include my_robot_model/models
         |
5. Reference model as "my_custom_auv" in the world file
         |
6. Launch and test simulation
```

---

## Ground Control Station (Cockpit)

To manually monitor the simulation or fly using a gamepad (Xbox/PlayStation controller), you can use the **Blue Robotics Cockpit** desktop application.

> **Note for Ubuntu 24.04 Users:** AppImages require a FUSE library that is no longer installed by default. Before running Cockpit for the first time, you must run:
> ```bash
> sudo apt update
> sudo apt install libfuse2t64
> ```

1.  **Start the simulation:**
    Run both `start_gazebo.sh` and `start_ardusub.sh` in separate terminals.
2.  **Run Cockpit:**
    You can launch Cockpit using the provided script in this repository:
    ```bash
    ./start_cockpit.sh
    ```
3.  Cockpit will automatically detect the MAVLink stream on `localhost:14550`. You can map your controller in the UI and test the AUV physics before autonomous AI testing.

