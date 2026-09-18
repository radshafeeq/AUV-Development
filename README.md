# Custom 4-DOF AUV Simulation
This repository contains the simulation files for a custom 6-motor, 4-DOF Autonomous Underwater Vehicle (AUV). This project is being developed as part of an undergraduate Mechatronics Engineering thesis at Hasanuddin University.

The simulation integrates a custom 3D AUV frame design with Gazebo Harmonic (for physics and 3D rendering) and ArduSub SITL (for the flight controller and vehicle dynamics).

## Architecture Overview
The simulation is split into two main components that communicate over a local network:

**Gazebo Sim:** Handles the 3D graphics, water physics, buoyancy, and collisions using the custom 3D .obj models.

**ArduSub SITL (Software In The Loop):** Acts as the "brain" of the robot. It runs the ArduSub firmware, calculates motor thrusts based on the 6-motor geometry, and provides a MAVLink interface via MAVProxy.

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

### 📈 4D Kalman Filter State Estimation & Trajectory Prediction

The AI tracking pipeline incorporates a 4-state Constant-Velocity **Kalman Filter** ([`kalman_filter.py`](file:///home/radhi/Documents/AUV_GitHub_Upload/kalman_filter.py)) to optimize AUV motion control:

#### 🧠 What the Kalman Filter Does for the AUV:
1. **Thruster Smoothing & Noise Elimination**:
   - Raw vision bounding boxes jitter due to frame noise. The Kalman Filter smooths target coordinates $(x, y)$ before feeding errors to the $K_p$ controller, preventing thrusters from jerking violently.
2. **Water Occlusion & Missing Frame Recovery**:
   - In turbid river/lake water, bubbles, or light glare, YOLO may lose detection for a few frames. The Kalman Filter **predicts where the target is moving** for up to 15 frames (~0.5s), allowing the AUV to keep tracking seamlessly.
3. **Velocity Estimation**:
   - Estimates real-time target velocity $(v_x, v_y)$ for predictive steering.

#### 📐 State Space Model:
- **State Vector**: $\mathbf{x}_k = [x, y, v_x, v_y]^T$ *(Position + Velocity)*
- **Measurement**: $\mathbf{z}_k = [z_x, z_y]^T$ *(Raw YOLO center)*
- **Module**: Implemented using OpenCV & NumPy in [`kalman_filter.py`](file:///home/radhi/Documents/AUV_GitHub_Upload/kalman_filter.py) and integrated into [`auv_yolo_tracking.py`](file:///home/radhi/Documents/AUV_GitHub_Upload/auv_yolo_tracking.py).

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

This repository includes a high-performance **Real-Time AI Computer Vision & Autonomous Tracking Node** powered by **YOLO26 World** open-vocabulary zero-shot detection and a **4D Constant-Velocity Kalman Filter**, designed for topside GPU acceleration (NVIDIA RTX 4070 Laptop GPU) connected to the AUV via BlueOS / Ethernet tether.

## Overview & Architecture
- **Camera Feed**: Zero-latency UDP 5600 RTP H.264 stream or RTSP stream (`rtsp://192.168.2.2:8554/video_udp_stream_0`) from BlueOS 1.4.5 on Raspberry Pi 4B.
- **Zero-Latency Video Receiver**: Built with GStreamer pipeline to eliminate network video delay, running synchronously at 30 FPS.
- **AI Inference Engine**: Ultralytics **YOLO26 World** (`weights/yolo26_world.pt`) running on NVIDIA RTX 4070 GPU (CUDA). Employs open-vocabulary text embeddings for instant zero-shot recognition of mechatronics hardware (Pixhawk, BLDC motors, ESCs, batteries, cables), desktop gadgets (smartphones, mice, keyboards, laptops), and underwater targets (buoys, gates, pipes).
- **Target Tracking & State Estimation**: **4D Constant-Velocity Kalman Filter** ($[x, y, v_x, v_y]^T$) with measurement noise covariance tuned for pixel-scale bounding box jitter reduction, smooth trajectory projection, and dead-reckoning during temporary target occlusions.
- **Visual Servo Guidance**: Calculates normalized tracking errors ($e_x, e_y \in [-1, +1]$) from frame center and transmits PyMAVLink `MANUAL_CONTROL` yaw and heave commands to ArduSub.

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
| **`r`** | Rotate 90° CW | Cycles camera orientation (0° -> 90° CW -> 180° -> 270° CW) |
| **`f`** | Flip 180° | Toggles 180° upside-down flip |
| **`+` / `=`** | Confidence Up | Increases detection confidence threshold (+0.02) |
| **`-` / `_`** | Confidence Down | Decreases detection confidence threshold (-0.02) |
| **`q`** | Exit | Closes camera stream and shuts down cleanly |

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

