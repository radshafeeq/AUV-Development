# AUV Simulation Requirements & Installation Guide

This document records all software requirements, sources, and installation
instructions for the custom 5-DOF, 6-motor AUV simulation environment.

**Developed by:** Radhi Shafeeq  
**University:** Hasanuddin University (Mechatronics Engineering)  
**Purpose:** Undergraduate Thesis - Custom AUV Development  

> **Note on Simulation Approach:** This simulation was NOT built from scratch.
> It is based on an existing open-source BlueROV2 Gazebo simulation framework
> that was found online and adapted for the custom AUV design. The base simulation
> framework (bluerov2_gz) was modified to replace the original BlueROV2 3D model
> with a custom AUV model that has different frame dimensions, thruster sizes,
> and thruster positions to match the physical hardware being developed.

---

## System Environment

| Component        | Details                             |
|-----------------|-------------------------------------|
| Operating System | Ubuntu 24.04.4 LTS (Noble Numbat)  |
| Python Version   | 3.12.3                              |
| Architecture     | x86_64                              |

---

## Software Stack Overview

```
Your Laptop (Ubuntu 24.04)
        ↕  Ethernet Tether (192.168.2.2)
Raspberry Pi 4B  →  BlueOS
    + 5MP Camera Module
        ↕  USB Serial
Pixhawk 2.4.8  →  ArduSub Firmware
        ↕  PWM Signal
6x ESCs → 6x Thrusters (T200)
```

---

## 1. ROS 2 Jazzy Jalisco

**Version:** ROS 2 Jazzy (LTS, compatible with Ubuntu 24.04)  
**Source:** https://docs.ros.org/en/jazzy/Installation.html

### Installation:

```bash
# Set locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# Add ROS 2 repository
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu noble main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Jazzy
sudo apt update
sudo apt install ros-jazzy-desktop

# Activate ROS 2 (add to ~/.bashrc)
source /opt/ros/jazzy/setup.bash
```

---

## 2. Gazebo Harmonic (gz-sim)

**Version:** Gazebo Sim 8.14.0 (Harmonic)  
**Source:** https://gazebosim.org/docs/harmonic/install_ubuntu

### Installation:

```bash
# Add Gazebo repository
sudo apt-get update
sudo apt-get install lsb-release wget gnupg

sudo wget https://packages.osrfoundation.org/gazebo.gpg \
  -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=amd64 signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
  http://packages.osrfoundation.org/gazebo/ubuntu-stable noble main" \
  | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# Install Gazebo Harmonic
sudo apt-get update
sudo apt-get install gz-harmonic
```

---

## 3. ArduPilot / ArduSub (SITL)

**Version:** ArduPilot-4.6.0-beta1 (commit: g0b42690c43)  
**Source:** https://github.com/ArduPilot/ardupilot  
**Documentation:** https://ardupilot.org/dev/docs/building-setup-linux.html

### Installation:

```bash
# Clone the repository
cd ~
git clone https://github.com/ArduPilot/ardupilot.git
cd ardupilot
git submodule update --init --recursive

# Install dependencies
Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile

# Create Python virtual environment
python3 -m venv ~/venv-ardupilot
source ~/venv-ardupilot/bin/activate

# Install Python dependencies
pip install MAVProxy pymavlink matplotlib
```

---

## 4. ardupilot_gazebo Plugin

**Version:** Commit 50e62a5  
**Source:** https://github.com/ArduPilot/ardupilot_gazebo  
**Purpose:** Connects ArduSub SITL to Gazebo physics simulation via MAVLink.
Provides Buoyancy, Hydrodynamics, and Thruster plugins.

### Installation:

```bash
# Install dependencies
sudo apt install libgz-sim8-dev rapidjson-dev
sudo apt install libopencv-dev libgstreamer1.0-dev \
  libgstreamer-plugins-base1.0-dev gstreamer1.0-plugins-bad \
  gstreamer1.0-libav gstreamer1.0-gl

# Clone and build
cd ~
git clone https://github.com/ArduPilot/ardupilot_gazebo.git
cd ardupilot_gazebo
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
make -j4
```

---

## 5. bluerov2_gz (BlueROV2 Gazebo Package)

**Version:** Commit 661264b  
**Source:** https://github.com/clydemcqueen/bluerov2_gz  
**Purpose:** Provides the BlueROV2 Gazebo world file and underwater simulation
environment including water physics, buoyancy, and the custom AUV model world.

### Installation:

```bash
cd ~
git clone https://github.com/clydemcqueen/bluerov2_gz.git
```

---

## 6. MAVProxy

**Version:** 2.4.49  
**Source:** https://ardupilot.org/mavproxy/  
**Purpose:** Ground control station and MAVLink terminal for sending commands
to ArduSub (arm, mode switching, RC overrides, etc.).

### Installation (inside ArduPilot virtual environment):

```bash
source ~/venv-ardupilot/bin/activate
pip install MAVProxy
```

---

## 7. pymavlink

**Version:** 2.4.49  
**Source:** https://github.com/ArduPilot/pymavlink  
**Purpose:** Python library for sending and receiving MAVLink messages.
Will be used for the autonomous detection and control pipeline.

### Installation:

```bash
source ~/venv-ardupilot/bin/activate
pip install pymavlink
```

---

## Simulation Source & Adaptation

### Where the Simulation Was Found

The simulation was NOT built from scratch. The base simulation was found from
the following open-source repository:

**Repository:** bluerov2_gz by Clyde McQueen  
**URL:** https://github.com/clydemcqueen/bluerov2_gz  
**License:** MIT License (open-source, free to adapt)  

### How the Simulation Was Adapted

1. **Cloned the repository** to the local machine:
   ```bash
   cd ~
   git clone https://github.com/clydemcqueen/bluerov2_gz.git
   ```

2. **Created a new custom model folder** at:
   ```
   /home/radhi/my_robot_model/models/my_custom_auv/
   ```

3. **Replaced the original BlueROV2 3D mesh files** (`.dae`) with the
   custom-designed hull and propeller models (`.obj`).

4. **Modified the `model.sdf` file** to match the custom AUV frame:
   - Updated collision box dimensions.
   - Updated thruster positions and orientations.
   - Updated propeller scale to match custom propeller size.

5. **Updated the world file** (`bluerov2_underwater.world`) to load
   `my_custom_auv` instead of the original `bluerov2` model.

---

## Custom Model & Modifications

**Base Simulation:** bluerov2_gz (https://github.com/clydemcqueen/bluerov2_gz)  
**Original Model:** Default BlueROV2 frame by Blue Robotics  
**Custom Package Name:** my_custom_auv  
**Custom Model Location:** /home/radhi/my_robot_model/models/  

### Important Disclaimer: Simulation vs. Real-World Measurements

> **The simulation model does NOT use exact real-world measurements.**
>
> Multiple geometry and parameter adjustments were made during development
> to ensure the simulation runs **stably and correctly** inside Gazebo Harmonic.
> These compromises are a normal and accepted part of simulation engineering
> and are referred to as **"simulation fidelity trade-offs"**.
>
> Common reasons for adjustments include:
> - Collision box simplification (complex 3D mesh shapes are approximated
>   with simple boxes/cylinders for physics stability).
> - Scale differences between 3D design software (CAD) and the Gazebo
>   simulation engine coordinate system.
> - Thruster position fine-tuning to achieve stable 5-DOF motion in the
>   physics engine.
> - Buoyancy and hydrodynamics parameter tuning for realistic underwater
>   behaviour.
>
> The measurements below represent the **simulation model parameters**,
> which are close approximations of the intended real-world design but
> are not guaranteed to be exact.

### What Was Modified: Measurement Comparison

> **Note on Scaling:** When the custom OBJ file was first loaded into Gazebo,
> it appeared extremely large because Gazebo interprets all units as **meters**,
> but the OBJ vertices were designed in **centimeters**. A scale factor of
> `0.01` was applied in the SDF file to correctly convert cm → meters.
> Even after scaling, the physics **collision box** had to be manually
> simplified to a smaller approximate shape to achieve stable simulation.
> This is why two different sets of numbers exist for the simulation below.

#### 1. Hull / Frame Dimensions

| Version | Width | Height | Length | Notes |
|---|---|---|---|---|
| **Real-World Design (OBJ file)** | **45.15 cm** | **57.70 cm** | **11.97 cm** | Intended physical size to be 3D printed |
| **Simulation Visual Mesh** | **45.15 cm** | **57.70 cm** | **11.97 cm** | ✅ Successfully matches real-world design! |
| **Simulation Collision Box** | 45.0 cm | 25.0 cm | 13.0 cm | ⚠️ Simplified for physics stability only |

> ✅ **Achievement:** The simulation visual mesh dimensions successfully match
> the intended real-world physical design. By applying a scale factor of `0.01`
> in the SDF file, the OBJ vertices (designed in cm) are correctly interpreted
> as meters in Gazebo, producing a simulation model that is **true-to-size**.
>
> ⚠️ **Compromise:** The physics collision box (used for buoyancy and collision
> calculations) is simplified to `45cm × 25cm × 13cm`. The height (25.0 cm)
> is smaller than the real hull height (57.70 cm). This was a necessary
> trade-off to prevent physics instability in the Gazebo simulation.

| Dimension   | Original BlueROV2 (from SDF) | Custom AUV (from OBJ file)  | Difference   |
|-------------|------------------------------|-----------------------------|--------------|
| **Width**   | 45.7 cm                      | **45.15 cm**                | -0.55 cm     |
| **Height**  | 33.8 cm                      | **57.70 cm**                | +23.9 cm     |
| **Length**  | 6.5 cm                       | **11.97 cm**                | +5.47 cm     |

#### 2. Propeller Dimensions

| Property         | Original T200 Prop (BlueROV2) | Custom Propeller          | Difference    |
|------------------|-------------------------------|---------------------------|---------------|
| **Diameter**     | ~32.8 mm (3.28 cm)            | **66.9 mm (6.69 cm)**     | +34.1 mm      |
| **Thickness**    | N/A                           | **34.0 mm (3.40 cm)**     | N/A           |

#### 3. Thruster Positions (from model.sdf, in meters)

| Thruster | X (Forward) | Y (Lateral) | Z (Vertical) | Orientation (Roll, Pitch, Yaw)  |
|----------|-------------|-------------|--------------|----------------------------------|
| T1 (FR)  | +0.23 m     | -0.147 m    | 0.0 m        | -90°, 90°, -45° (angled)        |
| T2 (FL)  | +0.23 m     | +0.147 m    | 0.0 m        | -90°, 90°, -135° (angled)       |
| T3 (RR)  | -0.23 m     | -0.147 m    | 0.0 m        | -90°, 90°, +45° (angled)        |
| T4 (RL)  | -0.23 m     | +0.147 m    | 0.0 m        | -90°, 90°, +135° (angled)       |
| T5 (R)   | 0.0 m       | -0.119 m    | +0.007 m     | 0°, 0°, 0° (vertical)           |
| T6 (L)   | 0.0 m       | +0.119 m    | +0.007 m     | 0°, 0°, 0° (vertical)           |

> **Academic Note:** Adapting an existing simulation platform is standard
> engineering practice. The key original contributions of this thesis are:
> (1) the custom frame and thruster configuration design, (2) the Euler
> Equations of Motion derivation for this specific configuration, and
> (3) the integration of autonomous detection capabilities.

---

## Running the Simulation

### Terminal 1 - Launch Gazebo (3D Physics Environment):

```bash
export GZ_SIM_RESOURCE_PATH=/home/radhi/my_robot_model/models:/home/radhi/bluerov2_gz/models:/home/radhi/bluerov2_gz/worlds:$GZ_SIM_RESOURCE_PATH
export GZ_SIM_SYSTEM_PLUGIN_PATH=/home/radhi/ardupilot_gazebo/build:$GZ_SIM_SYSTEM_PLUGIN_PATH
gz sim -v 4 -r /home/radhi/bluerov2_gz/worlds/bluerov2_underwater.world
```

### Terminal 2 - Launch ArduSub SITL (Flight Controller and MAVProxy Console):

```bash
cd ~/ardupilot/ArduSub
python3 ~/ardupilot/Tools/autotest/sim_vehicle.py -v ArduSub -f gazebo-bluerov2 --model JSON --console
```

### Basic MAVProxy Control Commands (type at the MANUAL> prompt):

```
arm throttle          # Arm the thrusters
mode alt_hold         # Switch to depth hold mode
rc 3 1600             # Apply upward thrust (heave)
rc 3 1500             # Stop thrust (neutral)
disarm                # Disarm the AUV
```

---

## Future Requirements (Planned)

The following packages will be required for the autonomous detection pipeline:

| Package        | Purpose                          | Install Command             |
|----------------|----------------------------------|-----------------------------|
| ultralytics    | YOLOv8 object detection         | pip install ultralytics     |
| opencv-python  | Computer vision / camera stream | pip install opencv-python   |
| pymavlink      | MAVLink command interface       | Already installed (above)   |

---

*Last updated: 2026-08-08*  
*Generated with Gemini Antigravity AI Coding Assistant*


# AUV Simulation Setup & Troubleshooting Guide

This document serves as a comprehensive record of the setup steps, physics tuning, and software configurations we applied to successfully connect the Gazebo AUV simulation with ArduSub SITL and the BlueRobotics Cockpit ground control station.

## 1. Running Cockpit (AppImage Dependencies)
BlueRobotics Cockpit is distributed as an `.AppImage` file. On newer Linux distributions (like Ubuntu 22.04+), the `fuse2` library is deprecated by default but is required to run AppImages. 
* **Fix:** Installed `libfuse2` to allow Cockpit to execute.
  ```bash
  sudo apt update && sudo apt install libfuse2
  ```

## 2. Bridging ArduSub SITL and Cockpit
By default, the ArduSub SITL (Software In The Loop) outputs MAVLink data via raw UDP on port `14550`. However, the Cockpit application expects to communicate with a companion computer via a REST API/WebSocket on port `6040`.
* **Fix:** We installed **`mavlink2rest`**, a tool that bridges raw MAVLink UDP traffic to a REST API.
* **Execution:** Run the bridge alongside the simulation:
  ```bash
  ./mavlink2rest -s 0.0.0.0:6040
  ```

## 3. Fixing the Gazebo "Blank Screen" (Zombie Processes)
When closing the simulation using `Ctrl+C`, the background Gazebo (`gz sim`) server processes would occasionally fail to terminate. Relaunching the simulation would result in a blank, unresponsive white screen because the old server was hogging the ports.
* **Fix:** We injected cleanup commands into `start_gazebo.sh` to forcefully kill any lingering Gazebo or Ruby processes before launching a new instance:
  ```bash
  killall -9 ruby gz 2>/dev/null
  pkill -9 -f "gz sim" 2>/dev/null
  ```

## 4. Hydrodynamic Tuning (Achieving Neutral Buoyancy)
When spawned, the custom AUV model was immediately sinking to the bottom of the ocean. 
* **Fix:** We calculated the exact mass of the vehicle and tuned the `<volume>` parameter in the Gazebo `model.sdf` file. 
* By setting the volume to `0.014228` (with a water density of `998.0` kg/m³), the buoyant force exactly matched the gravitational force, resulting in perfect neutral buoyancy.

## 5. Thruster Kinematics & Sinking on Arm
When the vehicle was armed and throttle was applied, the AUV would violently flip and dive into the floor.
* **Root Cause:** In the `model.sdf`, the Clockwise (CW) propellers were assigned a negative thrust coefficient (`-0.02`), while Counter-Clockwise (CCW) propellers were positive (`0.02`). 
* **Fix:** The ArduSub flight controller's internal motor mixer already handles the math for CW vs CCW propeller directions. Therefore, sending a "forward" PWM signal to Gazebo must *always* result in a positive thrust vector. We replaced all `-0.02` values in `model.sdf` with `0.02`, ensuring that all vertical thrusters push water in the same direction when ascending.

## 6. Cockpit Joystick Configuration
Even when perfectly buoyant, the AUV would immediately dive the moment the joystick was enabled.
* **Root Cause:** The 8BitDo gamepad had unassigned mappings. In MAVLink, the Z-axis (Throttle) uses a `0 to 1000` scale, where `500` is neutral hover. Unmapped or trigger-mapped axes default to `0` when resting, which ArduSub interprets as a "Full Dive" command.
* **Fix:** 
  1. Opened the Cockpit **Joystick** menu.
  2. Clicked the **Restore Defaults** icon to inject the standard BlueROV2 thumbstick mappings.
  3. Ran the **CALIBRATE** wizard to ensure the sticks rested precisely at `500` (neutral).
