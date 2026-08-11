# Custom 5-DOF AUV Simulation
This repository contains the simulation files for a custom 6-motor, 5-DOF Autonomous Underwater Vehicle (AUV). This project is being developed as part of an undergraduate Mechatronics Engineering thesis at Hasanuddin University.

The simulation integrates a custom 3D AUV frame design with Gazebo Harmonic (for physics and 3D rendering) and ArduSub SITL (for the flight controller and vehicle dynamics).

## Architecture Overview
The simulation is split into two main components that communicate over a local network:

**Gazebo Sim:** Handles the 3D graphics, water physics, buoyancy, and collisions using the custom 3D .obj models.

**ArduSub SITL (Software In The Loop):** Acts as the "brain" of the robot. It runs the ArduSub firmware, calculates motor thrusts based on the 6-motor geometry, and provides a MAVLink interface via MAVProxy.

## How to Run the Simulation
You will need to open two separate terminal windows to launch both the physics environment and the flight controller.

### 1. Launch the 3D Physics Environment (Gazebo)
Open a terminal and run this command. It sets the required paths so Gazebo can locate the custom 3D AUV models before launching the world.

```bash
export GZ_SIM_RESOURCE_PATH=/home/radhi/auv_ws/simulation/my_robot_model/models:/home/radhi/auv_ws/simulation/bluerov2_gz/models:/home/radhi/auv_ws/simulation/bluerov2_gz/worlds:$GZ_SIM_RESOURCE_PATH
export GZ_SIM_SYSTEM_PLUGIN_PATH=/home/radhi/auv_ws/firmware/ardupilot_gazebo/build:$GZ_SIM_SYSTEM_PLUGIN_PATH
gz sim -v 4 -r /home/radhi/auv_ws/simulation/bluerov2_gz/worlds/bluerov2_underwater.world
```

### 2. Launch the ArduSub Flight Controller
Open a second terminal and run this command to start the SITL firmware and open the MAVProxy console:

```bash
cd ~/auv_ws/firmware/ardupilot/ArduSub
python3 ~/auv_ws/firmware/ardupilot/Tools/autotest/sim_vehicle.py -v ArduSub -f gazebo-bluerov2 --model JSON --console
```

## Basic Control Commands
Once the ArduSub terminal is running, you will see a `MANUAL>` prompt. You can use the following MAVLink commands to control the AUV:

- **Arm the thrusters:** `arm throttle`
- **Change flight mode:** `mode alt_hold` (Depth hold mode)
- **Apply vertical thrust (heave):** `rc 3 1600` (1500 is neutral/stop, 1600 is up, 1400 is down)
- **Stop thrust:** `rc 3 1500`
- **Disarm the AUV:** `disarm`

---

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
├── start_gazebo.sh                        <- Quick Gazebo launch script
├── start_ardusub.sh                       <- Quick ArduSub launch script
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
