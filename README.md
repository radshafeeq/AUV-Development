# Custom AUV Development - Gazebo Harmonic + ArduSub + Webots

> **Undergraduate Thesis Project** — Hasanuddin University  
> A custom 5-DOF, 6-thruster Autonomous Underwater Vehicle (AUV) simulation  
> built on top of the BlueROV2 Gazebo framework, integrated with ArduSub firmware.

---

## Table of Contents
1. [Project Overview](#project-overview)
2. [System Requirements](#system-requirements)
3. [Step 1 - Install Ubuntu 24.04](#step-1---install-ubuntu-2404)
4. [Step 2 - Install ROS 2 Jazzy](#step-2---install-ros-2-jazzy)
5. [Step 3 - Install Gazebo Harmonic](#step-3---install-gazebo-harmonic)
6. [Step 4 - Install ArduPilot and ArduSub](#step-4---install-ardupilot-and-ardusub)
7. [Step 5 - Install ardupilot_gazebo Plugin](#step-5---install-ardupilot_gazebo-plugin)
8. [Step 6 - Clone This Repository](#step-6---clone-this-repository)
9. [Step 7 - Set Up Environment Paths](#step-7---set-up-environment-paths)
10. [Step 8 - Add the Custom AUV Model to Gazebo](#step-8---add-the-custom-auv-model-to-gazebo)
11. [Running the Simulation](#running-the-simulation)
12. [Thruster Control Commands](#thruster-control-commands)
13. [Repository Structure](#repository-structure)
14. [References](#references)

---

## Project Overview

This repository contains:
- **Custom AUV Gazebo model** (`my_custom_auv`) - a 6-thruster AUV with custom OBJ hull mesh, physically tuned for underwater simulation
- **BlueROV2 Gazebo framework** (`bluerov2_gz`) - the base simulation package providing buoyancy, hydrodynamics, and thruster plugins
- **Webots simulation** (`My_AUV_Simulation`) - an alternative simulation environment with ArduPilot vehicle controller
- **Thruster control scripts** - shell scripts for sending direct thrust commands via Gazebo topics

---

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

Run these commands before launching the simulation:

```bash
# Tell Gazebo where the ardupilot_gazebo plugin is
export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/ardupilot_gazebo/build:$GZ_SIM_SYSTEM_PLUGIN_PATH

# Tell Gazebo where ardupilot_gazebo models and worlds are
export GZ_SIM_RESOURCE_PATH=$HOME/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds:$GZ_SIM_RESOURCE_PATH

# Tell Gazebo where bluerov2_gz models and worlds are
export GZ_SIM_RESOURCE_PATH=$HOME/AUV-Development/bluerov2_gz/models:$HOME/AUV-Development/bluerov2_gz/worlds:$GZ_SIM_RESOURCE_PATH

# Tell Gazebo where the custom AUV model is
export GZ_SIM_RESOURCE_PATH=$HOME/AUV-Development/my_robot_model/models:$GZ_SIM_RESOURCE_PATH
```

**Tip:** To make these permanent (run automatically every time you open a terminal),
add them to your ~/.bashrc file:

```bash
echo 'export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/ardupilot_gazebo/build:$GZ_SIM_SYSTEM_PLUGIN_PATH' >> ~/.bashrc
echo 'export GZ_SIM_RESOURCE_PATH=$HOME/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds:$GZ_SIM_RESOURCE_PATH' >> ~/.bashrc
echo 'export GZ_SIM_RESOURCE_PATH=$HOME/AUV-Development/bluerov2_gz/models:$HOME/AUV-Development/bluerov2_gz/worlds:$GZ_SIM_RESOURCE_PATH' >> ~/.bashrc
echo 'export GZ_SIM_RESOURCE_PATH=$HOME/AUV-Development/my_robot_model/models:$GZ_SIM_RESOURCE_PATH' >> ~/.bashrc
source ~/.bashrc
```

---

## Step 8 - Add the Custom AUV Model to Gazebo

Copy the custom AUV model to Gazebo's local model directory so it appears in the model browser:

```bash
mkdir -p ~/.gz/sim/models
cp -r ~/AUV-Development/my_robot_model/models/my_custom_auv ~/.gz/sim/models/
```

---

## Running the Simulation

You need TWO separate terminals running at the same time.

### Terminal 1 - Launch Gazebo

```bash
gz sim -v 3 -r bluerov2_underwater.world
```

Available world files:

| World File | Description |
|---|---|
| `bluerov2_underwater.world` | BlueROV2 base configuration (underwater) |
| `bluerov2_heavy_underwater.world` | BlueROV2 Heavy configuration (underwater) |
| `bluerov2_ping.world` | BlueROV2 with Ping sonar |

### Terminal 2 - Launch ArduSub SITL

Open a new terminal and run:

```bash
cd ~/ardupilot
Tools/autotest/sim_vehicle.py -L RATBeach -v ArduSub -f vectored \
  --model=JSON --out=udp:0.0.0.0:14550 --console
```

> Replace `-f vectored` with `-f vectored_6dof` for the Heavy configuration.
> Add `-w` flag if switching between frames to reset ArduSub parameters.

### Terminal 3 - Send Commands via MAVProxy (Optional)

Once ArduSub is running, control the AUV:

```bash
arm throttle          # Arm the vehicle
mode alt_hold         # Switch to altitude hold mode
rc 3 1450             # Descend
rc 3 1500             # Stop vertical movement
rc 5 1550             # Move forward
disarm                # Disarm
```

---

## Thruster Control Commands

Send direct thrust commands to Gazebo topics (bypasses ArduSub):

```bash
cd ~/AUV-Development/bluerov2_gz

scripts/forward.sh my_custom_auv    # Move forward
scripts/cw.sh my_custom_auv         # Rotate clockwise
scripts/ccw.sh my_custom_auv        # Rotate counter-clockwise
scripts/up.sh my_custom_auv         # Move up
scripts/down.sh my_custom_auv       # Move down
scripts/stop.sh my_custom_auv       # Stop all thrusters
```

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
