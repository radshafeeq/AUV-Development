# Custom 5-DOF AUV Simulation

This repository contains the simulation files for a custom 6-motor, 5-DOF Autonomous Underwater Vehicle (AUV). This project is being developed as part of an undergraduate Mechatronics Engineering thesis at Hasanuddin University.

The simulation integrates a custom 3D AUV frame design with **Gazebo Harmonic** (for physics and 3D rendering) and **ArduSub SITL** (for the flight controller and vehicle dynamics).

## Architecture Overview

The simulation is split into two main components that communicate over a local network:
1. **Gazebo Sim:** Handles the 3D graphics, water physics, buoyancy, and collisions using the custom 3D `.obj` models.
2. **ArduSub SITL (Software In The Loop):** Acts as the "brain" of the robot. It runs the ArduSub firmware, calculates motor thrusts based on the 6-motor geometry, and provides a MAVLink interface via MAVProxy.

## How to Run the Simulation

You will need to open two separate terminal windows to launch both the physics environment and the flight controller.

### 1. Launch the 3D Physics Environment (Gazebo)
Open a terminal and run this command. It sets the required paths so Gazebo can locate the custom 3D AUV models before launching the world.

```bash
export GZ_SIM_RESOURCE_PATH=/home/radhi/my_robot_model/models:/home/radhi/bluerov2_gz/models:/home/radhi/bluerov2_gz/worlds:$GZ_SIM_RESOURCE_PATH
export GZ_SIM_SYSTEM_PLUGIN_PATH=/home/radhi/ardupilot_gazebo/build:$GZ_SIM_SYSTEM_PLUGIN_PATH
gz sim -v 4 -r /home/radhi/bluerov2_gz/worlds/bluerov2_underwater.world

###2 2. Launch the ArduSub Flight Controller
Open a second terminal and run this command to start the SITL firmware and open the MAVProxy console:
cd ~/ardupilot/ArduSub
python3 ~/ardupilot/Tools/autotest/sim_vehicle.py -v ArduSub -f gazebo-bluerov2 --model JSON --console

Basic Control Commands
Once the ArduSub terminal is running, you will see a MANUAL> prompt. You can use the following MAVLink commands to control the AUV:

Arm the thrusters: arm throttle
Change flight mode: mode alt_hold (Depth hold mode)
Apply vertical thrust (heave): rc 3 1600 (1500 is neutral/stop, 1600 is up, 1400 is down)
Stop thrust: rc 3 1500
Disarm the AUV: disarm
