#!/bin/bash
# BlueROV2 Heavy (8-Thruster / 6-DOF) Gazebo Launch Script

# Environment paths for Gazebo Harmonic plugins and models
export GZ_SIM_RESOURCE_PATH=/home/radhi/auv_ws/simulation/my_robot_model/models:/home/radhi/auv_ws/simulation/bluerov2_gz/models:/home/radhi/auv_ws/simulation/bluerov2_gz/worlds:$GZ_SIM_RESOURCE_PATH
export GZ_SIM_SYSTEM_PLUGIN_PATH=/home/radhi/auv_ws/firmware/ardupilot_gazebo/build:$GZ_SIM_SYSTEM_PLUGIN_PATH

# Automatically kill any zombie Gazebo processes to prevent black screen / empty world
killall -9 ruby gz 2>/dev/null || true
pkill -9 -f "gz sim" 2>/dev/null || true

echo "Starting Gazebo Harmonic (BlueROV2 Heavy - 8 Thrusters)..."
gz sim -v 4 -r /home/radhi/auv_ws/simulation/bluerov2_gz/worlds/bluerov2_heavy_underwater.world
