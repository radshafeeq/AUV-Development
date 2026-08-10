#!/bin/bash
export GZ_SIM_RESOURCE_PATH=/home/radhi/auv_ws/simulation/my_robot_model/models:/home/radhi/auv_ws/simulation/bluerov2_gz/models:/home/radhi/auv_ws/simulation/bluerov2_gz/worlds:$GZ_SIM_RESOURCE_PATH
export GZ_SIM_SYSTEM_PLUGIN_PATH=/home/radhi/auv_ws/firmware/ardupilot_gazebo/build:$GZ_SIM_SYSTEM_PLUGIN_PATH
gz sim -v 4 -r /home/radhi/auv_ws/simulation/bluerov2_gz/worlds/bluerov2_underwater.world
