#!/bin/bash
export GZ_SIM_RESOURCE_PATH=/home/radhi/my_robot_model/models:/home/radhi/bluerov2_gz/models:/home/radhi/bluerov2_gz/worlds:$GZ_SIM_RESOURCE_PATH
export GZ_SIM_SYSTEM_PLUGIN_PATH=/home/radhi/ardupilot_gazebo/build:$GZ_SIM_SYSTEM_PLUGIN_PATH
gz sim -v 4 -r /home/radhi/bluerov2_gz/worlds/bluerov2_underwater.world
