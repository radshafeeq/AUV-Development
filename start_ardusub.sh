#!/bin/bash
cd ~/auv_ws/firmware/ardupilot/ArduSub
python3 ~/auv_ws/firmware/ardupilot/Tools/autotest/sim_vehicle.py -v ArduSub -f gazebo-bluerov2 --model JSON --console
