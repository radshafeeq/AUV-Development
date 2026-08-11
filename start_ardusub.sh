#!/bin/bash
cd ~/auv_ws/firmware/ardupilot/ArduSub
LOCAL_IP=$(hostname -I | awk '{print $1}')
python3 ~/auv_ws/firmware/ardupilot/Tools/autotest/sim_vehicle.py -v ArduSub -f gazebo-bluerov2 --model JSON --console --out udp:127.0.0.1:14550 --out udp:${LOCAL_IP}:14550
