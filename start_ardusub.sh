#!/bin/bash
cd ~/ardupilot/ArduSub
python3 ~/ardupilot/Tools/autotest/sim_vehicle.py -v ArduSub -f gazebo-bluerov2 --model JSON --console
