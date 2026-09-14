#!/bin/bash
# BlueROV2 Heavy (8-Thruster / 4-DOF) ArduSub SITL Launch Script

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
cd ~/auv_ws/firmware/ardupilot/ArduSub

LOCAL_IP=$(hostname -I | awk '{print $1}')

echo "Starting ArduSub SITL (BlueROV2 Heavy - 8 Thrusters / vectored_6dof)..."
python3 ~/auv_ws/firmware/ardupilot/Tools/autotest/sim_vehicle.py -v ArduSub -f vectored_6dof --model JSON -w --console --out udp:127.0.0.1:14550 --out udp:${LOCAL_IP}:14550 --add-param-file="${SCRIPT_DIR}/joystick_fix.param"
