#!/bin/bash
# Blue Robotics Cockpit & mavlink2rest Bridge Launch Script

# 1. Start mavlink2rest bridge (UDP 14550 -> REST/WebSocket port 6040)
if ! pgrep -f "mavlink2rest" > /dev/null; then
    echo "Starting mavlink2rest bridge on port 6040..."
    /home/radhi/mavlink2rest -c udpin:0.0.0.0:14550 -s 0.0.0.0:6040 &
    sleep 1
else
    echo "mavlink2rest bridge is already running."
fi

# 2. Launch Cockpit AppImage
echo "Starting Blue Robotics Cockpit..."
~/auv_ws/tools/Cockpit-linux-x86_64.AppImage --no-sandbox
