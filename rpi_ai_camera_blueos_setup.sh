#!/bin/bash
# ==============================================================================
# Raspberry Pi AI Camera (Sony IMX500 CSI Autofocus) BlueOS Setup Script
# ==============================================================================
# Author: Radhi Shafeeq
# Affiliation: Hasanuddin University (Mechatronics Engineering)
# Project: Over-Actuated 6-DOF, 8-Motor Autonomous Underwater Vehicle (AUV)
#
# Hardware:
# - Upper Main Hull: Raspberry Pi 4B + Raspberry Pi AI Camera (IMX500 12.3MP)
# - Subsea Tether IP: 192.168.2.2 (RPi BlueOS) <---> 192.168.2.1 (Topside Laptop)
# - Stream Target: UDP port 5600 (H.264 RTP stream to Topside RTX 4070 GPU)
#
# BlueOS Pirate Mode Instructions:
# 1. Open BlueOS Web Interface in browser: http://192.168.2.2
# 2. Click on the version number in bottom left corner 5 times or go to Settings.
# 3. Toggle "Pirate Mode" (Red Pill Mode) to unlock root host terminal access.
# 4. Open BlueOS Terminal or SSH: `ssh pi@192.168.2.2` (or `root@192.168.2.2`).
# 5. Execute this script: `sudo bash rpi_ai_camera_blueos_setup.sh`
# ==============================================================================

set -e

echo "======================================================================"
echo "   AUV RASPBERRY PI AI CAMERA (IMX500) BLUEOS SETUP & STREAM CONFIG   "
echo "======================================================================"

TOPSIDE_IP="192.168.2.1"
UDP_PORT="5600"
STREAM_WIDTH="1280"
STREAM_HEIGHT="720"
STREAM_FPS="30"

# 1. Determine boot config path
CONFIG_TXT="/boot/firmware/config.txt"
if [ ! -f "$CONFIG_TXT" ]; then
    CONFIG_TXT="/boot/config.txt"
fi

if [ ! -f "$CONFIG_TXT" ]; then
    echo "[ERROR] Cannot locate config.txt! Checked /boot/firmware/config.txt and /boot/config.txt."
    exit 1
fi

echo "[1/5] Configuring Raspberry Pi Device Tree in $CONFIG_TXT..."
# Create backup
cp "$CONFIG_TXT" "${CONFIG_TXT}.bak_$(date +%Y%m%d_%H%M%S)"

# Ensure camera auto-detect doesn't conflict and IMX500 overlay is loaded
if grep -q "dtoverlay=imx500" "$CONFIG_TXT"; then
    echo "  -> dtoverlay=imx500 already present in $CONFIG_TXT"
else
    echo "  -> Appending dtoverlay=imx500 and camera settings..."
    cat << 'EOF' >> "$CONFIG_TXT"

# --- Raspberry Pi AI Camera (Sony IMX500) Config for AUV ---
camera_auto_detect=0
dtoverlay=imx500
gpu_mem=128
# -----------------------------------------------------------
EOF
fi

# 2. Check and Install Required Camera Tools
echo "[2/5] Checking rpicam-apps / libcamera packages..."
if ! command -v rpicam-vid &> /dev/null && ! command -v libcamera-vid &> /dev/null; then
    echo "  -> Installing rpicam-apps-lite, libcamera-tools, and v4l-utils..."
    apt-get update -qq || true
    apt-get install -y -qq rpicam-apps-lite libcamera-tools v4l-utils imx500-firmware || true
fi

# Detect which binary is installed (rpicam-vid on Bookworm, libcamera-vid on Bullseye)
CAM_BIN="rpicam-vid"
if ! command -v rpicam-vid &> /dev/null; then
    if command -v libcamera-vid &> /dev/null; then
        CAM_BIN="libcamera-vid"
    else
        echo "[WARNING] Neither rpicam-vid nor libcamera-vid found. Please ensure Raspberry Pi OS packages are updated."
    fi
fi
echo "  -> Using camera capture utility: $CAM_BIN"

# 3. Create Standalone Streaming Script
STREAM_SCRIPT="/usr/local/bin/start_auv_front_camera_stream.sh"
echo "[3/5] Writing stream script to $STREAM_SCRIPT..."

cat << EOF > "$STREAM_SCRIPT"
#!/bin/bash
# AUV Front AI Camera Hardware H.264 Streamer with Continuous Autofocus
# Target: Topside Laptop at $TOPSIDE_IP:$UDP_PORT

CAM_CMD="$CAM_BIN"
if ! command -v \$CAM_CMD &> /dev/null; then
    CAM_CMD="libcamera-vid"
fi

echo "[AUV Front Cam] Starting hardware H.264 stream to udp://$TOPSIDE_IP:$UDP_PORT..."
exec \$CAM_CMD \\
    -t 0 \\
    --width $STREAM_WIDTH \\
    --height $STREAM_HEIGHT \\
    --framerate $STREAM_FPS \\
    --codec h264 \\
    --inline \\
    --profile baseline \\
    --bitrate 3500000 \\
    --intra 15 \\
    --autofocus-mode continuous \\
    --autofocus-range normal \\
    --nopreview \\
    -o "udp://$TOPSIDE_IP:$UDP_PORT"
EOF

chmod +x "$STREAM_SCRIPT"
echo "  -> Executable created: $STREAM_SCRIPT"

# 4. Create Systemd Service for Auto-Start in Pirate Mode
SERVICE_FILE="/etc/systemd/system/auv-ai-camera.service"
echo "[4/5] Creating systemd service at $SERVICE_FILE..."

cat << EOF > "$SERVICE_FILE"
[Unit]
Description=AUV Front AI Camera (Sony IMX500) UDP Streamer
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=root
ExecStart=$STREAM_SCRIPT
Restart=always
RestartSec=3
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=multi-user.target
EOF

systemctl daemon-reload
echo "  -> Systemd service configured. You can start it with: systemctl start auv-ai-camera"

# 5. Final Diagnostic Verification
echo "[5/5] Diagnostic instructions & next steps:"
echo "----------------------------------------------------------------------"
echo "1. If this is the first time adding dtoverlay=imx500, REBOOT the RPi:"
echo "   sudo reboot"
echo ""
echo "2. After reboot, verify that IMX500 is detected:"
echo "   rpicam-hello --list-cameras"
echo "   (or libcamera-hello --list-cameras)"
echo ""
echo "3. To test the live camera stream manually:"
echo "   $STREAM_SCRIPT"
echo ""
echo "4. To enable continuous streaming on boot:"
echo "   sudo systemctl enable --now auv-ai-camera"
echo "======================================================================"
