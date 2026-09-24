import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
import os

plt.rcParams['font.family'] = 'sans-serif'
plt.rcParams['font.sans-serif'] = ['DejaVu Sans', 'Arial', 'Liberation Sans', 'Helvetica']

os.makedirs('/home/radhi/Documents/AUV Development/Thesis/figures', exist_ok=True)

# -------------------------------------------------------------------------
# 1. ARSITEKTUR SITL DIAGRAM
# -------------------------------------------------------------------------
def generate_sitl_diagram():
    fig = plt.figure(figsize=(13.0, 8.5), dpi=300)
    ax = fig.add_subplot(111)
    ax.set_xlim(0, 100)
    ax.set_ylim(0, 100)
    ax.axis('off')

    # Colors
    C_OUTER_BG = '#F8FAFC'
    C_OUTER_BORDER = '#334155'
    C_BOX_BG = '#FFFFFF'
    C_BLUE_HEADER = '#1E3A8A'
    C_TEAL_HEADER = '#0D9488'
    C_AMBER_HEADER = '#D97706'
    C_PURPLE_HEADER = '#4338CA'
    C_ARROW = '#1E3A8A'

    # Outer Frame
    outer = patches.FancyBboxPatch((2, 2), 96, 96, boxstyle="round,pad=0.3,rounding_size=1.2",
                                   facecolor=C_OUTER_BG, edgecolor=C_OUTER_BORDER, linewidth=1.5, zorder=1)
    ax.add_patch(outer)
    ax.text(50, 95.5, "LINGKUNGAN SIMULASI SOFTWARE-IN-THE-LOOP (SITL)",
            ha='center', va='center', fontsize=10.5, fontweight='bold', color='#0F172A', zorder=2)

    def draw_card(x, y, w, h, header_bg, title, items):
        card = patches.FancyBboxPatch((x - w/2, y - h/2), w, h, boxstyle="round,pad=0.2,rounding_size=0.8",
                                     facecolor=C_BOX_BG, edgecolor=header_bg, linewidth=1.4, zorder=3)
        ax.add_patch(card)
        hb_h = 4.0
        hb = patches.FancyBboxPatch((x - w/2, y + h/2 - hb_h), w, hb_h, boxstyle="round,pad=0.15,rounding_size=0.6",
                                    facecolor=header_bg, edgecolor=header_bg, linewidth=1.0, zorder=4)
        ax.add_patch(hb)
        ax.text(x, y + h/2 - hb_h/2, title, ha='center', va='center', fontsize=8.8, fontweight='bold', color='white', zorder=5)
        
        start_y = y + h/2 - hb_h - 2.2
        spacing = 2.4
        for idx, itm in enumerate(items):
            ax.text(x - w/2 + 2.0, start_y - idx*spacing, f"• {itm}", ha='left', va='center',
                    fontsize=8.0, color='#1E293B', zorder=5)

    def draw_conn(x1, y1, x2, y2, label="", color=C_ARROW, rad=0.0):
        arr = patches.FancyArrowPatch((x1, y1), (x2, y2), arrowstyle="<->",
                                      connectionstyle=f"arc3,rad={rad}",
                                      mutation_scale=12, linewidth=1.5, color=color, zorder=6)
        ax.add_patch(arr)
        if label:
            mx, my = (x1+x2)/2, (y1+y2)/2
            ax.text(mx, my + 1.6, label, ha='center', va='center', fontsize=7.6, fontweight='bold',
                    color=color, zorder=7, bbox=dict(boxstyle="square,pad=0.2", facecolor='#F1F5F9', edgecolor='none'))

    def draw_dir_arrow(x1, y1, x2, y2, label="", color=C_ARROW):
        arr = patches.FancyArrowPatch((x1, y1), (x2, y2), arrowstyle="->",
                                      mutation_scale=12, linewidth=1.5, color=color, zorder=6)
        ax.add_patch(arr)
        if label:
            mx, my = (x1+x2)/2, (y1+y2)/2
            ax.text(mx, my + 1.2, label, ha='center', va='center', fontsize=7.4, fontweight='bold',
                    color=color, zorder=7, bbox=dict(boxstyle="square,pad=0.15", facecolor='white', edgecolor='none'))

    # Top Row: Gazebo Harmonic & ArduSub SITL
    card_w = 38
    c1_x = 24
    c2_x = 76
    draw_card(c1_x, 72, card_w, 22, C_BLUE_HEADER, "GAZEBO HARMONIC (gz-sim 8.14)",
              [
                  "Model SDF AUV 8-Pendorong (vectored_6dof)",
                  "Plugin Daya Apung Archimedes (gz-sim-buoyancy)",
                  "Plugin Hidrodinamika Nonlinier Fossen (6-DOF)",
                  "Plugin Jembatan Sistem Fisika ardupilot_gazebo",
                  "Rendering Kamera Virtual Monokuler Subsea"
              ])

    draw_card(c2_x, 72, card_w, 22, C_PURPLE_HEADER, "ARDUSUB AUTOPILOT SITL",
              [
                  "Firmware ArduSub-4.6.0-beta1 (vectored_6dof)",
                  "Estimator Keadaan Extended Kalman Filter 3 (EKF3)",
                  "Loop Penstabil Sikap Bodi (Attitude PID Controller)",
                  "Matriks Pemetaan Alokasi Gaya Dorong 6x8",
                  "Eksekusi Lockstep Waktu Nyata @ 200 Hz"
              ])

    # Connecting Socket between Gazebo & ArduSub
    draw_conn(c1_x + card_w/2, 72, c2_x - card_w/2, 72, "JSON / UDP Lockstep\nSocket (200 Hz)", color='#047857')

    # Middle Row: ROS 2 Jazzy & MAVProxy
    draw_card(c1_x, 36, card_w, 20, C_TEAL_HEADER, "ROS 2 JAZZY JALISCO",
              [
                  "ros_gz_bridge: Konversi Topik Gazebo ke ROS 2",
                  "Topik Citra: /camera/image_raw (30 FPS @ 720p)",
                  "Topik Odometri: /model/bluerov2_heavy/odometry",
                  "Node Ekstraksi Kecepatan Bodi FRD display_velocity"
              ])

    draw_card(c2_x, 36, card_w, 20, C_AMBER_HEADER, "MAVPROXY GCS GATEWAY",
              [
                  "MAVLink Telemetry Router (UDP: 14550 & 14551)",
                  "Ground Control Station (QGroundControl / HUD)",
                  "Monitoring Keadaan Baterai, Mode, & Heartbeat",
                  "Pengiriman Perintah Manual & Setpoint MAVLink"
              ])

    # Vertical Links from Top Row to Middle Row
    draw_dir_arrow(c1_x - 7, 61, c1_x - 7, 46, "/camera/image_raw")
    draw_dir_arrow(c1_x + 7, 61, c1_x + 7, 46, "Odometry Ground Truth")
    draw_dir_arrow(c2_x, 61, c2_x, 46, "MAVLink UDP:14550")

    # Bottom Full Width: Topside Estimation & Tracking Node
    draw_card(50, 11.5, 88, 13, C_BLUE_HEADER, "NODE ESTIMATOR KEADAAN OPTIMAL & TRACKING (TOPSIDE WORKSTATION)",
              [
                  "Deteksi Visual YOLO & Algoritma Pra-pemrosesan Kontras CLAHE (Lab Color Space)",
                  "AUVVisualKalmanFilter: Model 8D CWNA, Kovariansi Adaptif R_k(conf), & Mahalanobis Gating chi^2(4)",
                  "AUVDynamicsKalmanFilter: Fusi Kinematika Fossen 6-DOF, EKF Dinamika, & Pengamat Arus Subsea"
              ])

    # Arrows to Bottom Node
    draw_dir_arrow(27, 26, 35, 18, "Citra & Kecepatan")
    draw_dir_arrow(73, 26, 65, 18, "Status Wahana MAVLink")

    plt.tight_layout()
    out = '/home/radhi/Documents/AUV Development/Thesis/figures/arsitektur_sitl.png'
    plt.savefig(out, dpi=300, bbox_inches='tight', facecolor='white', edgecolor='none')
    plt.close()
    print("Generated SITL diagram successfully:", out)

# -------------------------------------------------------------------------
# 2. ARSITEKTUR HITL DIAGRAM
# -------------------------------------------------------------------------
def generate_hitl_diagram():
    fig = plt.figure(figsize=(13.0, 11.0), dpi=300)
    ax = fig.add_subplot(111)
    ax.set_xlim(0, 100)
    ax.set_ylim(0, 100)
    ax.axis('off')

    C_OUTER_BG = '#F8FAFC'
    C_OUTER_BORDER = '#334155'
    C_BOX_BG = '#FFFFFF'
    C_BLUE_HEADER = '#1E3A8A'
    C_CYAN_HEADER = '#0284C7'
    C_PURPLE_HEADER = '#4338CA'
    C_AMBER_HEADER = '#D97706'
    C_GREEN_HEADER = '#15803D'

    # Outer Frame
    outer = patches.FancyBboxPatch((2, 2), 96, 96, boxstyle="round,pad=0.3,rounding_size=1.2",
                                   facecolor=C_OUTER_BG, edgecolor=C_OUTER_BORDER, linewidth=1.5, zorder=1)
    ax.add_patch(outer)
    ax.text(50, 96.0, "ARSITEKTUR HARDWARE-IN-THE-LOOP (HITL) MEKATRONIKA AUV",
            ha='center', va='center', fontsize=10.5, fontweight='bold', color='#0F172A', zorder=2)

    def draw_card(x, y, w, h, header_bg, title, items):
        card = patches.FancyBboxPatch((x - w/2, y - h/2), w, h, boxstyle="round,pad=0.2,rounding_size=0.8",
                                     facecolor=C_BOX_BG, edgecolor=header_bg, linewidth=1.4, zorder=3)
        ax.add_patch(card)
        hb_h = 3.8
        hb = patches.FancyBboxPatch((x - w/2, y + h/2 - hb_h), w, hb_h, boxstyle="round,pad=0.15,rounding_size=0.6",
                                    facecolor=header_bg, edgecolor=header_bg, linewidth=1.0, zorder=4)
        ax.add_patch(hb)
        ax.text(x, y + h/2 - hb_h/2, title, ha='center', va='center', fontsize=8.6, fontweight='bold', color='white', zorder=5)
        
        start_y = y + h/2 - hb_h - 2.0
        spacing = 2.2
        for idx, itm in enumerate(items):
            ax.text(x - w/2 + 2.0, start_y - idx*spacing, f"• {itm}", ha='left', va='center',
                    fontsize=8.0, color='#1E293B', zorder=5)

    def draw_bi_arrow(x1, y1, x2, y2, label=""):
        arr = patches.FancyArrowPatch((x1, y1), (x2, y2), arrowstyle="<->",
                                      mutation_scale=13, linewidth=2.0, color='#0284C7', zorder=6)
        ax.add_patch(arr)
        if label:
            mx, my = (x1+x2)/2, (y1+y2)/2
            ax.text(mx + 2.0, my, label, ha='left', va='center', fontsize=7.8, fontweight='bold',
                    color='#0369A1', zorder=7, bbox=dict(boxstyle="round,pad=0.25", facecolor='#F0F9FF', edgecolor='#BAE6FD'))

    def draw_dir_arrow(x1, y1, x2, y2, label="", color='#1E3A8A'):
        arr = patches.FancyArrowPatch((x1, y1), (x2, y2), arrowstyle="->",
                                      mutation_scale=12, linewidth=1.5, color=color, zorder=6)
        ax.add_patch(arr)
        if label:
            mx, my = (x1+x2)/2, (y1+y2)/2
            ax.text(mx, my, label, ha='center', va='center', fontsize=7.4, fontweight='bold',
                    color=color, zorder=7, bbox=dict(boxstyle="square,pad=0.15", facecolor='white', edgecolor='none'))

    # Top: Topside Workstation
    draw_card(50, 83.5, 88, 17, C_BLUE_HEADER, "STASIUN PERMUKAAN: TOPSIDE WORKSTATION (LAPTOP GPU)",
              [
                  "Penerimaan Video Streaming UDP Port 5600/5601 via OpenCV & Pra-pemrosesan Kontras CLAHE",
                  "Deteksi Objek Visual YOLO (Akselerasi CUDA / TensorRT pada GPU NVIDIA)",
                  "Visual Kalman Filter 8D: Propagasi CWNA, Kovariansi Adaptif R_k(conf), & Penanganan Oklusi",
                  "Visual Servoing Controller: Menghitung Setpoint Galat & Transmisi Paket MAVLink MANUAL_CONTROL"
              ])

    # Middle Link: Subsea Ethernet Tether
    draw_bi_arrow(50, 75.0, 50, 64.0, "Kabel Tether Ethernet Subsea (CAT5e/CAT6)\nSubnet IP: 192.168.2.x | Bandwidth 100 Mbps")

    # Middle: Companion Computer Raspberry Pi 4B
    draw_card(50, 53.5, 88, 17, C_PURPLE_HEADER, "KOMPUTER PENDAMPING SUBSEA: RASPBERRY PI 4B (BLUEOS 1.4.5)",
              [
                  "Video Streamer Service (H.264 Hardware Encoder, Port 5600/5601, 720p@30fps)",
                  "BlueOS mavlink2rest Bridge (REST API Endpoint HTTP: Port 6040)",
                  "MAVLink Router: Routing Endpoints UDP: 14550 & Serial UART: /dev/ttyAMA0",
                  "Node Eksekusi AUVDynamicsKalmanFilter (Estimasi Keadaan 6-DOF Live @ 50 Hz)"
              ])

    # Bottom Row: Kamera, Flight Controller Pixhawk, Sensor Bar30, ESC & Motor
    # Bottom Left: Modul Kamera
    draw_card(22, 28, 32, 17, C_CYAN_HEADER, "MODUL KAMERA SUBSEA",
              [
                  "Raspberry Pi Camera Rev 1.3 (5MP OV5647)",
                  "Logitech C922 Pro USB Kamera",
                  "Antarmuka CSI-2 Bus & USB 2.0",
                  "Akuisisi Video 720p @ 30 FPS"
              ])

    # Bottom Right: Flight Controller Pixhawk 2.4.8
    draw_card(68, 28, 52, 17, C_AMBER_HEADER, "FLIGHT CONTROLLER: PIXHAWK 2.4.8 (ARDUSUB)",
              [
                  "Prosesor STM32F427 Cortex-M4 32-bit @ 168 MHz",
                  "Dual Sensor Inersia Internal: IMU MPU6000 & LSM303D",
                  "Firmware ArduSub kerangka vectored_6dof",
                  "Output Sinyal PWM 1-8 untuk Pengendali Kecepatan ESC"
              ])

    # Arrows from Raspberry Pi to Bottom Devices
    draw_dir_arrow(22, 45, 22, 36.5, "Kabel CSI-2 / USB", color='#0284C7')
    draw_dir_arrow(68, 45, 68, 36.5, "Serial UART /dev/ttyAMA0", color='#B45309')

    # Sub-bottom Row: Sensor Bar30 & 8x ESC + Thruster
    draw_card(32, 9.5, 34, 11, C_CYAN_HEADER, "SENSOR KEDALAMAN (BAR30)",
              [
                  "Sensor Tekanan Digital MS5837-30BA",
                  "Resolusi Sub-Milimeter Kedalaman Air",
                  "Antarmuka Bus Komunikasi I2C Subsea"
              ])

    draw_card(73, 9.5, 42, 11, C_GREEN_HEADER, "AKTUATOR: 8x ESC & BLDC THRUSTER",
              [
                  "8 Unit ESC EMAX BLHeli 30A Bidirectional",
                  "8 Unit BLDC Underwater Thruster (Propeler 4-Bilah)",
                  "4x Pendorong Horizontal (45 deg) & 4x Vertikal"
              ])

    draw_dir_arrow(50, 19.5, 36, 15, "I2C Bus", color='#0284C7')
    draw_dir_arrow(73, 19.5, 73, 15, "8x Saluran PWM", color='#15803D')

    plt.tight_layout()
    out = '/home/radhi/Documents/AUV Development/Thesis/figures/arsitektur_hitl.png'
    plt.savefig(out, dpi=300, bbox_inches='tight', facecolor='white', edgecolor='none')
    plt.close()
    print("Generated HITL diagram successfully:", out)

if __name__ == '__main__':
    generate_sitl_diagram()
    generate_hitl_diagram()
