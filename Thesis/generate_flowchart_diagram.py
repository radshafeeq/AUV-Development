import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
import os

# Set matplotlib font properties for publication quality
plt.rcParams['font.family'] = 'sans-serif'
plt.rcParams['font.sans-serif'] = ['DejaVu Sans', 'Arial', 'Liberation Sans', 'Helvetica']

fig = plt.figure(figsize=(14.0, 13.0), dpi=300)
ax = fig.add_subplot(111)
ax.set_xlim(0, 100)
ax.set_ylim(0, 100)
ax.axis('off')

# Professional Academic Color Palette
C_START_BG = '#E8F5E9'
C_START_BORDER = '#2E7D32'
C_START_TEXT = '#1B5E20'

C_BOX_BG = '#F8FAFC'
C_BOX_BORDER = '#1E3A8A'
C_BOX_HEADER_BG = '#1E3A8A'
C_BOX_HEADER_TEXT = '#FFFFFF'

C_DEC_BG = '#FFFBEB'
C_DEC_BORDER = '#D97706'
C_DEC_TEXT = '#B45309'

C_ARROW = '#1E3A8A'
C_FEEDBACK_ARROW = '#DC2626'
C_SUCCESS_ARROW = '#15803D'

def draw_pill(ax, x, y, w, h, text, bg_col, border_col, text_col):
    box = patches.FancyBboxPatch((x - w/2, y - h/2), w, h,
                                boxstyle=f"round,pad=0.2,rounding_size={h/2}",
                                facecolor=bg_col, edgecolor=border_col, linewidth=2, zorder=3)
    ax.add_patch(box)
    ax.text(x, y, text, ha='center', va='center', fontsize=11, fontweight='bold', color=text_col, zorder=4)

def draw_process_box(ax, x, y, w, h, title, bullets):
    # Main outer card
    box = patches.FancyBboxPatch((x - w/2, y - h/2), w, h,
                                boxstyle="round,pad=0.25,rounding_size=0.8",
                                facecolor=C_BOX_BG, edgecolor=C_BOX_BORDER, linewidth=1.5, zorder=3)
    ax.add_patch(box)
    
    # Title header bar
    header_h = 3.2
    header_y = y + h/2 - header_h/2
    header_box = patches.FancyBboxPatch((x - w/2, y + h/2 - header_h), w, header_h,
                                        boxstyle="round,pad=0.2,rounding_size=0.6",
                                        facecolor=C_BOX_HEADER_BG, edgecolor=C_BOX_BORDER, linewidth=1.0, zorder=4)
    ax.add_patch(header_box)
    ax.text(x, header_y, title, ha='center', va='center', fontsize=8.8, fontweight='bold', color=C_BOX_HEADER_TEXT, zorder=5)
    
    # Bullets text
    start_text_y = y + h/2 - header_h - 1.8
    line_spacing = 2.1
    for idx, b in enumerate(bullets):
        ax.text(x - w/2 + 1.8, start_text_y - idx * line_spacing, f"•  {b}",
                ha='left', va='center', fontsize=8.0, color='#1E293B', zorder=5)

def draw_diamond(ax, x, y, w, h, title):
    pts = np.array([
        [x, y + h/2],
        [x + w/2, y],
        [x, y - h/2],
        [x - w/2, y]
    ])
    poly = patches.Polygon(pts, closed=True, facecolor=C_DEC_BG, edgecolor=C_DEC_BORDER, linewidth=1.8, zorder=3)
    ax.add_patch(poly)
    ax.text(x, y, title, ha='center', va='center', fontsize=8.2, fontweight='bold', color=C_DEC_TEXT, zorder=4,
            multialignment='center')

def draw_arrow(ax, x1, y1, x2, y2, color=C_ARROW, style="->", rad=0.0):
    arrow = patches.FancyArrowPatch((x1, y1), (x2, y2),
                                   arrowstyle=style,
                                   connectionstyle=f"arc3,rad={rad}",
                                   mutation_scale=13,
                                   linewidth=1.6,
                                   color=color,
                                   zorder=6)
    ax.add_patch(arrow)

# Layout: 2 Balanced Columns with safe side gutters
col1_x = 23.5
col2_x = 74.5
box_w = 36.5

# -------------------------------------------------------------
# COLUMN 1: TAHAP PEMODELAN MATEMATIS & SIMULASI (SITL)
# -------------------------------------------------------------

# 1. Mulai
draw_pill(ax, col1_x, 94.5, 16.0, 3.8, "MULAI", C_START_BG, C_START_BORDER, C_START_TEXT)
draw_arrow(ax, col1_x, 92.6, col1_x, 88.8)

# 2. Fase I
fase1_h = 10.5
fase1_y = 83.5
draw_process_box(ax, col1_x, fase1_y, box_w, fase1_h,
                 "FASE I: STUDI PENDAHULUAN & PERUMUSAN",
                 [
                     "Tinjauan pustaka jurnal internasional bereputasi (>= 2021)",
                     "Identifikasi batasan kendali 6-DOF & ketidakstabilan Munk",
                     "Penetapan tujuan penelitian & spesifikasi wahana AUV 8-motor"
                 ])
draw_arrow(ax, col1_x, fase1_y - fase1_h/2, col1_x, 74.0)

# 3. Fase II
fase2_h = 13.0
fase2_y = 67.5
draw_process_box(ax, col1_x, fase2_y, box_w, fase2_h,
                 "FASE II: PEMODELAN MATEMATIS RIGOROUS",
                 [
                     "Kinematika 6-DOF: Transformasi SO(3) & kuaternion bebas singularitas",
                     "Dinamika Fossen: Tensor inersia M, Coriolis C(nu), redaman coupled D(nu_r)",
                     "Gaya pemulih g(eta) & matriks alokasi gaya dorong T (6x8 over-actuated)",
                     "Penurunan invers semu Moore-Penrose berbobot energi kuadratis minimum"
                 ])
draw_arrow(ax, col1_x, fase2_y - fase2_h/2, col1_x, 56.5)

# 4. Fase III
fase3_h = 13.0
fase3_y = 50.0
draw_process_box(ax, col1_x, fase3_y, box_w, fase3_h,
                 "FASE III: PERANCANGAN DUAL KALMAN FILTER",
                 [
                     "Topside 8D Visual Target KF: Model stokastik CWNA bounding box",
                     "Kovariansi adaptif R_k(conf), gating Mahalanobis, & estimasi oklusi",
                     "Subsea 6-DOF Dynamics EKF: Integrasi sensor IMU & Bar30 kedalaman",
                     "Pengamat gangguan arus laut horizontal (ocean current observer)"
                 ])
draw_arrow(ax, col1_x, fase3_y - fase3_h/2, col1_x, 39.5)

# 5. Fase IV
fase4_h = 10.5
fase4_y = 34.0
draw_process_box(ax, col1_x, fase4_y, box_w, fase4_h,
                 "FASE IV: SIMULASI SOFTWARE-IN-THE-LOOP (SITL)",
                 [
                     "Pemodelan SDF AUV 8-pendorong & plugin hidro Gazebo Harmonic",
                     "Integrasi eksekusi ArduSub SITL (firmware frame vectored_6dof)",
                     "Jembatan topik dua arah ROS 2 Jazzy (ros_gz_bridge & ardupilot_gz)"
                 ])
draw_arrow(ax, col1_x, fase4_y - fase4_h/2, col1_x, 24.8)

# 6. Decision 1
dec1_y = 20.0
dec1_w = 31.0
dec1_h = 8.5
draw_diamond(ax, col1_x, dec1_y, dec1_w, dec1_h, "Simulasi SITL Valid\n& Konvergen?")

# Decision 1 Feedback (Tidak -> Fase II)
ax.text(col1_x - dec1_w/2 - 1.2, dec1_y + 1.2, "Tidak", fontsize=8.5, fontweight='bold', color=C_FEEDBACK_ARROW, ha='right')
left_loop_x = 2.2
ax.plot([col1_x - dec1_w/2, left_loop_x, left_loop_x, col1_x - box_w/2],
        [dec1_y, dec1_y, fase2_y, fase2_y],
        color=C_FEEDBACK_ARROW, linewidth=1.5, linestyle='--', zorder=5)
draw_arrow(ax, left_loop_x, fase2_y, col1_x - box_w/2, fase2_y, color=C_FEEDBACK_ARROW)
ax.text(left_loop_x + 0.6, (dec1_y + fase2_y)/2, "Revisi Pemodelan &\nPenalaan Kovariansi Filter",
        fontsize=7.5, color=C_FEEDBACK_ARROW, fontweight='bold', rotation=90, va='center', ha='left',
        bbox=dict(boxstyle="square,pad=0.2", facecolor='white', edgecolor='none'))

# Decision 1 Success (Ya -> Lane 2 / Fase V)
gutter_x = 49.0
ax.text(col1_x + dec1_w/2 + 1.2, dec1_y + 1.2, "Ya", fontsize=8.5, fontweight='bold', color=C_SUCCESS_ARROW, ha='left')
ax.plot([col1_x + dec1_w/2, gutter_x, gutter_x, col2_x],
        [dec1_y, dec1_y, 93.0, 93.0],
        color=C_SUCCESS_ARROW, linewidth=1.5, linestyle='-', zorder=5)
draw_arrow(ax, col2_x, 93.0, col2_x, 88.5, color=C_SUCCESS_ARROW)

# Middle Gutter Banner Label
ax.text(gutter_x, 56.0, "Validasi SITL Sukses\n(Lanjut ke Integrasi Fisik)",
        fontsize=8.0, color=C_SUCCESS_ARROW, fontweight='bold', rotation=90, va='center', ha='center',
        bbox=dict(boxstyle="round,pad=0.3", facecolor='#F0FDF4', edgecolor='#86EFAC', linewidth=1.0))

# -------------------------------------------------------------
# COLUMN 2: TAHAP INTEGRASI PERANGKAT KERAS (HITL) & VALIDASI
# -------------------------------------------------------------

# 7. Fase V
fase5_h = 13.0
fase5_y = 82.0
draw_process_box(ax, col2_x, fase5_y, box_w, fase5_h,
                 "FASE V: INTEGRASI HARDWARE (HITL) & FISIK",
                 [
                     "Sasis akrilik kedap air & rangka kustom wahana AUV 8-thruster",
                     "Integrasi Pixhawk 2.4.8, Raspberry Pi 4B (BlueOS), & 8 ESC EMAX 30A",
                     "Motor BLDC underwater thruster, baterai Li-Po 4S 6000 mAh & sensor Bar30",
                     "Kamera Pi Rev 1.3 & Logitech C922 via tether Ethernet subsea RJ45"
                 ])
draw_arrow(ax, col2_x, fase5_y - fase5_h/2, col2_x, 71.0)

# 8. Fase VI
fase6_h = 13.0
fase6_y = 64.5
draw_process_box(ax, col2_x, fase6_y, box_w, fase6_h,
                 "FASE VI: PENGUJIAN EKSPERIMENTAL & DATA",
                 [
                     "Skenario 1: Pelacakan visual target monokuler YOLO & uji oklusi",
                     "Skenario 2: Rekonstruksi kecepatan 6-DOF & kompensasi momen Munk",
                     "Skenario 3: Analisis waktu komputasi, latensi tether UDP, & profil real-time",
                     "Pencatatan telemetri MAVLink & evaluasi metrik akurasi navigasi"
                 ])
draw_arrow(ax, col2_x, fase6_y - fase6_h/2, col2_x, 53.5)

# 9. Decision 2
dec2_y = 48.5
dec2_w = 31.0
dec2_h = 8.5
draw_diamond(ax, col2_x, dec2_y, dec2_w, dec2_h, "Kinerja Memenuhi\nKriteria Keberhasilan?")

# Decision 2 Feedback (Tidak -> Fase V)
ax.text(col2_x + dec2_w/2 + 1.2, dec2_y + 1.2, "Tidak", fontsize=8.5, fontweight='bold', color=C_FEEDBACK_ARROW, ha='left')
right_loop_x = 97.2
ax.plot([col2_x + dec2_w/2, right_loop_x, right_loop_x, col2_x + box_w/2],
        [dec2_y, dec2_y, fase5_y, fase5_y],
        color=C_FEEDBACK_ARROW, linewidth=1.5, linestyle='--', zorder=5)
draw_arrow(ax, right_loop_x, fase5_y, col2_x + box_w/2, fase5_y, color=C_FEEDBACK_ARROW)
ax.text(right_loop_x - 0.6, (dec2_y + fase5_y)/2, "Penalaan Ulang Algoritma\nKendali & Estimasi Fisik",
        fontsize=7.5, color=C_FEEDBACK_ARROW, fontweight='bold', rotation=-90, va='center', ha='right',
        bbox=dict(boxstyle="square,pad=0.2", facecolor='white', edgecolor='none'))

# Decision 2 Success (Ya -> Fase VII)
ax.text(col2_x + 1.5, dec2_y - dec2_h/2 - 2.0, "Ya", fontsize=8.5, fontweight='bold', color=C_SUCCESS_ARROW)
draw_arrow(ax, col2_x, dec2_y - dec2_h/2, col2_x, 37.5, color=C_SUCCESS_ARROW)

# 10. Fase VII
fase7_h = 11.5
fase7_y = 31.5
draw_process_box(ax, col2_x, fase7_y, box_w, fase7_h,
                 "FASE VII: ANALISIS DATA & PEMBAHASAN",
                 [
                     "Evaluasi metrik kuantitatif: RMSE posisi/kecepatan, MAE, & settling time",
                     "Pembahasan integratif kinematika, dinamika coupled, & ketahanan tracking",
                     "Penyusunan naskah komprehensif Tugas Akhir & persiapan publikasi"
                 ])
draw_arrow(ax, col2_x, fase7_y - fase7_h/2, col2_x, 21.5)

# 11. Selesai
draw_pill(ax, col2_x, 18.0, 18.0, 3.8, "SELESAI", C_START_BG, C_START_BORDER, C_START_TEXT)

# Lane Column Super-Headers
ax.text(col1_x, 98.8, "TAHAP PEMODELAN MATEMATIS & SIMULASI (SITL)",
        ha='center', va='center', fontsize=9.0, fontweight='bold', color='#1E3A8A',
        bbox=dict(boxstyle="square,pad=0.35", facecolor='#F1F5F9', edgecolor='#94A3B8', linewidth=1.2))

ax.text(col2_x, 98.8, "TAHAP INTEGRASI PERANGKAT KERAS (HITL) & VALIDASI",
        ha='center', va='center', fontsize=9.0, fontweight='bold', color='#1E3A8A',
        bbox=dict(boxstyle="square,pad=0.35", facecolor='#F1F5F9', edgecolor='#94A3B8', linewidth=1.2))

plt.tight_layout()
os.makedirs('/home/radhi/Documents/AUV Development/Thesis/figures', exist_ok=True)
out_png = '/home/radhi/Documents/AUV Development/Thesis/figures/diagram_alir_penelitian.png'
plt.savefig(out_png, dpi=300, bbox_inches='tight', facecolor='white', edgecolor='none')
plt.close()
print(f"Flowchart successfully updated: {out_png}")
