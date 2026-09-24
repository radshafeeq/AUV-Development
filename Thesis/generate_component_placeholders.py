import matplotlib.pyplot as plt
import matplotlib.patches as patches
import os

plt.rcParams['font.family'] = 'sans-serif'
plt.rcParams['font.sans-serif'] = ['DejaVu Sans', 'Arial', 'Liberation Sans', 'Helvetica']

out_dir = '/home/radhi/Documents/AUV Development/Thesis/figures'
os.makedirs(out_dir, exist_ok=True)

placeholders = [
    {
        'filename': 'placeholder_frame_hull.png',
        'fig_num': 'Gambar 3.4',
        'comp_name': 'RANGKA (FRAME) DAN LAMBUNG TEKANAN KUSTOM\n(CUSTOM PRESSURE HULL) AUV 8-PENDORONG',
        'spec': 'Bahan Akrilik Transparan Tebal 5 mm, Diameter Luar 110 mm, Tutup Aluminium Anodized Sealing Ganda O-Ring',
        'inst': 'Tempelkan foto rangka struktural dan tabung lambung kedap air di sini'
    },
    {
        'filename': 'placeholder_pixhawk.png',
        'fig_num': 'Gambar 3.5',
        'comp_name': 'PAPAN PENGENDALI PENERBANGAN (FLIGHT CONTROLLER)\nPIXHAWK 2.4.8 (ARDUSUB)',
        'spec': 'Prosesor STM32F427 Cortex-M4 168 MHz, Dual IMU (MPU6000 & LSM303D), Firmware ArduSub vectored_6dof',
        'inst': 'Tempelkan foto unit flight controller Pixhawk 2.4.8 di sini'
    },
    {
        'filename': 'placeholder_rpi4.png',
        'fig_num': 'Gambar 3.6',
        'comp_name': 'KOMPUTER PENDAMPING (COMPANION COMPUTER)\nRASPBERRY PI 4B (BLUEOS 1.4.5)',
        'spec': 'Broadcom BCM2711 Quad-Core Cortex-A72 @ 1.5 GHz, RAM 4GB, Sistem Operasi BlueOS & Jembatan REST MAVLink',
        'inst': 'Tempelkan foto komputer mini Raspberry Pi 4B di sini'
    },
    {
        'filename': 'placeholder_esc.png',
        'fig_num': 'Gambar 3.7',
        'comp_name': 'MODUL PENGENDALI KECEPATAN ELEKTRONIK\n(ESC EMAX BLHELI 30A BIDIRECTIONAL)',
        'spec': 'Arus Kontinu 30A (Burst 40A), Dukungan Firmware Bidirectional 3D (Maju/Mundur), Sinyal PWM 1100–1900 μs',
        'inst': 'Tempelkan foto unit ESC EMAX BLHeli 30A di sini'
    },
    {
        'filename': 'placeholder_thruster.png',
        'fig_num': 'Gambar 3.8',
        'comp_name': 'MOTOR PENDORONG BAWAH AIR\n(BLDC UNDERWATER THRUSTER 12–24V)',
        'spec': 'Motor Brushless Tahan Air dengan Baling-Baling 4-Bilah (Propeller Ducting), Gaya Dorong Nominal 3–5 kgf',
        'inst': 'Tempelkan foto motor pendorong bawah air (underwater thruster) di sini'
    },
    {
        'filename': 'placeholder_baterai.png',
        'fig_num': 'Gambar 3.9',
        'comp_name': 'SUMBER DAYA BATERAI LI-PO 4S 14.8V 6000 MAH &\nPENGISI DAYA CERDAS SKYRC IMAX B6AC V2',
        'spec': 'Konfigurasi 4S1P 14.8V 88.8 Wh, Konektor XT90 Arus Tinggi & Charger Penyeimbang Mikroprosesor Presisi',
        'inst': 'Tempelkan foto baterai Li-Po dan charger SKYRC IMAX B6AC di sini'
    }
]

def make_box(p):
    fig = plt.figure(figsize=(9.0, 5.0), dpi=300)
    ax = fig.add_subplot(111)
    ax.set_xlim(0, 100)
    ax.set_ylim(0, 60)
    ax.axis('off')

    # Background
    bg = patches.FancyBboxPatch((1.5, 1.5), 97, 57, boxstyle="round,pad=0.2,rounding_size=1.2",
                                facecolor='#F8FAFC', edgecolor='#94A3B8', linewidth=1.5, zorder=1)
    ax.add_patch(bg)

    # Dashed Inner Box
    inner_box = patches.FancyBboxPatch((4, 4), 92, 52, boxstyle="round,pad=0.2,rounding_size=0.8",
                                      facecolor='#FFFFFF', edgecolor='#0284C7', linewidth=1.8,
                                      linestyle='--', zorder=2)
    ax.add_patch(inner_box)

    # Header Bar
    hb = patches.FancyBboxPatch((4, 48), 92, 8, boxstyle="round,pad=0.1,rounding_size=0.6",
                                facecolor='#1E3A8A', edgecolor='#1E3A8A', zorder=3)
    ax.add_patch(hb)
    ax.text(50, 52.0, f"[ KOTAK PENEMPATAN FOTO: {p['fig_num']} ]",
            ha='center', va='center', fontsize=9.5, fontweight='bold', color='white', zorder=4)

    # Draw Vector Camera Icon
    cam_body = patches.FancyBboxPatch((44, 36), 12, 8, boxstyle="round,pad=0.1,rounding_size=0.8",
                                     facecolor='#334155', edgecolor='#0F172A', linewidth=1.2, zorder=4)
    ax.add_patch(cam_body)
    cam_top = patches.Polygon([[47, 44], [48.5, 46], [51.5, 46], [53, 44]], closed=True,
                              facecolor='#1E293B', edgecolor='#0F172A', linewidth=1.0, zorder=4)
    ax.add_patch(cam_top)
    cam_lens = patches.Circle((50, 40), 2.8, facecolor='#E2E8F0', edgecolor='#0F172A', linewidth=1.2, zorder=5)
    ax.add_patch(cam_lens)
    cam_pupil = patches.Circle((50, 40), 1.5, facecolor='#0284C7', edgecolor='#0369A1', linewidth=0.8, zorder=6)
    ax.add_patch(cam_pupil)

    # Component Name
    ax.text(50, 27.5, p['comp_name'], ha='center', va='center',
            fontsize=9.2, fontweight='bold', color='#0F172A', linespacing=1.25, zorder=4)

    # Specifications
    ax.text(50, 18.0, f"Spesifikasi Teknis: {p['spec']}", ha='center', va='center',
            fontsize=7.2, fontstyle='italic', color='#475569', zorder=4)

    # Action Instruction Box at Bottom
    inst_bg = patches.FancyBboxPatch((10, 6.0), 80, 7.8, boxstyle="round,pad=0.1,rounding_size=0.4",
                                    facecolor='#EFF6FF', edgecolor='#BFDBFE', linewidth=1.0, zorder=3)
    ax.add_patch(inst_bg)
    ax.text(50, 10.5, f"PETUNJUK: {p['inst']}", ha='center', va='center',
            fontsize=7.8, fontweight='bold', color='#1D4ED8', zorder=4)
    ax.text(50, 7.8, "(Di Microsoft Word / Google Docs: Klik kanan kotak ini lalu pilih 'Ganti Gambar' / 'Change Picture')",
            ha='center', va='center', fontsize=6.8, fontstyle='italic', color='#64748B', zorder=4)

    plt.tight_layout()
    fpath = os.path.join(out_dir, p['filename'])
    plt.savefig(fpath, dpi=300, bbox_inches='tight', facecolor='white', edgecolor='none')
    plt.close()
    print(f"Generated placeholder: {fpath}")

if __name__ == '__main__':
    for p in placeholders:
        make_box(p)
