import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
import os

plt.rcParams['font.family'] = 'sans-serif'
plt.rcParams['font.sans-serif'] = ['DejaVu Sans', 'Arial', 'Liberation Sans', 'Helvetica']

os.makedirs('/home/radhi/Documents/AUV Development/Thesis/figures', exist_ok=True)

def generate_sname_diagram():
    fig = plt.figure(figsize=(14.0, 7.5), dpi=300)
    ax = fig.add_subplot(111)
    ax.set_xlim(0, 140)
    ax.set_ylim(0, 75)
    ax.axis('off')

    # Color palette
    C_BG = '#F8FAFC'
    C_BORDER = '#334155'
    C_CARD_BG = '#FFFFFF'
    C_BLUE = '#1E3A8A'
    C_TEAL = '#0D9488'
    C_NAVY = '#0F172A'
    C_ACCENT_RED = '#DC2626'
    C_ACCENT_GREEN = '#16A34A'
    C_ACCENT_BLUE = '#2563EB'
    C_ACCENT_PURPLE = '#7C3AED'
    C_WATER = '#E0F2FE'

    # Outer Frame
    outer = patches.FancyBboxPatch((1.5, 1.5), 137, 72, boxstyle="round,pad=0.3,rounding_size=1.0",
                                   facecolor=C_BG, edgecolor=C_BORDER, linewidth=1.4, zorder=1)
    ax.add_patch(outer)

    # Top Title
    ax.text(70, 71.2, "SISTEM KERANGKA ACUAN INERSIA BUMI (NED) DAN KERANGKA ACUAN BODI (FRD)",
            ha='center', va='center', fontsize=11, fontweight='bold', color=C_NAVY, zorder=2)
    ax.text(70, 68.8, "Konvensi Standar The Society of Naval Architects and Marine Engineers (SNAME, 1950) & Fossen (2021)",
            ha='center', va='center', fontsize=8.8, fontstyle='italic', color='#475569', zorder=2)

    # -------------------------------------------------------------------------
    # LEFT PANEL: EARTH-FIXED FRAME {n} (NED)
    # -------------------------------------------------------------------------
    p1 = patches.FancyBboxPatch((4, 4), 48, 62, boxstyle="round,pad=0.3,rounding_size=0.8",
                                facecolor=C_CARD_BG, edgecolor='#94A3B8', linewidth=1.2, zorder=2)
    ax.add_patch(p1)

    # Header Panel 1
    h1 = patches.FancyBboxPatch((4, 61), 48, 5, boxstyle="round,pad=0.2,rounding_size=0.6",
                                facecolor=C_BLUE, edgecolor=C_BLUE, linewidth=1.0, zorder=3)
    ax.add_patch(h1)
    ax.text(28, 63.5, "KERANGKA ACUAN INERSIA BUMI {n} (NED)",
            ha='center', va='center', fontsize=9.2, fontweight='bold', color='white', zorder=4)

    # Water surface line
    ax.plot([8, 48], [52, 52], color='#0284C7', linewidth=2.0, linestyle='--', zorder=4)
    ax.text(28, 53.5, "Permukaan Air / Datum Geografis (z_n = 0)", ha='center', va='bottom',
            fontsize=7.8, color='#0284C7', fontstyle='italic', zorder=5)

    # Origin O_n
    on_x, on_y = 20, 42
    ax.scatter(on_x, on_y, s=70, color=C_NAVY, zorder=6)
    ax.text(on_x - 1.5, on_y + 2.0, r"$\mathbf{O}_n$", fontsize=10, fontweight='bold', color=C_NAVY, zorder=7)

    # Axis x_n (North)
    arr_xn = patches.FancyArrowPatch((on_x, on_y), (on_x, on_y + 13), arrowstyle="-|>",
                                     mutation_scale=14, linewidth=2.0, color=C_ACCENT_RED, zorder=6)
    ax.add_patch(arr_xn)
    ax.text(on_x, on_y + 14.5, r"$\mathbf{x}_n$ (Utara / True North)", ha='center', va='bottom',
            fontsize=8.5, fontweight='bold', color=C_ACCENT_RED, zorder=7)

    # Axis y_n (East)
    arr_yn = patches.FancyArrowPatch((on_x, on_y), (on_x + 18, on_y), arrowstyle="-|>",
                                     mutation_scale=14, linewidth=2.0, color=C_ACCENT_GREEN, zorder=6)
    ax.add_patch(arr_yn)
    ax.text(on_x + 19.5, on_y, r"$\mathbf{y}_n$ (Timur / True East)", ha='left', va='center',
            fontsize=8.5, fontweight='bold', color=C_ACCENT_GREEN, zorder=7)

    # Axis z_n (Down)
    arr_zn = patches.FancyArrowPatch((on_x, on_y), (on_x, on_y - 20), arrowstyle="-|>",
                                     mutation_scale=14, linewidth=2.0, color=C_ACCENT_BLUE, zorder=6)
    ax.add_patch(arr_zn)
    ax.text(on_x, on_y - 22.0, r"$\mathbf{z}_n$ (Bawah / Down $\mathbf{g}$)", ha='center', va='top',
            fontsize=8.5, fontweight='bold', color=C_ACCENT_BLUE, zorder=7)

    # Info Box Left
    info_l = [
        r"• Vektor Posisi: $\mathbf{\eta}_1 = [x, y, z]^T \in \mathbb{R}^3$",
        r"• Sudut Euler: $\mathbf{\eta}_2 = [\phi, \theta, \psi]^T \in \mathbb{R}^3$",
        r"• Orientasi: Roll ($\phi$), Pitch ($\theta$), Yaw ($\psi$)",
        r"• Sifat: Tangensial bumi stasioner (Inersia)"
    ]
    for idx, txt in enumerate(info_l):
        ax.text(6.5, 14 - idx*2.8, txt, fontsize=7.8, color='#1E293B', zorder=5)

    # -------------------------------------------------------------------------
    # RIGHT PANEL: BODY-FIXED FRAME {b} (FRD) & SNAME 6-DOF
    # -------------------------------------------------------------------------
    p2 = patches.FancyBboxPatch((55, 4), 81, 62, boxstyle="round,pad=0.3,rounding_size=0.8",
                                facecolor=C_CARD_BG, edgecolor='#94A3B8', linewidth=1.2, zorder=2)
    ax.add_patch(p2)

    # Header Panel 2
    h2 = patches.FancyBboxPatch((55, 61), 81, 5, boxstyle="round,pad=0.2,rounding_size=0.6",
                                facecolor=C_TEAL, edgecolor=C_TEAL, linewidth=1.0, zorder=3)
    ax.add_patch(h2)
    ax.text(95.5, 63.5, "KERANGKA ACUAN BERGERAK BODI {b} (FRD) & 6-DOF SNAME (1950)",
            ha='center', va='center', fontsize=9.2, fontweight='bold', color='white', zorder=4)

    # Draw Submarine/AUV Hull in Center of Right Panel
    ob_x, ob_y = 92, 38

    # Water background tint in right panel
    hull_bg = patches.Ellipse((ob_x, ob_y), 50, 24, facecolor='#F0F9FF', edgecolor='none', zorder=3)
    ax.add_patch(hull_bg)

    # Main AUV Pressure Hull (Cylinder + End caps)
    hull_cyl = patches.FancyBboxPatch((ob_x - 14, ob_y - 4.5), 28, 9, boxstyle="round,pad=0.2,rounding_size=2.5",
                                     facecolor='#E2E8F0', edgecolor='#475569', linewidth=2.0, zorder=4)
    ax.add_patch(hull_cyl)

    # Acrylic dome at front (Surge / Bow)
    front_dome = patches.Wedge((ob_x + 14, ob_y), 4.5, -90, 90, facecolor='#BAE6FD', edgecolor='#0284C7', linewidth=1.8, zorder=5)
    ax.add_patch(front_dome)

    # Thrusters illustration (4 horizontal corners)
    th_coords = [
        (ob_x + 9, ob_y + 7, 45),
        (ob_x - 9, ob_y + 7, -45),
        (ob_x + 9, ob_y - 7, -45),
        (ob_x - 9, ob_y - 7, 45)
    ]
    for tx, ty, ang in th_coords:
        th_box = patches.FancyBboxPatch((tx - 2.5, ty - 1.2), 5, 2.4, boxstyle="round,pad=0.1,rounding_size=0.4",
                                       facecolor='#334155', edgecolor='#0F172A', linewidth=1.0, zorder=6)
        ax.add_patch(th_box)

    # Center of Origin (CO)
    ax.scatter(ob_x, ob_y, s=80, color=C_NAVY, zorder=7)
    ax.text(ob_x - 2.5, ob_y + 2.5, r"$\mathbf{O}_b$ (CO)", fontsize=9.5, fontweight='bold', color=C_NAVY, zorder=8)

    # Center of Buoyancy (CB) & Center of Gravity (CG)
    cb_y = ob_y + 2.0
    cg_y = ob_y - 2.5
    ax.scatter(ob_x, cb_y, s=40, color='#0284C7', marker='^', zorder=7)
    ax.text(ob_x + 2.0, cb_y, r"$\mathbf{r}_b$ (CB)", fontsize=8.0, fontweight='bold', color='#0284C7', zorder=8)

    ax.scatter(ob_x, cg_y, s=40, color='#DC2626', marker='v', zorder=7)
    ax.text(ob_x + 2.0, cg_y, r"$\mathbf{r}_g$ (CG)", fontsize=8.0, fontweight='bold', color='#DC2626', zorder=8)

    # 1. Axis x_b (Surge / Forward)
    arr_xb = patches.FancyArrowPatch((ob_x, ob_y), (ob_x + 28, ob_y), arrowstyle="-|>",
                                     mutation_scale=15, linewidth=2.4, color=C_ACCENT_RED, zorder=8)
    ax.add_patch(arr_xb)
    ax.text(ob_x + 29.5, ob_y, r"$\mathbf{x}_b$" + "\n(Surge / Haluan)\n" + r"$u, X$", ha='left', va='center',
            fontsize=8.5, fontweight='bold', color=C_ACCENT_RED, zorder=9)

    # Roll rate p, moment K arc around x_b
    arc_roll = patches.Arc((ob_x + 22, ob_y), 6, 6, angle=0, theta1=40, theta2=320,
                           color=C_ACCENT_RED, linewidth=1.8, zorder=8)
    ax.add_patch(arc_roll)
    ax.annotate('', xy=(ob_x + 25, ob_y + 1.8), xytext=(ob_x + 24.8, ob_y + 2.2),
                arrowprops=dict(arrowstyle="->", color=C_ACCENT_RED, lw=1.8), zorder=9)
    ax.text(ob_x + 22, ob_y + 4.5, r"Roll ($p, K, \phi$)", ha='center', va='bottom',
            fontsize=7.8, fontweight='bold', color=C_ACCENT_RED, zorder=9)

    # 2. Axis y_b (Sway / Starboard)
    arr_yb = patches.FancyArrowPatch((ob_x, ob_y), (ob_x, ob_y + 16), arrowstyle="-|>",
                                     mutation_scale=15, linewidth=2.4, color=C_ACCENT_GREEN, zorder=8)
    ax.add_patch(arr_yb)
    ax.text(ob_x, ob_y + 17.5, r"$\mathbf{y}_b$ (Sway / Lambung Kanan) | $v, Y$", ha='center', va='bottom',
            fontsize=8.5, fontweight='bold', color=C_ACCENT_GREEN, zorder=9)

    # Pitch rate q, moment M arc around y_b
    arc_pitch = patches.Arc((ob_x, ob_y + 11), 6, 6, angle=90, theta1=40, theta2=320,
                            color=C_ACCENT_GREEN, linewidth=1.8, zorder=8)
    ax.add_patch(arc_pitch)
    ax.annotate('', xy=(ob_x - 1.8, ob_y + 14), xytext=(ob_x - 2.2, ob_y + 13.8),
                arrowprops=dict(arrowstyle="->", color=C_ACCENT_GREEN, lw=1.8), zorder=9)
    ax.text(ob_x - 4.5, ob_y + 11, r"Pitch ($q, M, \theta$)", ha='right', va='center',
            fontsize=7.8, fontweight='bold', color=C_ACCENT_GREEN, zorder=9)

    # 3. Axis z_b (Heave / Down / Keel)
    arr_zb = patches.FancyArrowPatch((ob_x, ob_y), (ob_x - 16, ob_y - 16), arrowstyle="-|>",
                                     mutation_scale=15, linewidth=2.4, color=C_ACCENT_BLUE, zorder=8)
    ax.add_patch(arr_zb)
    ax.text(ob_x - 17.5, ob_y - 17.5, r"$\mathbf{z}_b$" + "\n(Heave / Lunas Bawah)\n" + r"$w, Z$", ha='right', va='top',
            fontsize=8.5, fontweight='bold', color=C_ACCENT_BLUE, zorder=9)

    # Yaw rate r, moment N arc around z_b
    arc_yaw = patches.Arc((ob_x - 9, ob_y - 9), 6, 6, angle=225, theta1=40, theta2=320,
                          color=C_ACCENT_BLUE, linewidth=1.8, zorder=8)
    ax.add_patch(arc_yaw)
    ax.annotate('', xy=(ob_x - 11, ob_y - 7.5), xytext=(ob_x - 11.4, ob_y - 7.8),
                arrowprops=dict(arrowstyle="->", color=C_ACCENT_BLUE, lw=1.8), zorder=9)
    ax.text(ob_x - 11, ob_y - 5.0, r"Yaw ($r, N, \psi$)", ha='center', va='bottom',
            fontsize=7.8, fontweight='bold', color=C_ACCENT_BLUE, zorder=9)

    # Legend / Summary Table inside Right Panel Bottom
    t_sum = patches.FancyBboxPatch((57, 5.5), 77, 9.5, boxstyle="round,pad=0.2,rounding_size=0.5",
                                  facecolor='#F1F5F9', edgecolor='#CBD5E1', linewidth=1.0, zorder=3)
    ax.add_patch(t_sum)
    ax.text(59, 12.5, "Ringkasan Notasi SNAME 6-DOF:", fontsize=7.8, fontweight='bold', color=C_NAVY, zorder=4)
    ax.text(59, 9.8, r"• Kecepatan Linier / Sudut ($\mathbf{\nu}$): $\mathbf{\nu}_1 = [u, v, w]^T$ (m/s), $\mathbf{\nu}_2 = [p, q, r]^T$ (rad/s)",
            fontsize=7.4, color='#334155', zorder=4)
    ax.text(59, 7.2, r"• Gaya & Momen Aktuasi ($\mathbf{\tau}$): $\mathbf{\tau}_1 = [X, Y, Z]^T$ (N), $\mathbf{\tau}_2 = [K, M, N]^T$ (N$\cdot$m)",
            fontsize=7.4, color='#334155', zorder=4)

    plt.tight_layout()
    out_path = '/home/radhi/Documents/AUV Development/Thesis/figures/sname_fossen_coordinate_system.png'
    plt.savefig(out_path, dpi=300, bbox_inches='tight', facecolor='white', edgecolor='none')
    plt.close()
    print("Generated SNAME diagram successfully:", out_path)

if __name__ == '__main__':
    generate_sname_diagram()
