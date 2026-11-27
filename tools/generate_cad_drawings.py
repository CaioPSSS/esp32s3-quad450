#!/usr/bin/env python3
"""
Gerador Determinístico de Desenhos Técnicos Estilo CAD & Esquemáticos (v2.1) — ESP32-S3 Quad-450.

Substitui todos os diagramas ASCII da documentação por pranchas técnicas de engenharia
com coordenadas milimétricas exatas, linhas de cota ISO, cortes ortogonais, esquemáticos
elétricos vetoriais reais e carimbo de revisão reproduzíveis por código.

Uso:
    python3 tools/generate_cad_drawings.py
"""

import math
import os
import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch
from matplotlib.transforms import Affine2D

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
OUT_DIR = os.path.join(REPO_ROOT, "docs", "assets")

# Paleta de Engenharia CAD de Alto Contraste
C_BG       = "#FFFFFF"
C_SHEET    = "#F8FAFC"
C_LINE     = "#0F172A"
C_SUB      = "#475569"
C_GRID     = "#E2E8F0"
C_DIM      = "#0369A1"
C_CENTER   = "#64748B"
C_WOOD     = "#E6C280"
C_WOOD_EDG = "#78350F"
C_ALUM     = "#CBD5E1"
C_ALUM_EDG = "#334155"
C_PCB      = "#14532D"
C_PCB_FILL = "#DCFCE7"
C_ESP      = "#1E293B"
C_OK       = "#15803D"
C_OK_BG    = "#DCFCE7"
C_WARN     = "#B91C1C"
C_WARN_BG  = "#FEE2E2"
C_ACCENT   = "#1D4ED8"


def setup_cad_ax(ax, xlim, ylim, grid_step=10.0, equal=True):
    ax.set_facecolor(C_BG)
    if equal:
        ax.set_aspect("equal")
    ax.set_xlim(*xlim)
    ax.set_ylim(*ylim)
    if grid_step:
        tx0 = math.ceil(xlim[0] / grid_step) * grid_step
        tx1 = math.floor(xlim[1] / grid_step) * grid_step
        ty0 = math.ceil(ylim[0] / grid_step) * grid_step
        ty1 = math.floor(ylim[1] / grid_step) * grid_step
        ax.set_xticks(np.arange(tx0, tx1 + 0.1, grid_step))
        ax.set_yticks(np.arange(ty0, ty1 + 0.1, grid_step))
        ax.grid(True, which="major", linestyle=":", linewidth=0.65, color=C_GRID, zorder=0)
    ax.tick_params(labelsize=8, colors=C_SUB)
    for spine in ax.spines.values():
        spine.set_color("#94A3B8")
        spine.set_linewidth(1.0)


def draw_centerlines(ax, cx, cy, half_w, half_h, zorder=2):
    ax.plot([cx - half_w, cx + half_w], [cy, cy], linestyle=(0, (8, 3, 2, 3)),
            linewidth=0.85, color=C_CENTER, zorder=zorder)
    ax.plot([cx, cx], [cy - half_h, cy + half_h], linestyle=(0, (8, 3, 2, 3)),
            linewidth=0.85, color=C_CENTER, zorder=zorder)


def draw_dim_h(ax, x1, x2, y_ref, y_dim, text, color=C_DIM, fontsize=8.3, text_dy=1.6):
    ax.plot([x1, x1], [y_ref, y_dim], color=color, linewidth=0.75, alpha=0.75, zorder=8)
    ax.plot([x2, x2], [y_ref, y_dim], color=color, linewidth=0.75, alpha=0.75, zorder=8)
    arr = FancyArrowPatch((x1, y_dim), (x2, y_dim), arrowstyle="<|-|>",
                          mutation_scale=9, linewidth=1.1, color=color, zorder=9)
    ax.add_patch(arr)
    va = "bottom" if text_dy >= 0 else "top"
    ax.text((x1 + x2) * 0.5, y_dim + text_dy, text, ha="center", va=va,
            fontsize=fontsize, fontweight="bold", color=color,
            bbox=dict(boxstyle="round,pad=0.15", facecolor=C_BG, edgecolor="none", alpha=0.92),
            zorder=10)


def draw_dim_v(ax, x_ref, x_dim, y1, y2, text, color=C_DIM, fontsize=8.3, text_dx=1.8, rot=90):
    ax.plot([x_ref, x_dim], [y1, y1], color=color, linewidth=0.75, alpha=0.75, zorder=8)
    ax.plot([x_ref, x_dim], [y2, y2], color=color, linewidth=0.75, alpha=0.75, zorder=8)
    arr = FancyArrowPatch((x_dim, y1), (x_dim, y2), arrowstyle="<|-|>",
                          mutation_scale=9, linewidth=1.1, color=color, zorder=9)
    ax.add_patch(arr)
    ha = "left" if text_dx >= 0 else "right"
    ax.text(x_dim + text_dx, (y1 + y2) * 0.5, text, ha=ha, va="center", rotation=rot,
            fontsize=fontsize, fontweight="bold", color=color,
            bbox=dict(boxstyle="round,pad=0.15", facecolor=C_BG, edgecolor="none", alpha=0.92),
            zorder=10)


def add_title_banner(fig, sheet_code, title, subtitle):
    fig.patch.set_facecolor(C_SHEET)
    fig.text(0.02, 0.966, f"[{sheet_code}]  {title}",
             fontsize=11.0, fontweight="bold", color=C_LINE, va="top")
    fig.text(0.02, 0.933, subtitle,
             fontsize=8.9, color=C_SUB, va="top")
    fig.text(0.98, 0.966, "ESP32-S3 QUAD-450  |  REV v2.1  |  ESCALA COTADA (mm)",
             fontsize=8.2, fontweight="bold", color=C_DIM, ha="right", va="top",
             bbox=dict(boxstyle="round,pad=0.28", facecolor="#E0F2FE", edgecolor="#7DD3FC", lw=0.9))


# Primitivas Vetoriais de Esquemático Elétrico (IEC/IEEE)
def draw_resistor_h(ax, cx, cy, w, h, label, sublabel="", color=C_LINE):
    ax.add_patch(patches.Rectangle((cx - w / 2, cy - h / 2), w, h,
                                   facecolor="#FEF9C3", edgecolor=color, lw=1.4, zorder=6))
    ax.text(cx, cy + h / 2 + 1.2, label, ha="center", va="bottom", fontsize=7.3, fontweight="bold", color=color)
    if sublabel:
        ax.text(cx, cy - h / 2 - 1.2, sublabel, ha="center", va="top", fontsize=6.6, color=C_SUB)


def draw_resistor_v(ax, cx, cy, w, h, label, color=C_LINE):
    ax.add_patch(patches.Rectangle((cx - w / 2, cy - h / 2), w, h,
                                   facecolor="#FEF9C3", edgecolor=color, lw=1.4, zorder=6))
    ax.text(cx + w / 2 + 1.5, cy, label, ha="left", va="center", fontsize=7.2, fontweight="bold", color=color)


def draw_cap_v(ax, cx, cy_top, cy_bot, label, color=C_LINE):
    ym = (cy_top + cy_bot) * 0.5
    gap = 1.4
    ax.plot([cx, cx], [cy_top, ym + gap], color=color, lw=1.4, zorder=5)
    ax.plot([cx - 3.6, cx + 3.6], [ym + gap, ym + gap], color=color, lw=2.0, zorder=6)
    ax.plot([cx - 3.6, cx + 3.6], [ym - gap, ym - gap], color=color, lw=2.0, zorder=6)
    ax.plot([cx, cx], [ym - gap, cy_bot], color=color, lw=1.4, zorder=5)
    ax.text(cx + 4.4, ym, label, ha="left", va="center", fontsize=6.9, fontweight="bold", color=C_ACCENT)


def draw_schottky_h(ax, x_in, x_out, y, label="1N5819"):
    xm = (x_in + x_out) * 0.5
    ax.plot([x_in, xm - 3.5], [y, y], color="#DC2626", lw=1.6)
    tri = patches.Polygon([[xm - 3.5, y + 3.5], [xm - 3.5, y - 3.5], [xm + 3.0, y]],
                          closed=True, facecolor="#FCA5A5", edgecolor=C_LINE, lw=1.3, zorder=6)
    ax.add_patch(tri)
    # Barra Schottky em S
    ax.plot([xm + 3.0, xm + 3.0], [y - 3.8, y + 3.8], color=C_LINE, lw=1.8, zorder=7)
    ax.plot([xm + 3.0, xm + 4.4], [y + 3.8, y + 3.8], color=C_LINE, lw=1.3, zorder=7)
    ax.plot([xm + 1.6, xm + 3.0], [y - 3.8, y - 3.8], color=C_LINE, lw=1.3, zorder=7)
    ax.plot([xm + 3.0, x_out], [y, y], color="#DC2626", lw=1.6)
    ax.text(xm, y + 4.8, label, ha="center", va="bottom", fontsize=7.2, fontweight="bold", color=C_WARN)


def draw_gnd(ax, cx, y_top):
    ax.plot([cx, cx], [y_top, y_top - 2.5], color=C_LINE, lw=1.4)
    for dy, hw in [(2.5, 4.2), (4.2, 2.7), (5.9, 1.3)]:
        ax.plot([cx - hw, cx + hw], [y_top - dy, y_top - dy], color=C_LINE, lw=1.5)


def chamfered_square_pts(half_s, ch):
    s, c = half_s, ch
    return np.array([
        [-s + c,  s], [ s - c,  s], [ s,  s - c], [ s, -s + c],
        [ s - c, -s], [-s + c, -s], [-s, -s + c], [-s,  s - c]
    ])


# =============================================================================
# PRANCHA 00: Visão Geral do Quad-X 450 mm + Corte Lateral do Sanduíche Central
# =============================================================================
def generate_cad_00_drone_overview():
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15.2, 7.5), dpi=180,
                                   gridspec_kw={"width_ratios": [1.10, 1.0]})
    add_title_banner(
        fig, "CAD-00",
        "PLANTA GEOMÉTRICA DO QUADRICÓPTERO 450 mm (QUAD-X FRD) & CORTE DE EMPILHAMENTO",
        "Esquerda: Vista superior em escala real c/ discos das hélices 1045 (Ø254 mm) e zona central livre  |  Direita: Elevação ortogonal Z-Stack"
    )

    setup_cad_ax(ax1, (-318, 318), (-318, 318), grid_step=50.0)
    draw_centerlines(ax1, 0, 0, 295, 295)

    free_circ = patches.Circle((0, 0), 98.0, linestyle="--", linewidth=1.3,
                               edgecolor=C_OK, facecolor="#F0FDF4", alpha=0.55, zorder=1)
    ax1.add_patch(free_circ)
    ax1.text(0, -83, "Zona Central Livre\nR = 98,0 mm (Ø 196 mm)", ha="center", va="center",
             fontsize=7.5, fontweight="bold", color=C_OK, zorder=6)

    motors = [
        ( 45.0, "M1 (FR)", "CCW ↺", "#0284C7", ( 159.1,  204.0)),
        (315.0, "M2 (RR)", "CW ↻",  "#D97706", ( 159.1, -204.0)),
        (225.0, "M3 (RL)", "CCW ↺", "#0284C7", (-159.1, -204.0)),
        (135.0, "M4 (FL)", "CW ↻",  "#D97706", (-159.1,  204.0)),
    ]
    for ang_deg, m_name, spin_lbl, prop_col, (lx, ly) in motors:
        rad = math.radians(ang_deg)
        arm = patches.Rectangle((20.0, -7.5), 225.0, 15.0, facecolor=C_WOOD,
                                edgecolor=C_WOOD_EDG, linewidth=1.4, zorder=3)
        arm.set_transform(Affine2D().rotate_deg(ang_deg) + ax1.transData)
        ax1.add_patch(arm)

        mx, my = 225.0 * math.cos(rad), 225.0 * math.sin(rad)
        prop = patches.Circle((mx, my), 127.0, facecolor=prop_col, alpha=0.11,
                              edgecolor=prop_col, linewidth=1.6, zorder=2)
        ax1.add_patch(prop)
        ax1.add_patch(patches.Circle((mx, my), 14.0, facecolor="#334155", edgecolor=C_LINE, lw=1.2, zorder=5))
        ax1.add_patch(patches.Circle((mx, my), 3.0, facecolor="#F8FAFC", edgecolor=C_LINE, lw=0.8, zorder=6))

        arc_r = 38.0
        arc = patches.Arc((mx, my), 2 * arc_r, 2 * arc_r, angle=0, theta1=200, theta2=340,
                          color=prop_col, linewidth=1.8, zorder=6) if my > 0 else \
              patches.Arc((mx, my), 2 * arc_r, 2 * arc_r, angle=0, theta1=20, theta2=160,
                          color=prop_col, linewidth=1.8, zorder=6)
        ax1.add_patch(arc)
        ax1.text(lx, ly, f"{m_name}\n{spin_lbl}", ha="center", va="center",
                 fontsize=8.0, fontweight="bold", color=C_LINE,
                 bbox=dict(boxstyle="round,pad=0.20", facecolor=C_BG, edgecolor=prop_col, lw=1.1), zorder=9)

    plate_poly = patches.Polygon(chamfered_square_pts(60.0, 10.0), closed=True,
                                 facecolor="#FEF3C7", edgecolor=C_WOOD_EDG, linewidth=1.8, zorder=4)
    ax1.add_patch(plate_poly)
    fc_box = patches.Rectangle((-35.0, -45.0), 70.0, 90.0, facecolor=C_PCB_FILL,
                               edgecolor=C_PCB, linewidth=1.4, zorder=5)
    ax1.add_patch(fc_box)
    ax1.text(0, 0, "FC\n70×90", ha="center", va="center", fontsize=7.5, fontweight="bold", color=C_PCB, zorder=7)

    ax1.annotate("FRENTE (+X FRD)", xy=(0, 286), xytext=(0, 232),
                 ha="center", va="center", fontsize=8.8, fontweight="bold", color=C_WARN,
                 arrowprops=dict(arrowstyle="-|>", lw=2.0, color=C_WARN), zorder=10)

    draw_dim_h(ax1, -159.1, 159.1, 159.1, 296.0, "L_adj = 318,2 mm (Centro a Centro)", text_dy=2.5)
    draw_dim_v(ax1, 159.1, 294.0, -32.1, 32.1, "Folga = 64,2 mm", color=C_OK, text_dx=4.0)

    diag = FancyArrowPatch((-159.1, -159.1), (159.1, 159.1), arrowstyle="<|-|>",
                           mutation_scale=10, linewidth=1.2, linestyle="--", color=C_ACCENT, zorder=8)
    ax1.add_patch(diag)
    ax1.text(-92, -74, "Diagonal D = 450,0 mm", rotation=45, ha="center", va="center",
             fontsize=8.0, fontweight="bold", color=C_ACCENT,
             bbox=dict(boxstyle="round,pad=0.16", facecolor=C_BG, edgecolor="none", alpha=0.92), zorder=10)

    ax1.set_title("A. Planta Superior em Escala (Quad-X 450 mm)", fontsize=10.2, fontweight="bold", pad=8)
    ax1.set_xlabel("Largura Esquerda–Direita (mm)")
    ax1.set_ylabel("Comprimento Traseira–Frente (mm)")

    # --- Painel Direito: Corte Ortogonal Lateral Limpo (Zero Colisão de Textos!) ---
    ax2.set_facecolor(C_BG)
    ax2.set_xlim(-150, 155)
    ax2.set_ylim(-58, 92)
    ax2.set_xticks(np.arange(-150, 151, 50))
    ax2.set_yticks(np.arange(-40, 81, 20))
    ax2.grid(True, linestyle=":", linewidth=0.65, color=C_GRID, zorder=0)
    for sp in ax2.spines.values():
        sp.set_color("#94A3B8")

    ax2.axhline(-48, color="#64748B", linewidth=1.8, zorder=2)
    ax2.fill_between([-150, 155], -58, -48, color="#F1F5F9", hatch="///", edgecolor="#CBD5E1", zorder=1)
    ax2.text(-144, -54.5, "SOLO / PISTA DE POUSO", fontsize=8, fontweight="bold", color=C_SUB)

    for sx in [-110, 110]:
        ax2.add_patch(patches.Circle((sx, -32), 16.0, facecolor="#94A3B8", edgecolor=C_LINE, lw=1.2, alpha=0.65, zorder=3))
        ax2.add_patch(patches.Circle((sx, -32), 7.0, facecolor=C_BG, edgecolor=C_LINE, lw=0.9, zorder=4))
    ax2.text(110, -11, "Pés Espuma\nIsolante Ø32", ha="center", fontsize=7.5, color=C_SUB, fontweight="bold")

    ax2.add_patch(patches.Rectangle((-52, -29), 104, 26, facecolor="#DBEAFE", edgecolor="#1D4ED8", lw=1.5, zorder=4))
    ax2.text(0, -16, "LiPo 3S 2200–3000 mAh (188 g)\nVelcro + EVA no CG (0,0)",
             ha="center", va="center", fontsize=7.9, fontweight="bold", color="#1E3A8A", zorder=6)
    ax2.add_patch(patches.Rectangle((-52, -3), 104, 3, facecolor="#334155", edgecolor=C_LINE, lw=0.8, zorder=4))

    ax2.add_patch(patches.Rectangle((-60, 0), 120, 4, facecolor=C_WOOD, edgecolor=C_WOOD_EDG, lw=1.3, zorder=5))
    ax2.add_patch(patches.Rectangle((-130, 4), 260, 15, facecolor="#FDE68A", edgecolor=C_WOOD_EDG, lw=1.4, zorder=4))
    ax2.text(-90, 11.5, "Braço 15×15 mm\n(ou Al 12,7 mm)", ha="center", va="center", fontsize=7.4, fontweight="bold", color=C_WOOD_EDG)
    ax2.add_patch(patches.Rectangle((-60, 19), 120, 4, facecolor=C_WOOD, edgecolor=C_WOOD_EDG, lw=1.3, zorder=5))

    for bx in [-46, -24, 24, 46]:
        ax2.plot([bx, bx], [-3, 25], color=C_LINE, lw=1.5, zorder=6)
        ax2.add_patch(patches.Rectangle((bx - 3, 23), 6, 2.0, facecolor=C_SUB, edgecolor=C_LINE, lw=0.8, zorder=7))
        ax2.add_patch(patches.Rectangle((bx - 3.2, -5.5), 6.4, 2.5, facecolor="#475569", edgecolor=C_LINE, lw=0.8, zorder=7))

    for sx in [-20, 20]:
        ax2.add_patch(patches.Rectangle((sx - 1.8, 23), 3.6, 6, facecolor="#E2E8F0", edgecolor=C_SUB, lw=0.9, zorder=5))
    ax2.add_patch(patches.Rectangle((-25, 29), 50, 2.8, facecolor="#991B1B", edgecolor=C_LINE, lw=1.1, zorder=6))

    for dx in [-41, 41]:
        ax2.add_patch(FancyBboxPatch((dx - 3.8, 24), 7.6, 14, boxstyle="round,pad=0.8",
                                     facecolor="#1E293B", edgecolor=C_LINE, lw=1.1, zorder=6))

    ax2.add_patch(patches.Rectangle((-45, 39), 90, 2.2, facecolor="#16A34A", edgecolor=C_PCB, lw=1.4, zorder=7))
    ax2.add_patch(patches.Rectangle((-8, 41.2), 16, 4.5, facecolor="#0284C7", edgecolor=C_LINE, lw=1.1, zorder=8))
    ax2.add_patch(patches.Rectangle((-37, 41.2), 22, 9.5, facecolor=C_ESP, edgecolor=C_LINE, lw=1.1, zorder=8))
    ax2.text(-26, 46.0, "ESP32-S3", ha="center", va="center", fontsize=6.9, fontweight="bold", color="#FFFFFF", zorder=9)

    # Setas de chamada limpas sem sobreposição!
    ax2.annotate("IMU ICM-42688-P\nExata no (0,0)!", xy=(2, 45.8), xytext=(32, 62),
                 ha="center", fontsize=7.8, fontweight="bold", color="#0369A1",
                 bbox=dict(boxstyle="round,pad=0.22", facecolor="#E0F2FE", edgecolor="#0284C7", lw=0.9),
                 arrowprops=dict(arrowstyle="-|>", color="#0284C7", lw=1.3), zorder=10)
    ax2.annotate("4× Dampers M3\n(±22, ±41 mm)", xy=(-45, 31), xytext=(-106, 33),
                 ha="center", fontsize=7.5, fontweight="bold", color=C_OK,
                 bbox=dict(boxstyle="round,pad=0.22", facecolor=C_OK_BG, edgecolor=C_OK, lw=0.9),
                 arrowprops=dict(arrowstyle="-|>", color=C_OK, lw=1.3), zorder=10)
    ax2.annotate("Mini PDB-XT60\n+ Cap 1000µF", xy=(25, 30.4), xytext=(95, 31),
                 ha="center", fontsize=7.5, fontweight="bold", color="#991B1B",
                 bbox=dict(boxstyle="round,pad=0.22", facecolor="#FEF2F2", edgecolor="#991B1B", lw=0.9),
                 arrowprops=dict(arrowstyle="-|>", color="#991B1B", lw=1.2), zorder=10)

    ax2.plot([-72, -72], [23, 82], color=C_LINE, lw=3.0, solid_capstyle="round", zorder=7)
    ax2.add_patch(patches.Circle((-72, 83), 2.2, color=C_WARN, zorder=8))
    ax2.text(-77, 68, "Antena 2.4 GHz\nVertical 90°\n(Longe da LiPo!)", ha="right", fontsize=7.7, fontweight="bold", color=C_LINE)

    draw_dim_v(ax2, 60, 140, 0, 23, "Sanduíche 23 mm", text_dx=2.5, rot=90)
    draw_dim_v(ax2, 45, 122, 23, 41.2, "Isolação 18 mm", color=C_OK, text_dx=2.5, rot=90)

    ax2.set_title("B. Corte Ortogonal Lateral — Empilhamento em 3 Camadas", fontsize=10.2, fontweight="bold", pad=8)
    ax2.set_xlabel("Eixo Longitudinal Traseira (–) ↔ Frente (+) (mm)")
    ax2.set_ylabel("Cota Vertical Z (mm)")

    fig.tight_layout(rect=[0.01, 0.01, 0.99, 0.90])
    out = os.path.join(OUT_DIR, "cad_00_drone_overview_450.png")
    fig.savefig(out)
    plt.close(fig)
    return out


# =============================================================================
# PRANCHA 01: Inspeção de Antena IPEX (WROOM-1U vs YD) + SMA + Buzzer NPN KY-012
# =============================================================================
def generate_cad_01_inspecao_ipex_buzzer():
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15.0, 6.5), dpi=180,
                                   gridspec_kw={"width_ratios": [1.20, 1.0]})
    add_title_banner(
        fig, "CAD-01",
        "INSPEÇÃO DE RECEBIMENTO — CHAVEAMENTO RF IPEX (2,4 GHz), PINO SMA & BUZZER NPN",
        "Substitui diagrama textual da Etapa 0.2: Identificação visual do jumper 0 Ω nos módulos ESP32-S3, conector SMA e teste do KY-012"
    )

    ax1.set_facecolor(C_BG)
    ax1.set_xlim(0, 200)
    ax1.set_ylim(0, 115)
    ax1.set_aspect("equal")
    ax1.axis("off")

    # Caso A: WROOM-1U Original
    ax1.add_patch(FancyBboxPatch((10, 22), 76, 76, boxstyle="round,pad=1.5",
                                 facecolor="#14532D", edgecolor=C_LINE, lw=1.8))
    ax1.add_patch(patches.Rectangle((16, 26), 64, 50, facecolor="#E2E8F0", edgecolor="#475569", lw=1.5))
    ax1.text(48, 51, "BLINDAGEM RF\nESP32-S3-WROOM-1U\n(N16R8 Original)", ha="center", va="center",
             fontsize=8.0, fontweight="bold", color=C_LINE)
    ax1.add_patch(patches.Rectangle((42, 81), 12, 12, facecolor="#FACC15", edgecolor="#854D0E", lw=1.3))
    ax1.add_patch(patches.Circle((48, 87), 3.6, facecolor=C_BG, edgecolor="#854D0E", lw=1.2))
    ax1.add_patch(patches.Circle((48, 87), 1.2, facecolor="#854D0E"))
    ax1.text(48, 105, "CASO A: WROOM-1U Autêntico", ha="center", fontsize=9.3, fontweight="bold", color=C_OK)
    ax1.text(48, 11, "✔ SEM antena impressa no topo\n✔ IPEX já vem 100% ATIVO de fábrica!",
             ha="center", va="center", fontsize=7.9, fontweight="bold", color=C_OK,
             bbox=dict(boxstyle="round,pad=0.35", facecolor=C_OK_BG, edgecolor=C_OK, lw=1.1))

    # Caso B: YD-ESP32-S3 Clone
    ax1.add_patch(FancyBboxPatch((110, 22), 78, 76, boxstyle="round,pad=1.5",
                                 facecolor="#1E293B", edgecolor=C_LINE, lw=1.8))
    ax1.add_patch(patches.Rectangle((116, 26), 66, 44, facecolor="#E2E8F0", edgecolor="#475569", lw=1.5))
    ax1.text(149, 48, "BLINDAGEM RF\nPlaca YD-ESP32-S3\n(Antena Dupla)", ha="center", va="center",
             fontsize=8.0, fontweight="bold", color=C_LINE)

    meander_x = [118, 118, 125, 125, 132, 132, 139, 139, 146, 146]
    meander_y = [83,  94,  94,  85,  85,  94,  94,  85,  85,  94]
    ax1.plot(meander_x, meander_y, color="#FDE047", lw=2.2)
    ax1.text(132, 78.5, "Antena PCB", ha="center", fontsize=6.8, color="#FDE047", fontweight="bold")

    for px, lbl in [(152, "1"), (158, "2"), (164, "3")]:
        ax1.add_patch(patches.Rectangle((px - 2.1, 75), 4.2, 5.2, facecolor="#FACC15", edgecolor=C_LINE, lw=0.8))
        ax1.text(px, 72.2, lbl, ha="center", va="top", fontsize=6.5, color="#FFFFFF", fontweight="bold")
    ax1.add_patch(patches.Rectangle((157.5, 76.2), 7.0, 2.8, facecolor="#22C55E", edgecolor="#FFFFFF", lw=1.0, zorder=8))

    ax1.add_patch(patches.Rectangle((171, 80), 12, 12, facecolor="#FACC15", edgecolor="#854D0E", lw=1.3))
    ax1.add_patch(patches.Circle((177, 86), 3.6, facecolor=C_BG, edgecolor="#854D0E", lw=1.2))
    ax1.add_patch(patches.Circle((177, 86), 1.2, facecolor="#854D0E"))
    ax1.plot([166, 171], [77.6, 86], color="#FACC15", lw=1.8)

    ax1.annotate("Mover resistor 0 Ω\nde [1–2] p/ [2–3] (IPEX)!",
                 xy=(161, 80.5), xytext=(152, 103),
                 ha="center", fontsize=7.9, fontweight="bold", color=C_WARN,
                 bbox=dict(boxstyle="round,pad=0.25", facecolor=C_WARN_BG, edgecolor=C_WARN, lw=1.1),
                 arrowprops=dict(arrowstyle="-|>", color=C_WARN, lw=1.5))

    ax1.text(149, 11, "⚠ De fábrica vem em [1–2] (alcance < 50 m)\n✔ Soldar ponte [2–3] ativa 100% o IPEX!",
             ha="center", va="center", fontsize=7.8, fontweight="bold", color=C_WARN,
             bbox=dict(boxstyle="round,pad=0.35", facecolor=C_WARN_BG, edgecolor=C_WARN, lw=1.1))

    # Painel B: SMA + KY-012
    ax2.set_facecolor(C_BG)
    ax2.set_xlim(0, 165)
    ax2.set_ylim(0, 115)
    ax2.set_aspect("equal")
    ax2.axis("off")

    ax2.add_patch(FancyBboxPatch((4, 66), 157, 43, boxstyle="round,pad=1.5",
                                 facecolor="#F8FAFC", edgecolor="#94A3B8", lw=1.2))
    ax2.text(82.5, 104, "VERIFICAÇÃO MECÂNICA DO CONECTOR SMA (OBRIGATÓRIA)", ha="center",
             fontsize=8.5, fontweight="bold", color=C_DIM)
    ax2.add_patch(patches.Rectangle((14, 78), 42, 16, facecolor="#FDE047", edgecolor="#854D0E", lw=1.3))
    ax2.add_patch(patches.Rectangle((46, 84), 10, 4, facecolor=C_BG, edgecolor="#854D0E", lw=1.0))
    ax2.text(35, 72, "Pigtail SMA (Furo Fêmea)", ha="center", fontsize=7.5, fontweight="bold", color=C_LINE)

    ax2.add_patch(patches.Rectangle((108, 78), 42, 16, facecolor="#334155", edgecolor=C_LINE, lw=1.3))
    ax2.plot([95, 108], [86, 86], color="#FACC15", lw=3.2)
    ax2.text(129, 72, "Antena 2.4 GHz (Pino Macho)", ha="center", fontsize=7.5, fontweight="bold", color=C_LINE)

    ax2.annotate("", xy=(59, 86), xytext=(92, 86),
                 arrowprops=dict(arrowstyle="<|-|>", color=C_OK, lw=1.6))
    ax2.text(75.5, 92, "1 Pino × 1 Furo\n(CASAMENTO OK!)", ha="center", fontsize=7.3, fontweight="bold", color=C_OK)

    ax2.add_patch(FancyBboxPatch((4, 6), 157, 54, boxstyle="round,pad=1.5",
                                 facecolor="#F8FAFC", edgecolor="#94A3B8", lw=1.2))
    ax2.text(82.5, 55, "TESTE DE BANCADA DO BUZZER ATIVO 5V — KY-012 (NPN ACTIVE-HIGH)",
             ha="center", fontsize=8.2, fontweight="bold", color=C_LINE)

    ax2.add_patch(patches.Rectangle((12, 17), 44, 30, facecolor="#1E293B", edgecolor=C_LINE, lw=1.3))
    ax2.add_patch(patches.Circle((34, 35), 9.8, facecolor="#0F172A", edgecolor="#94A3B8", lw=1.2))
    ax2.add_patch(patches.Circle((34, 35), 2.3, facecolor="#CBD5E1"))
    ax2.text(34, 20.5, "J3Y (NPN)", ha="center", fontsize=6.8, color="#38BDF8", fontweight="bold")

    for py, plbl, pcol, dest in [
        (42, "S (I/O)", "#2563EB", "GPIO21 (Drone) / GPIO14 (TX)"),
        (33, "+ (VCC)", "#DC2626", "+5V_SYS (~4,75 V)"),
        (24, "- (GND)", "#0F172A", "Barramento GND"),
    ]:
        ax2.plot([56, 70], [py, py], color=pcol, lw=2.2)
        ax2.text(72, py, f"{plbl} → {dest}", va="center", fontsize=7.5, fontweight="bold", color=pcol)

    ax2.text(82.5, 11.2, "Teste Rápido:   S = 0V → MUDO   |   S = 3,3V → APITA FORTE!",
             ha="center", fontsize=7.6, fontweight="bold", color=C_OK,
             bbox=dict(boxstyle="round,pad=0.22", facecolor=C_OK_BG, edgecolor=C_OK, lw=0.9))

    fig.tight_layout(rect=[0.01, 0.01, 0.99, 0.90])
    out = os.path.join(OUT_DIR, "cad_01_inspecao_ipex_buzzer.png")
    fig.savefig(out)
    plt.close(fig)
    return out


# =============================================================================
# PRANCHA 02: Gabarito Mecânico Cotado da Chapa Central 120 x 120 mm (v2.1)
# =============================================================================
def generate_cad_02_chapa_central():
    fig, (ax, ax_tbl) = plt.subplots(1, 2, figsize=(15.2, 7.6), dpi=180,
                                     gridspec_kw={"width_ratios": [1.22, 0.92]})
    add_title_banner(
        fig, "CAD-02",
        "GABARITO DE CORTE E FURAÇÃO — CHAPAS CENTRAIS 120,0 × 120,0 mm (COMPENSADO 4 mm)",
        "Substitui diagrama textual da Etapa 1.1: Furar as 2 chapas empilhadas juntas c/ broca Ø 3,0–3,2 mm  |  Zero colisão dampers × braços!"
    )

    setup_cad_ax(ax, (-85, 85), (-85, 85), grid_step=10.0)
    draw_centerlines(ax, 0, 0, 78, 78)

    ax.plot([-62, 62], [-62, 62], linestyle=(0, (6, 3)), lw=0.9, color="#94A3B8", zorder=2)
    ax.plot([-62, 62], [62, -62], linestyle=(0, (6, 3)), lw=0.9, color="#94A3B8", zorder=2)

    plate = patches.Polygon(chamfered_square_pts(60.0, 10.0), closed=True,
                            facecolor="#FEF3C7", edgecolor=C_WOOD_EDG, linewidth=2.2, zorder=3)
    ax.add_patch(plate)

    for ang_deg, canto_lbl in [(45, "M1 FR (CCW)"), (315, "M2 RR (CW)"),
                               (225, "M3 RL (CCW)"), (135, "M4 FL (CW)")]:
        arm_ref = patches.Rectangle((20.0, -7.5), 62.0, 15.0, facecolor="#FDE68A", alpha=0.45,
                                    edgecolor=C_WOOD_EDG, linestyle="--", linewidth=1.2, zorder=4)
        arm_ref.set_transform(Affine2D().rotate_deg(ang_deg) + ax.transData)
        ax.add_patch(arm_ref)

        rad = math.radians(ang_deg)
        for r_mm in [32.0, 68.0]:
            hx, hy = r_mm * math.cos(rad), r_mm * math.sin(rad)
            ax.add_patch(patches.Circle((hx, hy), 3.5, fill=False, edgecolor=C_SUB, lw=0.9, zorder=6))
            ax.add_patch(patches.Circle((hx, hy), 1.6, facecolor=C_BG, edgecolor=C_LINE, lw=1.5, zorder=7))

        tx, ty = 76.0 * math.cos(rad), 76.0 * math.sin(rad)
        ax.text(tx, ty, canto_lbl, ha="center", va="center", fontsize=7.8, fontweight="bold",
                color=C_WOOD_EDG, bbox=dict(boxstyle="round,pad=0.18", facecolor=C_BG, edgecolor=C_WOOD_EDG, lw=0.9))

    ax.add_patch(patches.Rectangle((-35.0, -45.0), 70.0, 90.0, fill=False,
                                   edgecolor=C_PCB, linestyle="-.", linewidth=1.5, zorder=5))
    ax.text(0, 29, "Contorno Placa FC\n70,0 × 90,0 mm", ha="center", va="center",
            fontsize=7.5, fontweight="bold", color=C_PCB)

    ax.add_patch(patches.Circle((0, 0), 8.0, facecolor=C_BG, edgecolor=C_ACCENT, linestyle="--", lw=1.4, zorder=5))
    ax.text(0, -13.0, "Ø 16 mm (Só Chapa Sup.)", ha="center", fontsize=7.0, color=C_ACCENT, fontweight="bold")

    for sx in [-1, 1]:
        for sy in [-1, 1]:
            dx, dy = sx * 22.0, sy * 41.0
            ax.add_patch(patches.Circle((dx, dy), 3.5, facecolor=C_OK_BG, edgecolor=C_OK, lw=1.3, zorder=6))
            ax.add_patch(patches.Circle((dx, dy), 1.6, facecolor=C_OK, edgecolor=C_LINE, lw=1.0, zorder=7))

    draw_dim_h(ax, -60.0, 60.0, 60.0, 74.0, "120,0 mm (Chapa Externa)", text_dy=1.5)
    draw_dim_v(ax, 60.0, 74.0, -60.0, 60.0, "120,0 mm", text_dx=1.8)
    draw_dim_h(ax, -22.0, 22.0, -41.0, -68.5, "44,0 mm (X = ±22,0 mm Dampers FC)", color=C_OK, text_dy=-1.8)
    draw_dim_v(ax, -22.0, -70.5, -41.0, 41.0, "82,0 mm (Y = ±41,0 mm)", color=C_OK, text_dx=-2.0)

    ax.annotate("r₁ = 32,0 mm", xy=(22.63, 22.63), xytext=(36, 7),
                fontsize=7.7, fontweight="bold", color=C_LINE,
                arrowprops=dict(arrowstyle="->", lw=1.1, color=C_LINE))
    ax.annotate("r₂ = 68,0 mm", xy=(48.08, 48.08), xytext=(45, 22),
                fontsize=7.7, fontweight="bold", color=C_LINE,
                arrowprops=dict(arrowstyle="->", lw=1.1, color=C_LINE))

    ax.annotate("FRENTE ↑ (+Y)", xy=(0, 60), xytext=(0, 49),
                ha="center", fontsize=8.4, fontweight="bold", color=C_WARN,
                arrowprops=dict(arrowstyle="-|>", lw=1.8, color=C_WARN))

    ax.set_xlabel("Eixo X — Largura Esquerda (–) / Direita (+) [mm]")
    ax.set_ylabel("Eixo Y — Traseira (–) / Frente (+) [mm]")

    ax_tbl.axis("off")
    rows = [
        ["Furo / Elemento", "Qtd", "Raio / Coord. (X, Y) [mm]", "Broca"],
        ["Braço M1 FR — Int.", "1", "r = 32,0 → (+22,6 , +22,6)", "Ø 3,2 mm"],
        ["Braço M1 FR — Ext.", "1", "r = 68,0 → (+48,1 , +48,1)", "Ø 3,2 mm"],
        ["Braço M2 RR — Int.", "1", "r = 32,0 → (+22,6 , -22,6)", "Ø 3,2 mm"],
        ["Braço M2 RR — Ext.", "1", "r = 68,0 → (+48,1 , -48,1)", "Ø 3,2 mm"],
        ["Braço M3 RL — Int.", "1", "r = 32,0 → (-22,6 , -22,6)", "Ø 3,2 mm"],
        ["Braço M3 RL — Ext.", "1", "r = 68,0 → (-48,1 , -48,1)", "Ø 3,2 mm"],
        ["Braço M4 FL — Int.", "1", "r = 32,0 → (-22,6 , +22,6)", "Ø 3,2 mm"],
        ["Braço M4 FL — Ext.", "1", "r = 68,0 → (-48,1 , +48,1)", "Ø 3,2 mm"],
        ["Damper FC Diant-Esq", "1", "X = -22,0  |  Y = +41,0", "Ø 3,2 mm"],
        ["Damper FC Diant-Dir", "1", "X = +22,0  |  Y = +41,0", "Ø 3,2 mm"],
        ["Damper FC Tras-Esq",  "1", "X = -22,0  |  Y = -41,0", "Ø 3,2 mm"],
        ["Damper FC Tras-Dir",  "1", "X = +22,0  |  Y = -41,0", "Ø 3,2 mm"],
        ["Passa-Cabos Central", "1*", "Centro (0,0) — Só Sup.", "Ø 16,0 mm"],
    ]
    tbl = ax_tbl.table(cellText=rows[1:], colLabels=rows[0], loc="upper center",
                       colWidths=[0.30, 0.08, 0.44, 0.18],
                       cellLoc="center", bbox=[0.0, 0.23, 1.0, 0.74])
    tbl.auto_set_font_size(False)
    tbl.set_fontsize(7.6)
    for (r, c), cell in tbl.get_celld().items():
        cell.set_edgecolor("#CBD5E1")
        if r == 0:
            cell.set_facecolor("#0F172A")
            cell.get_text().set_color("#FFFFFF")
            cell.get_text().set_weight("bold")
        elif 9 <= r <= 12:
            cell.set_facecolor("#F0FDF4")
        elif r % 2 == 0:
            cell.set_facecolor("#F8FAFC")

    note = (
        "NOTA DE PRECISÃO SEM MÁQUINA CNC:\n"
        "1. Empilhe as 2 chapas 120×120 mm com fita crepe firme nas bordas.\n"
        "2. Trace o 'X' entre os cantos; marque r = 32,0 mm e r = 68,0 mm na régua.\n"
        "3. Folga perpendicular real Damper ↔ Braço: d⊥ = 13,44 mm\n"
        "   (+2,44 mm livre na madeira 15 mm  |  +3,59 mm no alumínio 12,7 mm!)."
    )
    ax_tbl.text(0.5, 0.10, note, ha="center", va="center", fontsize=7.9, color=C_LINE,
                bbox=dict(boxstyle="round,pad=0.4", facecolor="#E0F2FE", edgecolor="#0284C7", lw=1.1))

    fig.tight_layout(rect=[0.01, 0.01, 0.99, 0.90])
    out = os.path.join(OUT_DIR, "cad_02_gabarito_chapa_central_120x120.png")
    fig.savefig(out)
    plt.close(fig)
    return out


# =============================================================================
# PRANCHA 03: Gabarito dos 4 Braços (225 mm) + Detalhe Parafuso do Motor A2212
# =============================================================================
def generate_cad_03_braco_e_motor():
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(15.0, 7.6), dpi=180,
                                   gridspec_kw={"height_ratios": [1.15, 1.0]})
    add_title_banner(
        fig, "CAD-03",
        "GABARITO DOS 4 BRAÇOS (225,0 mm) & DETALHE DE PENETRAÇÃO DO PARAFUSO NO MOTOR A2212",
        "Substitui diagrama textual das Etapas 1.2 e 1.3: Madeira maciça 15×15 mm ou Tubo Alumínio 1/2\" (12,7 mm) c/ bucha interna de 55 mm"
    )

    setup_cad_ax(ax1, (-15, 248), (-30, 34), grid_step=15.0, equal=True)
    ax1.plot([-8, 235], [0, 0], linestyle=(0, (8, 3, 2, 3)), lw=0.9, color=C_CENTER, zorder=2)

    ax1.add_patch(patches.Rectangle((0.0, -7.5), 225.0, 15.0, facecolor=C_WOOD,
                                    edgecolor=C_WOOD_EDG, linewidth=2.0, zorder=3))
    ax1.add_patch(patches.Rectangle((0.0, -5.35), 55.0, 10.7, facecolor="#FDE68A",
                                    edgecolor="#B45309", linestyle="--", hatch="//", alpha=0.55, lw=1.1, zorder=4))
    ax1.text(27.5, -14.0, "Bucha Interna Madeira Dura 55 mm\n(Obrigatória no tubo de Alumínio 1/2\")",
             ha="center", va="top", fontsize=7.5, fontweight="bold", color="#92400E")

    holes = [
        (12.0,  1.6, C_LINE,   "12,0\n(r=32)",  "center", 12.0),
        (48.0,  1.6, C_LINE,   "48,0\n(r=68)",  "center", 12.0),
        (197.0, 1.6, C_ACCENT, "197,0 (M3)",    "right",  11.5),
        (205.0, 4.0, C_WARN,   "205,0 (Ø8,0)\nEixo R=225!", "center", -14.5),
        (213.0, 1.6, C_ACCENT, "213,0 (M3)",    "left",   11.5),
    ]
    for hx, rad_h, col, lbl, ha_lbl, y_txt in holes:
        ax1.add_patch(patches.Circle((hx, 0.0), rad_h, facecolor=C_BG, edgecolor=col, lw=1.6, zorder=6))
        va_txt = "bottom" if y_txt > 0 else "top"
        ax1.plot([hx, hx], [rad_h if y_txt > 0 else -rad_h, y_txt * 0.84], color=col, lw=0.8, ls=":")
        dx_lbl = -1.5 if ha_lbl == "right" else (1.5 if ha_lbl == "left" else 0.0)
        ax1.text(hx + dx_lbl, y_txt, lbl, ha=ha_lbl, va=va_txt, fontsize=7.5, fontweight="bold", color=col)

    ax1.add_patch(patches.Circle((205.0, 0.0), 14.0, fill=False, edgecolor=C_ACCENT, ls="--", lw=1.2, zorder=5))

    draw_dim_h(ax1, 0.0, 225.0, 7.5, 26.0, "Comprimento Total do Braço = 225,0 mm (4 unidades idênticas)", text_dy=1.2)
    draw_dim_h(ax1, 12.0, 48.0, -7.5, -25.0, "Δ = 36,0 mm (Sanduíche)", color=C_LINE, text_dy=1.2)
    draw_dim_h(ax1, 197.0, 213.0, 7.5, 19.5, "16,0 mm* (ou 19,0 mm: 195,5 / 214,5)", color=C_ACCENT, fontsize=7.5, text_dy=0.9)
    draw_dim_v(ax1, 225.0, 238.0, -7.5, 7.5, "15,0 mm", text_dx=1.5)

    ax1.set_title("A. Vista Superior Cotada do Braço (Origem x = 0 mm na ponta interna → fica a r = 20 mm do centro do drone)",
                  fontsize=9.8, fontweight="bold", pad=6)
    ax1.set_ylabel("Largura [mm]")

    ax2.set_facecolor(C_BG)
    ax2.set_xlim(0, 220)
    ax2.set_ylim(0, 62)
    ax2.set_aspect("equal")
    ax2.axis("off")

    for x0, title_box, is_safe, screw_desc, pen_txt in [
        (8,   "✔ MONTAGEM SEGURA (Penetração 2,2 a 3,2 mm)", True,
              "Madeira 15mm: M3×18 + 1 arruela (2,2mm)\nAlumínio 12,7mm: M3×16 + 1 arruela (2,5mm)", "Folga segura\naté o cobre!"),
        (114, "✖ PERIGO DE CURTO MORTAL (Parafuso Longo > 4,0 mm!)", False,
              "M3×20 sem arruelas extras atravessa a base\ne rasga o verniz das bobinas do estator!", "CURTO NAS\nBOBINAS!"),
    ]:
        col_b = C_OK if is_safe else C_WARN
        bg_b  = "#F0FDF4" if is_safe else "#FEF2F2"
        ax2.add_patch(FancyBboxPatch((x0, 2), 98, 56, boxstyle="round,pad=1.2",
                                     facecolor=bg_b, edgecolor=col_b, lw=1.5))
        ax2.text(x0 + 49, 54.5, title_box, ha="center", fontsize=8.3, fontweight="bold", color=col_b)

        ax2.add_patch(patches.Rectangle((x0 + 8, 14), 52, 15, facecolor=C_WOOD, edgecolor=C_WOOD_EDG, lw=1.3))
        ax2.text(x0 + 13, 21.5, "Braço\n15 mm", va="center", fontsize=7.2, fontweight="bold", color=C_WOOD_EDG)
        ax2.add_patch(patches.Rectangle((x0 + 16, 29), 36, 3.5, facecolor="#94A3B8", edgecolor=C_LINE, lw=1.2))
        for wx in [x0 + 20, x0 + 42]:
            ax2.add_patch(FancyBboxPatch((wx, 34.2), 10, 10.5, boxstyle="round,pad=0.4",
                                         facecolor="#D97706", edgecolor="#78350F", lw=1.2))
        ax2.text(x0 + 34, 48.0, "Bobinas de Cobre A2212", ha="center", fontsize=7.3, fontweight="bold", color="#78350F")

        sx = x0 + 25
        ax2.add_patch(patches.Rectangle((sx - 4.0, 12.6), 8.0, 1.4, facecolor="#64748B", edgecolor=C_LINE, lw=0.8))
        ax2.add_patch(patches.Rectangle((sx - 3.2, 9.8), 6.4, 2.8, facecolor="#334155", edgecolor=C_LINE, lw=0.9))
        top_y = 31.8 if is_safe else 36.8
        ax2.add_patch(patches.Rectangle((sx - 1.6, 12.6), 3.2, top_y - 12.6,
                                        facecolor="#475569" if is_safe else C_WARN, edgecolor=C_LINE, lw=1.0, zorder=8))

        ax2.annotate(pen_txt, xy=(sx + 1.8, top_y), xytext=(x0 + 63, 36),
                     fontsize=7.6, fontweight="bold", color=col_b,
                     arrowprops=dict(arrowstyle="-|>", color=col_b, lw=1.4))
        ax2.text(x0 + 49, 5.5, screw_desc, ha="center", va="center", fontsize=7.3, color=C_LINE)

    fig.tight_layout(rect=[0.01, 0.01, 0.99, 0.90])
    out = os.path.join(OUT_DIR, "cad_03_gabarito_braco_225mm_e_motor.png")
    fig.savefig(out)
    plt.close(fig)
    return out


# =============================================================================
# PRANCHA 04: Layout CAD da Placa 70x90 mm + Esquemático Vetorial Real!
# =============================================================================
def generate_cad_04_placa_fc_70x90():
    fig, (ax, ax_sch) = plt.subplots(1, 2, figsize=(15.4, 8.1), dpi=180,
                                     gridspec_kw={"width_ratios": [1.05, 1.15]})
    add_title_banner(
        fig, "CAD-04",
        "PLACA CONTROLADORA 70,0 × 90,0 mm (FR4) — LAYOUT FÍSICO & ESQUEMÁTICO ELÉTRICO VETORIAL",
        "Substitui diagrama textual da Etapa 3.1: Posicionamento em malha 2,54 mm + esquemático completo (+5V_SYS, VBAT 39k/10k e Buffer 5V)"
    )

    setup_cad_ax(ax, (-48, 48), (-56, 56), grid_step=10.0)

    ax.add_patch(FancyBboxPatch((-35.0, -45.0), 70.0, 90.0, boxstyle="round,pad=0.6",
                                facecolor="#ECFDF5", edgecolor=C_PCB, linewidth=2.2, zorder=2))

    gx = np.arange(-12, 13) * 2.54
    gy = np.arange(-16, 17) * 2.54
    XX, YY = np.meshgrid(gx, gy)
    ax.scatter(XX.ravel(), YY.ravel(), s=1.8, color="#A7F3D0", zorder=3)
    draw_centerlines(ax, 0, 0, 42, 50, zorder=3)

    for sx in [-1, 1]:
        for sy in [-1, 1]:
            dx, dy = sx * 22.0, sy * 41.0
            ax.add_patch(patches.Circle((dx, dy), 3.5, facecolor=C_OK_BG, edgecolor=C_OK, lw=1.2, zorder=5))
            ax.add_patch(patches.Circle((dx, dy), 1.6, facecolor=C_BG, edgecolor=C_LINE, lw=1.4, zorder=6))

    # IMU ICM-42688-P em (0, 0)
    ax.add_patch(FancyBboxPatch((-7.5, -6.5), 15.0, 13.0, boxstyle="round,pad=0.3",
                                facecolor="#0284C7", edgecolor=C_LINE, lw=1.5, zorder=7))
    ax.text(0, 0.5, "ICM-42688-P\nSPI (0,0)", ha="center", va="center",
            fontsize=6.7, fontweight="bold", color="#FFFFFF", zorder=8)
    ax.annotate("", xy=(0, 11.5), xytext=(0, 6.5), arrowprops=dict(arrowstyle="-|>", color=C_WARN, lw=1.5), zorder=9)
    ax.text(0, 12.3, "+X", ha="center", fontsize=7.0, fontweight="bold", color=C_WARN)
    ax.annotate("", xy=(11.8, 0), xytext=(7.5, 0), arrowprops=dict(arrowstyle="-|>", color=C_ACCENT, lw=1.5), zorder=9)
    ax.text(12.2, 1.8, "+Y", ha="center", fontsize=6.8, fontweight="bold", color=C_ACCENT)

    # ESP32-S3-DevKitC-1 à esquerda (subida p/ y = -30 .. +25 para livrar 100% o damper (-22, -41)!)
    ax.add_patch(FancyBboxPatch((-33.2, -30.0), 22.0, 55.0, boxstyle="round,pad=0.4",
                                facecolor=C_ESP, edgecolor=C_LINE, lw=1.5, zorder=6))
    for hx in [-31.7, -12.7]:
        ax.add_patch(patches.Rectangle((hx - 0.9, -27.5), 1.8, 49.5, facecolor="#475569", edgecolor="#94A3B8", lw=0.7, zorder=7))
    ax.text(-22.2, 4.0, "ESP32-S3\nDevKitC-1\nN16R8\n(2×22p)", ha="center", va="center",
            fontsize=7.2, fontweight="bold", color="#FFFFFF", zorder=8)
    ax.add_patch(patches.Circle((-22.2, 19.5), 2.4, facecolor="#FACC15", edgecolor="#854D0E", lw=1.0, zorder=8))
    ax.text(-22.2, 15.0, "IPEX RF", ha="center", fontsize=6.2, color="#FDE047", fontweight="bold", zorder=8)
    ax.add_patch(patches.Rectangle((-26.5, -20.0), 3.0, 3.0, facecolor="#22C55E", edgecolor="#FFFFFF", lw=0.8, zorder=8))
    ax.text(-21.5, -18.5, "RGB", va="center", fontsize=6.0, color="#86EFAC", fontweight="bold", zorder=8)
    ax.add_patch(patches.Rectangle((-30.8, -32.2), 7.0, 3.8, facecolor="#CBD5E1", edgecolor=C_LINE, lw=1.0, zorder=8))
    ax.add_patch(patches.Rectangle((-20.6, -32.2), 7.0, 3.8, facecolor="#CBD5E1", edgecolor=C_LINE, lw=1.0, zorder=8))
    ax.text(-27.3, -34.8, "USB", ha="center", fontsize=6.2, fontweight="bold", color=C_ACCENT)
    ax.text(-16.2, -34.8, "UART", ha="center", fontsize=6.2, color=C_SUB)

    # Header 6p GPS Fase 2 (Frente-Esquerda: x = -17..+1, y = +29..+33.5)
    ax.add_patch(patches.Rectangle((-18.0, 29.0), 17.5, 4.4, facecolor="#334155", edgecolor=C_LINE, lw=1.1, zorder=6))
    ax.text(-9.25, 35.0, "Header 6p GPS (Fase 2)", ha="center", fontsize=6.6, fontweight="bold", color=C_SUB)

    # BMP280 c/ espuma (Frente-Direita)
    ax.add_patch(FancyBboxPatch((7.0, 25.0), 16.0, 10.5, boxstyle="round,pad=0.3",
                                facecolor="#7E22CE", edgecolor=C_LINE, lw=1.2, zorder=6))
    ax.add_patch(patches.Rectangle((9.0, 26.5), 12.0, 7.5, facecolor="#1E293B", alpha=0.55,
                                   edgecolor="#E9D5FF", ls="--", lw=1.0, zorder=7))
    ax.text(15.0, 30.2, "BMP280\n+Espuma", ha="center", va="center", fontsize=6.3, fontweight="bold", color="#FFFFFF", zorder=8)

    # DIP-14 SN74HCT125N + 100nF
    ax.add_patch(patches.Rectangle((13.5, -13.0), 9.5, 20.0, facecolor="#1E293B", edgecolor=C_LINE, lw=1.3, zorder=6))
    ax.add_patch(patches.Wedge((18.25, 7.0), 2.0, 180, 360, facecolor="#ECFDF5", edgecolor=C_LINE, lw=0.9, zorder=7))
    ax.text(18.25, -3.0, "74HCT\n125N\nDIP-14", ha="center", va="center", fontsize=6.2, fontweight="bold", color="#FDE047", zorder=8)
    ax.add_patch(patches.Rectangle((13.5, 8.8), 5.2, 2.4, facecolor="#F59E0B", edgecolor=C_LINE, lw=0.8, zorder=7))
    ax.text(11.8, 10.0, "100nF", ha="right", va="center", fontsize=6.0, color="#B45309", fontweight="bold")

    for ey, elbl in [(18.5, "M1 FR"), (12.0, "M4 FL"), (-18.0, "M2 RR"), (-24.5, "M3 RL")]:
        ax.add_patch(patches.Rectangle((26.5, ey - 2.1), 6.2, 4.2, facecolor="#0F172A", edgecolor="#94A3B8", lw=0.9, zorder=7))
        ax.text(25.3, ey, elbl, ha="right", va="center", fontsize=6.3, fontweight="bold", color=C_LINE)

    # KY-012 + 39k/10k + PDB Header
    ax.add_patch(patches.Circle((11.5, -24.5), 5.3, facecolor="#0F172A", edgecolor=C_LINE, lw=1.2, zorder=6))
    ax.add_patch(patches.Circle((11.5, -24.5), 1.5, facecolor="#E2E8F0", zorder=7))
    ax.text(11.5, -31.8, "KY-012", ha="center", fontsize=6.3, fontweight="bold", color=C_LINE)

    ax.add_patch(patches.Rectangle((-2.0, -39.5), 13.5, 4.5, facecolor="#DBEAFE", edgecolor=C_ACCENT, lw=1.0, zorder=6))
    ax.text(4.75, -37.25, "39k / 10k", ha="center", va="center", fontsize=6.2, fontweight="bold", color=C_ACCENT, zorder=7)

    ax.add_patch(patches.Rectangle((15.0, -35.0), 17.5, 5.2, facecolor="#991B1B", edgecolor=C_LINE, lw=1.1, zorder=6))
    ax.text(23.75, -32.4, "PDB 5V/VBAT", ha="center", va="center", fontsize=6.0, fontweight="bold", color="#FFFFFF", zorder=7)

    draw_dim_h(ax, -35.0, 35.0, 45.0, 51.5, "70,0 mm (26 ilhas)", text_dy=0.9)
    draw_dim_v(ax, 35.0, 42.5, -45.0, 45.0, "90,0 mm (33 ilhas)", text_dx=1.4)
    ax.annotate("FRENTE ↑", xy=(0, 45), xytext=(0, 39.2), ha="center", fontsize=7.8, fontweight="bold", color=C_WARN,
                arrowprops=dict(arrowstyle="-|>", color=C_WARN, lw=1.6))

    ax.set_title("A. Layout Físico Vista de Topo (Escala Real)", fontsize=10.0, fontweight="bold", pad=6)
    ax.set_xlabel("X [mm]")
    ax.set_ylabel("Y [mm]")

    # --- Painel Direito: Esquemático Elétrico Vetorial Real (Sem Arte ASCII!) ---
    ax_sch.set_facecolor(C_BG)
    ax_sch.set_xlim(0, 175)
    ax_sch.set_ylim(0, 145)
    ax_sch.set_aspect("equal")
    ax_sch.axis("off")

    # 1. Quadro Superior: Alimentação +5V_SYS & Divisor VBAT 39k/10k (y = 92..142)
    ax_sch.add_patch(FancyBboxPatch((3, 92), 169, 49, boxstyle="round,pad=1.2",
                                    facecolor="#F8FAFC", edgecolor="#94A3B8", lw=1.2))
    ax_sch.text(8, 137.5, "B1. ALIMENTAÇÃO PROTEGIDA +5V_SYS & SENSOR VBAT 3S (39 kΩ / 10 kΩ)",
                fontsize=8.3, fontweight="bold", color=C_ACCENT)

    # Ramo +5V BEC -> 1N5819 -> +5V_SYS
    ax_sch.text(8, 127, "BEC 5,0V\n(Mini PDB)", va="center", fontsize=7.2, fontweight="bold", color=C_WARN)
    draw_schottky_h(ax_sch, 30, 62, 127, "1N5819")
    ax_sch.plot([62, 165], [127, 127], color="#DC2626", lw=1.8)
    ax_sch.text(114, 129.2, "Barramento +5V_SYS (~4,75V) → ESP32 5V(IN), Pino 14 (74HCT) e KY-012",
                ha="center", fontsize=6.9, fontweight="bold", color="#991B1B")
    draw_cap_v(ax_sch, 156, 127, 117, "100 nF\n(Pino 14)", color=C_LINE)
    draw_gnd(ax_sch, 156, 117)

    # Ramo VBAT 3S -> 39k -> nó GPIO1 -> 10k || 100nF -> GND
    ax_sch.text(8, 108, "BAT+ 3S\n(12,6V máx)", va="center", fontsize=7.2, fontweight="bold", color="#B45309")
    ax_sch.plot([30, 42], [108, 108], color="#D97706", lw=1.6)
    draw_resistor_h(ax_sch, 51, 108, 18, 4.5, "39 kΩ 1%", color=C_LINE)
    ax_sch.plot([60, 116], [108, 108], color=C_ACCENT, lw=1.7)
    ax_sch.scatter([74, 96], [108, 108], s=18, color=C_ACCENT, zorder=8)
    ax_sch.annotate("GPIO1 (ADC1_CH0)\n2,57V @ 12,6V (< 2,90V!)", xy=(116, 108), xytext=(120, 108),
                    va="center", fontsize=7.2, fontweight="bold", color=C_OK)

    ax_sch.plot([74, 74], [108, 104], color=C_LINE, lw=1.4)
    draw_resistor_v(ax_sch, 74, 99.5, 4.2, 9.0, "10 kΩ 1%")
    draw_gnd(ax_sch, 74, 95)

    draw_cap_v(ax_sch, 96, 108, 95, "100 nF")
    draw_gnd(ax_sch, 96, 95)

    # 2. Quadro Médio: Buffer Level-Shifter 5V SN74HCT125N (y = 43..88)
    ax_sch.add_patch(FancyBboxPatch((3, 43), 169, 45, boxstyle="round,pad=1.2",
                                    facecolor="#F0FDF4", edgecolor=C_OK, lw=1.2))
    ax_sch.text(8, 84, "B2. BUFFER LEVEL-SHIFTER 3,3V → 5,0V (SN74HCT125N DIP-14 — 4 CANAIS 400 Hz)",
                fontsize=8.2, fontweight="bold", color=C_OK)

    # Bloco central do CI SN74HCT125N
    ax_sch.add_patch(patches.Rectangle((64, 48), 44, 32, facecolor=C_ESP, edgecolor=C_LINE, lw=1.5, zorder=5))
    ax_sch.text(86, 64, "SN74HCT125N\nVCC(14)=+5V_SYS\nGND(7) & OE#(1,4,10,13)=GND",
                ha="center", va="center", fontsize=6.5, fontweight="bold", color="#FDE047", zorder=7)

    esc_map = [
        (76, "GPIO4", "2 (1A)", "3 (1Y)",  "ESC 1 — M1 FR (CCW)"),
        (68, "GPIO5", "5 (2A)", "6 (2Y)",  "ESC 2 — M2 RR (CW)"),
        (60, "GPIO6", "9 (3A)", "8 (3Y)",  "ESC 3 — M3 RL (CCW)"),
        (52, "GPIO7", "12(4A)", "11(4Y)",  "ESC 4 — M4 FL (CW)"),
    ]
    for ry, gpio_s, pin_in, pin_out, esc_s in esc_map:
        ax_sch.text(8, ry, gpio_s, va="center", fontsize=7.2, fontweight="bold", color=C_ACCENT)
        ax_sch.plot([23, 64], [ry, ry], color=C_ACCENT, lw=1.4)
        ax_sch.text(62.5, ry + 1.0, pin_in, ha="right", fontsize=6.1, color=C_SUB, fontweight="bold")
        ax_sch.plot([108, 126], [ry, ry], color=C_OK, lw=1.6)
        ax_sch.text(109.5, ry + 1.0, pin_out, ha="left", fontsize=6.1, color=C_SUB, fontweight="bold")
        ax_sch.text(128, ry, esc_s, va="center", fontsize=7.0, fontweight="bold", color=C_LINE)

    ax_sch.text(42, 45.5, "*Pull-down 10 kΩ → GND em cada entrada + Fio vermelho (+5V) dos 4 ESCs isolado!",
                ha="left", fontsize=6.8, fontweight="bold", color=C_WARN)

    # 3. Quadro Inferior: SPI 10 MHz (IMU) & I2C 400 kHz (BMP280) (y = 3..39)
    ax_sch.add_patch(FancyBboxPatch((3, 3), 169, 36, boxstyle="round,pad=1.2",
                                    facecolor="#FAF5FF", edgecolor="#7E22CE", lw=1.2))
    ax_sch.text(8, 35, "B3. SENSORES EM 3,3V LIMPO (+10 µF || 100 nF) — SPI 10 MHz & I2C 400 kHz",
                fontsize=8.2, fontweight="bold", color="#7E22CE")

    ax_sch.add_patch(patches.Rectangle((10, 8), 36, 23, facecolor=C_ESP, edgecolor=C_LINE, lw=1.2))
    ax_sch.text(28, 19.5, "ESP32-S3\n3V3 / FSPI\nI2C0 / GPIO", ha="center", va="center",
                fontsize=6.8, fontweight="bold", color="#FFFFFF")

    ax_sch.add_patch(patches.Rectangle((76, 18), 42, 14, facecolor="#0284C7", edgecolor=C_LINE, lw=1.2))
    ax_sch.text(97, 25, "ICM-42688-P\n(FSYNC → GND)", ha="center", va="center",
                fontsize=6.7, fontweight="bold", color="#FFFFFF")
    ax_sch.plot([46, 76], [25, 25], color="#0284C7", lw=1.8)
    ax_sch.text(61, 27.2, "CS10/MOSI11/SCK12\nMISO13 / INT1=14", ha="center", fontsize=6.1, fontweight="bold", color="#0369A1")

    ax_sch.add_patch(patches.Rectangle((126, 7), 40, 14, facecolor="#7E22CE", edgecolor=C_LINE, lw=1.2))
    ax_sch.text(146, 14, "BMP280 (0x76)\nSDO → GND", ha="center", va="center",
                fontsize=6.7, fontweight="bold", color="#FFFFFF")
    ax_sch.plot([46, 126], [12, 12], color="#7E22CE", lw=1.6)
    ax_sch.text(86, 13.8, "I2C: SDA = GPIO8  |  SCL = GPIO9  (Core 0)", ha="center", fontsize=6.4, fontweight="bold", color="#6B21A8")

    fig.tight_layout(rect=[0.01, 0.01, 0.99, 0.90])
    out = os.path.join(OUT_DIR, "cad_04_layout_placa_fc_70x90.png")
    fig.savefig(out)
    plt.close(fig)
    return out


# =============================================================================
# PRANCHA 05: Painel Frontal do Controle Caseiro + Esquemático Vetorial +V_POTS
# =============================================================================
def generate_cad_05_controle_ground_tx():
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15.2, 7.3), dpi=180,
                                   gridspec_kw={"width_ratios": [1.12, 1.08]})
    add_title_banner(
        fig, "CAD-05",
        "CONTROLE REMOTO LONG-RANGE (ground_tx) — PAINEL FRONTAL (150 × 100 mm) & ESQUEMÁTICO ADC1",
        "Substitui diagrama textual da Etapa 4: Ergonomia Mode 2 + OLED 0,96\" + Circuito vetorial do barramento +V_POTS (560 Ω)"
    )

    setup_cad_ax(ax1, (-90, 90), (-68, 72), grid_step=15.0)
    draw_centerlines(ax1, 0, 0, 82, 58)

    ax1.add_patch(FancyBboxPatch((-75.0, -50.0), 150.0, 100.0, boxstyle="round,pad=3.0",
                                 facecolor="#F1F5F9", edgecolor=C_LINE, linewidth=2.2, zorder=2))

    ax1.add_patch(patches.Rectangle((-5.0, 50.0), 10.0, 6.0, facecolor="#FACC15", edgecolor="#854D0E", lw=1.2, zorder=5))
    ax1.add_patch(patches.Rectangle((-3.0, 56.0), 6.0, 11.0, facecolor=C_ESP, edgecolor=C_LINE, lw=1.2, zorder=5))
    ax1.text(10, 61, "Antena SMA 2.4 GHz (3–5 dBi)", va="center", fontsize=7.6, fontweight="bold", color=C_LINE)

    for sx, slbl, ssub, scol in [
        (-52.0, "SW1: ARM / KILL", "Cima = SAFE (Aberto)\nBaixo = ARM (GPIO10→GND)", C_WARN),
        ( 52.0, "SW2: MODO 3-POS", "STAB / ALTH / CINE\n(GPIO11 / Aberto / GPIO12)", C_ACCENT),
    ]:
        ax1.add_patch(patches.Circle((sx, 37.0), 6.0, facecolor="#CBD5E1", edgecolor=C_LINE, lw=1.4, zorder=5))
        ax1.add_patch(patches.Circle((sx, 37.0), 2.8, facecolor="#334155", zorder=6))
        ax1.text(sx, 26.5, slbl, ha="center", fontsize=7.5, fontweight="bold", color=scol)
        ax1.text(sx, 18.0, ssub, ha="center", fontsize=6.3, color=C_SUB)

    ax1.add_patch(FancyBboxPatch((-15.5, 13.0), 31.0, 23.0, boxstyle="round,pad=0.5",
                                 facecolor="#0F172A", edgecolor="#334155", lw=1.5, zorder=5))
    ax1.text(0, 31.2, "DISARMED STAB -58dB", ha="center", fontsize=5.2, family="monospace", color="#38BDF8", fontweight="bold", zorder=7)
    ax1.text(0, 25.5, "3S:12.4V   TX:4.1V", ha="center", fontsize=5.2, family="monospace", color="#FFFFFF", zorder=7)
    ax1.text(0, 19.8, "OK P/ ARMAR", ha="center", fontsize=5.4, family="monospace", color="#4ADE80", fontweight="bold", zorder=7)

    for gx, gtitle, gv, gh in [
        (-44.0, "Gimbal Esq. (Mode 2)", "Throttle ↑↓ (SEM mola, GPIO1)", "Yaw ←→ (GPIO2)"),
        ( 44.0, "Gimbal Dir. (Mode 2)", "Pitch ↑↓ (COM mola, GPIO4)",    "Roll ←→ (GPIO5)"),
    ]:
        ax1.add_patch(FancyBboxPatch((gx - 22.0, -30.0), 44.0, 44.0, boxstyle="round,pad=1.0",
                                     facecolor="#E2E8F0", edgecolor=C_SUB, ls="--", lw=1.0, zorder=3))
        ax1.add_patch(patches.Circle((gx, -8.0), 19.0, facecolor="#1E293B", edgecolor=C_LINE, lw=1.6, zorder=4))
        ax1.plot([gx - 14, gx + 14], [-8, -8], color="#64748B", lw=1.0, zorder=5)
        ax1.plot([gx, gx], [-22, 6], color="#64748B", lw=1.0, zorder=5)
        stick_y = -18.0 if gx < 0 else -8.0
        ax1.add_patch(patches.Circle((gx, stick_y), 4.8, facecolor="#EF4444" if gx < 0 else "#3B82F6",
                                     edgecolor="#FFFFFF", lw=1.2, zorder=6))
        ax1.text(gx, -36.5, gtitle, ha="center", fontsize=7.4, fontweight="bold", color=C_LINE)
        ax1.text(gx, -43.0, f"{gv}\n{gh}", ha="center", fontsize=6.1, color=C_SUB)

    ax1.add_patch(FancyBboxPatch((-6.0, -26.0), 12.0, 12.0, boxstyle="round,pad=0.5",
                                 facecolor="#CBD5E1", edgecolor=C_LINE, lw=1.2, zorder=5))
    ax1.add_patch(patches.Circle((0, -20.0), 4.0, facecolor="#F59E0B", edgecolor=C_LINE, lw=1.0, zorder=6))
    ax1.text(0, -33.5, "Botão [AUX] GPIO13\n(Boot=Cal | 1,5s=NVS)", ha="center", fontsize=6.4, fontweight="bold", color="#B45309")

    ax1.add_patch(patches.Rectangle((-7.0, -54.0), 14.0, 4.0, facecolor="#334155", edgecolor=C_LINE, lw=1.2, zorder=5))
    ax1.text(0, -61.5, "Porta USB-C Nativa (OTG MAVLink v2 → QGroundControl)",
             ha="center", fontsize=7.4, fontweight="bold", color=C_ACCENT)

    draw_dim_h(ax1, -75.0, 75.0, 50.0, 67.5, "150,0 mm (Caixa Patola PB-114)", text_dy=1.0)
    draw_dim_h(ax1, -44.0, 44.0, -8.0, 4.5, "88,0 mm (Entre Centros)", color=C_DIM, fontsize=7.5, text_dy=1.0)
    ax1.set_title("A. Gabarito Frontal Mecânico (Mode 2)", fontsize=10.0, fontweight="bold", pad=6)
    ax1.set_xlabel("X [mm]")
    ax1.set_ylabel("Y [mm]")

    # --- Painel Direito: Esquemático Elétrico Vetorial Real do Controle ---
    ax2.set_facecolor(C_BG)
    ax2.set_xlim(0, 175)
    ax2.set_ylim(0, 130)
    ax2.set_aspect("equal")
    ax2.axis("off")

    # Bloco Superior: Barramento +V_POTS (560R + 10uF) e 4 Potenciômetros (y = 54..127)
    ax2.add_patch(FancyBboxPatch((3, 54), 169, 72, boxstyle="round,pad=1.2",
                                 facecolor="#F0FDF4", edgecolor=C_OK, lw=1.3))
    ax2.text(8, 121.5, "B1. CIRCUITO ANTI-SATURAÇÃO DOS 4 POTENCIÔMETROS (V_max <= 2,68 V)",
             fontsize=8.2, fontweight="bold", color=C_OK)

    ax2.text(8, 110, "ESP32-S3\nPino 3V3", va="center", fontsize=7.3, fontweight="bold", color=C_WARN)
    ax2.plot([28, 40], [110, 110], color=C_WARN, lw=1.7)
    draw_resistor_h(ax2, 50, 110, 20, 4.8, "560 Ω 1%", color=C_LINE)
    ax2.plot([60, 156], [110, 110], color=C_OK, lw=2.0)
    ax2.text(112, 112.5, "Barramento +V_POTS (2,28..2,68 V < 2,90 V!)", ha="center",
             fontsize=7.2, fontweight="bold", color=C_OK)

    draw_cap_v(ax2, 68, 110, 95, "10 µF", color=C_LINE)
    draw_gnd(ax2, 68, 95)

    pots = [
        ( 24, "THR",   "GPIO1"),
        ( 64, "YAW",   "GPIO2"),
        (104, "PITCH", "GPIO4"),
        (144, "ROLL",  "GPIO5"),
    ]
    ax2.plot([24, 60], [110, 110], color=C_OK, ls=":", lw=0.0) # anchor
    # Conecta linha +V_POTS de x=24 a x=144 em y=90? Melhor: puxa de y=110 em x=88..156 ou faz barramento y=92!
    ax2.plot([90, 90], [110, 92], color=C_OK, lw=1.8)
    ax2.plot([24, 144], [92, 92], color=C_OK, lw=1.8)
    for px, pname, pgpio in pots:
        ax2.scatter([px], [92], s=16, color=C_OK, zorder=7)
        ax2.plot([px, px], [92, 86], color=C_OK, lw=1.3)
        # Corpo do potenciômetro
        ax2.add_patch(patches.Rectangle((px - 3.2, 73), 6.4, 13, facecolor="#FEF9C3", edgecolor=C_LINE, lw=1.3, zorder=6))
        ax2.text(px - 4.5, 79.5, pname, ha="right", va="center", fontsize=6.6, fontweight="bold", color=C_LINE)
        ax2.plot([px, px], [73, 66], color=C_LINE, lw=1.2)
        draw_gnd(ax2, px, 66)
        # Cursor lateral (wiper) + filtro 100nF + saída GPIO
        ax2.annotate("", xy=(px + 3.2, 79.5), xytext=(px + 13, 79.5),
                     arrowprops=dict(arrowstyle="-|>", color=C_ACCENT, lw=1.3))
        ax2.text(px + 13.5, 81.8, pgpio, ha="left", fontsize=6.8, fontweight="bold", color=C_ACCENT)
        draw_cap_v(ax2, px + 11, 79.5, 66, "100nF")
        draw_gnd(ax2, px + 11, 66)

    # Bloco Inferior: Alimentação 18650 + Boost 5V + 1N5819 + Divisor 100k/100k (y = 4..49)
    ax2.add_patch(FancyBboxPatch((3, 4), 169, 45, boxstyle="round,pad=1.2",
                                 facecolor="#EFF6FF", edgecolor=C_ACCENT, lw=1.3))
    ax2.text(8, 44.5, "B2. BATERIA 18650 INTERNA + PROTEÇÃO USB-OTG (1N5819) + TELEMETRIA TX",
             fontsize=8.1, fontweight="bold", color=C_ACCENT)

    ax2.add_patch(patches.Rectangle((8, 25), 24, 12, facecolor="#DBEAFE", edgecolor=C_ACCENT, lw=1.3))
    ax2.text(20, 31, "18650\n3,5..4,2V", ha="center", va="center", fontsize=6.6, fontweight="bold", color="#1E3A8A")
    ax2.plot([32, 44], [33, 33], color=C_LINE, lw=1.5)

    ax2.add_patch(patches.Rectangle((44, 26), 44, 13, facecolor=C_ESP, edgecolor=C_LINE, lw=1.2))
    ax2.text(66, 32.5, "TP4056 + MT3608\n(Ajustar 5,00V!)", ha="center", va="center", fontsize=6.5, fontweight="bold", color="#FDE047")

    draw_schottky_h(ax2, 88, 120, 33, "1N5819")
    ax2.annotate("", xy=(140, 33), xytext=(120, 33),
                 arrowprops=dict(arrowstyle="-|>", color=C_WARN, lw=1.6))
    ax2.text(142, 33, "ESP32-S3\n5V (IN)", va="center", fontsize=7.3, fontweight="bold", color=C_WARN)

    # Divisor 100k + 100k no GPIO6
    ax2.plot([37, 37], [33, 17], color=C_ACCENT, lw=1.4)
    ax2.plot([37, 50], [17, 17], color=C_ACCENT, lw=1.4)
    draw_resistor_h(ax2, 59, 17, 18, 4.2, "100 kΩ")
    ax2.plot([68, 118], [17, 17], color=C_ACCENT, lw=1.5)
    ax2.text(120, 17, "→ GPIO6 (2,10V máx)", va="center", fontsize=7.1, fontweight="bold", color=C_ACCENT)
    ax2.plot([82, 82], [17, 14.5], color=C_LINE, lw=1.3)
    draw_resistor_v(ax2, 82, 11.2, 3.8, 6.5, "100k||100nF")
    draw_gnd(ax2, 82, 7.8)

    fig.tight_layout(rect=[0.01, 0.01, 0.99, 0.90])
    out = os.path.join(OUT_DIR, "cad_05_painel_controle_ground_tx.png")
    fig.savefig(out)
    plt.close(fig)
    return out


# =============================================================================
# PRANCHA 06: Bancada "Gangorra 1-Eixo" para Tuning Seguro de PID (Portão 8)
# =============================================================================
def generate_cad_06_gangorra_1eixo():
    fig, ax = plt.subplots(1, 1, figsize=(13.8, 6.7), dpi=180)
    add_title_banner(
        fig, "CAD-06",
        "BANCADA MECÂNICA 'GANGORRA 1-EIXO' — SINTONIA SEGURA DE PID COM HÉLICES (PORTÃO 8)",
        "Substitui diagrama textual da Etapa 5: Isola 1 grau de liberdade por vez (Roll, depois Pitch) com limitador físico de ±35°"
    )

    setup_cad_ax(ax, (-260, 260), (-25, 315), grid_step=25.0, equal=True)

    ax.add_patch(patches.Rectangle((-210, 0), 420, 18, facecolor=C_WOOD, edgecolor=C_WOOD_EDG, lw=2.0, zorder=4))
    ax.fill_between([-260, 260], -25, 0, color="#F1F5F9", hatch="///", edgecolor="#CBD5E1", zorder=1)
    ax.text(0, 9, "Base Pesada de Madeira 400×400×18 mm (Fixada na Mesa Externa com Sargento / Peso)",
            ha="center", va="center", fontsize=8.4, fontweight="bold", color=C_WOOD_EDG, zorder=6)

    pillar_poly = patches.Polygon([[-62, 18], [-16, 232], [16, 232], [62, 18]], closed=True,
                                  facecolor="#FDE68A", edgecolor=C_WOOD_EDG, lw=1.8, zorder=3)
    ax.add_patch(pillar_poly)
    ax.text(0, 78, "2× Pilares A-Frame\nAltura = 230 mm\n(Afastados 360 mm\nna profundidade)",
            ha="center", va="center", fontsize=7.9, fontweight="bold", color=C_WOOD_EDG,
            bbox=dict(boxstyle="round,pad=0.25", facecolor="#FEF9C3", edgecolor=C_WOOD_EDG, lw=0.9), zorder=5)

    pivot = (0.0, 224.0)
    ax.add_patch(patches.Circle(pivot, 7.5, facecolor="#334155", edgecolor=C_LINE, lw=1.6, zorder=8))
    ax.add_patch(patches.Circle(pivot, 2.8, facecolor="#FACC15", edgecolor=C_LINE, lw=1.0, zorder=9))
    ax.annotate("Eixo Livre M5\ns/ Atrito no CG", xy=(6, 220), xytext=(45, 182),
                fontsize=7.8, fontweight="bold", color=C_LINE,
                arrowprops=dict(arrowstyle="-|>", color=C_LINE, lw=1.3), zorder=9)

    tilt_deg = -8.0
    tr = Affine2D().rotate_deg_around(pivot[0], pivot[1], tilt_deg) + ax.transData

    beam = patches.Rectangle((-170, 228), 340, 12, facecolor=C_WOOD, edgecolor=C_WOOD_EDG, lw=1.5, zorder=5)
    beam.set_transform(tr)
    ax.add_patch(beam)

    fc_r = patches.Rectangle((-35, 240), 70, 10, facecolor=C_PCB_FILL, edgecolor=C_PCB, lw=1.4, zorder=6)
    fc_r.set_transform(tr)
    ax.add_patch(fc_r)

    for mx in [-159.1, 159.1]:
        mot = patches.Rectangle((mx - 12, 240), 24, 18, facecolor="#334155", edgecolor=C_LINE, lw=1.2, zorder=6)
        prop_bar = patches.Rectangle((mx - 90, 259), 180, 3.5, facecolor="#0284C7", edgecolor=C_LINE, lw=1.0, zorder=7)
        mot.set_transform(tr)
        prop_bar.set_transform(tr)
        ax.add_patch(mot)
        ax.add_patch(prop_bar)

    arc = patches.Arc(pivot, 150, 150, angle=0, theta1=155, theta2=205, color=C_ACCENT, lw=1.6, ls="--", zorder=7)
    ax.add_patch(arc)
    ax.text(-88, 214, "Giro Livre\n±35° Máx", ha="right", va="center", fontsize=8.3, fontweight="bold", color=C_ACCENT)

    ax.plot([-150, -150], [18, 248], color=C_WARN, ls="--", lw=1.6, zorder=4)
    ax.plot([ 150,  150], [18, 204], color=C_WARN, ls="--", lw=1.6, zorder=4)
    ax.text(155, 115, "Cordão Limitador (±35°)\nImpede que a hélice\ntoque na bancada!",
            ha="left", va="center", fontsize=8.0, fontweight="bold", color=C_WARN,
            bbox=dict(boxstyle="round,pad=0.3", facecolor=C_WARN_BG, edgecolor=C_WARN, lw=1.0))

    draw_dim_v(ax, -62, -222, 18, 224, "Altura Livre do Eixo = 206–250 mm", text_dx=3.0)
    draw_dim_h(ax, -159.1, 159.1, 260, 294, "Projeção Lateral Entre Eixos = 318,2 mm", text_dy=1.8)

    ax.set_xlabel("Largura Lateral [mm]")
    ax.set_ylabel("Altura Acima da Bancada [mm]")

    fig.tight_layout(rect=[0.01, 0.01, 0.99, 0.90])
    out = os.path.join(OUT_DIR, "cad_06_bancada_gangorra_1eixo.png")
    fig.savefig(out)
    plt.close(fig)
    return out


# =============================================================================
# PRANCHA 07: Painel Quantitativo de Engenharia v2.1 (Bode, Modal, RF, ADC1)
# =============================================================================
def generate_cad_07_validacao_engenharia_v21():
    fig, axes = plt.subplots(2, 2, figsize=(14.8, 9.4), dpi=165)
    add_title_banner(
        fig, "CAD-07",
        "PAINEL ANALÍTICO v2.1 — ESTABILIDADE BODE, ESPECTRO MODAL, ORÇAMENTO RF & LINEARIDADE ADC1",
        "Curvas matemáticas exatas do projeto fechado v2.1 (Filtros 65/35 Hz, Madeira 15×15 mm / Alumínio 1/2\", Divisor 39k/10k e Topo 560 Ω)"
    )

    fs = 1000.0
    dt = 1.0 / fs
    freqs = np.geomspace(0.2, 450.0, 2000)
    w_rad = 2.0 * np.pi * freqs
    z = np.exp(1j * w_rad * dt)
    s = 1j * w_rad

    def biquad_h(fc_hz):
        w = math.tan(math.pi * fc_hz / fs)
        w2 = w * w
        k = 1.0 + math.sqrt(2.0) * w + w2
        b = np.array([w2 / k, 2.0 * w2 / k, w2 / k])
        a = np.array([1.0, 2.0 * (w2 - 1.0) / k, (1.0 - math.sqrt(2.0) * w + w2) / k])
        return np.polyval(b, z**-1) / np.polyval(a, z**-1)

    ax = axes[0, 0]
    H_gyro = biquad_h(65.0)
    H_dlpf = biquad_h(35.0)
    C_pid = 0.14 + 0.18 * dt / (1.0 - z**-1) + 0.0045 * ((1.0 - z**-1) / dt) * H_dlpf
    Gp = (4.277 / (0.0105 * s + 0.008)) * (1.0 / (1.0 + 0.035 * s)) * np.exp(-0.002 * s)
    L = Gp * H_gyro * C_pid
    mag_db = 20.0 * np.log10(np.abs(L))

    ax.semilogx(freqs, mag_db, color=C_ACCENT, lw=2.0, label="|L(jω)| Malha de Taxa v2.1 (dB)")
    ax.axhline(0.0, color=C_LINE, ls=":", lw=1.0)
    ax.axvline(4.18, color=C_OK, ls="--", lw=1.3, label="Cruzamento fc = 4,18 Hz (PM = 54,2° | GM = 14,3 dB)")
    ax.axvspan(125, 165, color=C_WARN, alpha=0.15, label="Vibração 2× Pás Hover (~142 Hz: -26,7 dB)")
    ax.set_xlim(0.3, 350)
    ax.set_ylim(-50, 42)
    ax.set_title("1. Malha Interna de Taxa (65 Hz / 35 Hz) — Margem de Fase e Ganho", fontsize=10, fontweight="bold")
    ax.set_xlabel("Frequência (Hz)")
    ax.set_ylabel("Magnitude Aberta (dB)")
    ax.grid(True, which="both", alpha=0.35)
    ax.legend(fontsize=7.8, loc="upper right")

    ax = axes[0, 1]
    f_lin = np.linspace(15.0, 230.0, 1000)
    for name, fn, Q, col, ls in [
        ("Madeira 13×13 mm REPROVADA (fn1 = 76,6 Hz)", 76.6, 5.0, C_WARN, "--"),
        ("Madeira 15×15 mm APROVADA (fn1 = 101,3 Hz)", 101.3, 5.0, "#D97706", "-"),
        ("Alumínio 1/2\"×1,0 mm PADRÃO OURO (fn1 = 137,2 Hz)", 137.2, 8.5, C_ACCENT, "-"),
    ]:
        r = f_lin / fn
        trans = 1.0 / np.sqrt((1.0 - r**2)**2 + (r / Q)**2)
        ax.plot(f_lin, 20.0 * np.log10(trans), color=col, ls=ls, lw=1.9, label=name)
    ax.axvspan(63.0, 80.0, color="#FDE047", alpha=0.35, label="Banda Fundamental 1× RPM Hover (63–80 Hz)")
    ax.set_title("2. Espectro Modal dos Braços vs. Rotação 1× de Hover", fontsize=10, fontweight="bold")
    ax.set_xlabel("Frequência (Hz)")
    ax.set_ylabel("Transmissibilidade Mecânica (dB)")
    ax.set_ylim(-25, 22)
    ax.grid(True, alpha=0.35)
    ax.legend(fontsize=7.7, loc="lower left")

    ax = axes[1, 0]
    d_m = np.linspace(25.0, 2000.0, 600)
    fspl = 40.09 + 20.0 * np.log10(d_m)
    pr_los = 19.5 + 4.4 - fspl - 8.0
    ax.plot(d_m, pr_los, color=C_ACCENT, lw=2.2, label="Potência Recebida Pr(d) c/ -8 dB perdas reais")
    ax.axhline(-105.0, color=C_OK, ls="-.", lw=1.6, label="Sensibilidade ESP-NOW LR 250 kbps (-105 dBm)")
    ax.axvline(500.0, color="#64748B", ls=":", lw=1.2)
    ax.axvline(1000.0, color="#64748B", ls=":", lw=1.2)
    ax.scatter([500.0, 1000.0], [-78.2, -84.2], color=C_OK, zorder=6)
    ax.text(515, -76.5, "500 m: -78,2 dBm\n(+26,8 dB margem!)", fontsize=7.8, fontweight="bold", color=C_OK)
    ax.text(1015, -83.0, "1000 m: -84,2 dBm\n(+20,8 dB margem!)", fontsize=7.8, fontweight="bold", color=C_OK)
    ax.set_title("3. Orçamento de Enlace Friis 2,4 GHz — WIFI_PHY_RATE_LORA_250K", fontsize=10, fontweight="bold")
    ax.set_xlabel("Distância Controle ↔ Drone (m)")
    ax.set_ylabel("Potência Recebida (dBm)")
    ax.set_ylim(-112, -52)
    ax.grid(True, alpha=0.35)
    ax.legend(fontsize=7.8, loc="upper right")

    ax = axes[1, 1]
    v_pin = np.linspace(0.0, 3.35, 500)
    adc_raw = np.minimum(4095.0, (v_pin / 2.90) * 4095.0)
    ax.plot(v_pin, adc_raw, color=C_ACCENT, lw=2.2, label="Resposta Real SAR ADC1 ESP32-S3 (ADC_11db)")
    ax.axvspan(2.90, 3.35, color=C_WARN, alpha=0.18, label="Zona de Saturação (4095 fixo > 2,90 V)")
    ax.axvline(2.571, color=C_OK, ls="-", lw=1.8, label="VBAT 12,6V c/ 39k/10k v2.1 = 2,57 V (100% Linear!)")
    ax.axvline(2.68, color="#7E22CE", ls="--", lw=1.6, label="Topo Gimbals c/ 560 Ω v2.1 = 2,28..2,68 V (Zero Perda!)")
    ax.set_title("4. Adequação Analógica v2.1 ao Limite Linear do ADC1 (<= 2,90 V)", fontsize=10, fontweight="bold")
    ax.set_xlabel("Tensão no Pino ADC1 (V)")
    ax.set_ylabel("Contagem Digital (0..4095)")
    ax.set_xlim(0, 3.35)
    ax.set_ylim(0, 4350)
    ax.grid(True, alpha=0.35)
    ax.legend(fontsize=7.5, loc="lower right")

    fig.tight_layout(rect=[0.01, 0.01, 0.99, 0.90])
    out = os.path.join(OUT_DIR, "cad_07_validacao_engenharia_v21.png")
    fig.savefig(out)
    plt.close(fig)
    return out


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    generators = [
        generate_cad_00_drone_overview,
        generate_cad_01_inspecao_ipex_buzzer,
        generate_cad_02_chapa_central,
        generate_cad_03_braco_e_motor,
        generate_cad_04_placa_fc_70x90,
        generate_cad_05_controle_ground_tx,
        generate_cad_06_gangorra_1eixo,
        generate_cad_07_validacao_engenharia_v21,
    ]
    for gen in generators:
        path = gen()
        print(f"[OK] Generated {os.path.relpath(path, REPO_ROOT)}")


if __name__ == "__main__":
    main()
