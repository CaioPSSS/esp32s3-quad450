#!/usr/bin/env python3
"""
Analisador de Caixa-Preta (Blackbox PSRAM) — Quadricóptero 450 mm ESP32-S3
==========================================================================
Lê arquivos exportados pela CLI USB (`bb_dump_csv` ou `bb_dump_bin`) e executa:
  1. Verificação de integridade (CRC-16-CCITT p/ dumps binários `BBOX`).
  2. Espectro FFT de Vibração (Giroscópio Bruto vs. Filtrado pelo Biquad 2ª Ordem).
  3. Rastreamento da Malha PID (Setpoint vs. Taxa Medida + Termos P / I / D).
  4. Desempenho dos 4 Motores (`M1..M4`) & Queda de Tensão `VBAT` 3S.
  5. Auditoria de Tempo Real (`dt_us` no Core 1 @ 1000 Hz: média, P99 e máximo).

Exemplos de uso:
  python3 tools/analyze_blackbox.py voo1.csv --save-png voo1_relatorio.png
  python3 tools/analyze_blackbox.py voo1.bin
  python3 tools/analyze_blackbox.py --self-test
"""

from __future__ import annotations

import argparse
import csv
import io
import math
import struct
import sys
import tempfile
from dataclasses import dataclass
from pathlib import Path
from typing import List


# Layout exato de `BlackboxFrame` (64 bytes, little-endian, #pragma pack(1)):
#   uint32 timestamp_us, uint16 dt_us, uint8 state, uint8 flags,
#   float gyro_raw[3], float gyro_filt[3],
#   int16 rate_sp_mrads[2], int16 pid_p[2], int16 pid_i[2], int16 pid_d[2],
#   int16 euler_deg100[3], uint16 motor_permil[4], uint16 vbat_mv
FRAME_STRUCT = struct.Struct("<IHBB6f11h5H")
HEADER_STRUCT = struct.Struct("<4sHHHHI")
assert FRAME_STRUCT.size == 64, f"Tamanho inesperado de frame: {FRAME_STRUCT.size}"
assert HEADER_STRUCT.size == 16


@dataclass
class BlackboxDataset:
    sample_rate_hz: float
    t_s: List[float]
    dt_us: List[int]
    gx_raw_dps: List[float]
    gy_raw_dps: List[float]
    gz_raw_dps: List[float]
    gx_filt_dps: List[float]
    gy_filt_dps: List[float]
    gz_filt_dps: List[float]
    rate_sp_r_dps: List[float]
    rate_sp_p_dps: List[float]
    pid_p_r: List[float]
    pid_i_r: List[float]
    pid_d_r: List[float]
    roll_deg: List[float]
    pitch_deg: List[float]
    yaw_deg: List[float]
    motors_pct: List[List[float]]  # 4 listas [M1, M2, M3, M4]
    vbat_v: List[float]


def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def parse_binary_blackbox(raw: bytes) -> BlackboxDataset:
    idx = raw.find(b"BBOX")
    if idx < 0 or len(raw) < idx + HEADER_STRUCT.size + 2:
        raise ValueError("Assinatura binária 'BBOX' não encontrada no arquivo.")

    magic, ver, fsize, rate_hz, _res, count = HEADER_STRUCT.unpack_from(raw, idx)
    if magic != b"BBOX" or ver != 1 or fsize != 64:
        raise ValueError(f"Cabeçalho BBOX incompatível: ver={ver}, frame_size={fsize}")

    payload_len = HEADER_STRUCT.size + count * 64
    if len(raw) < idx + payload_len + 2:
        raise ValueError("Arquivo binário truncado antes do CRC16 final.")

    expected_crc = struct.unpack_from("<H", raw, idx + payload_len)[0]
    calc_crc = crc16_ccitt(raw[idx : idx + payload_len])
    if calc_crc != expected_crc:
        raise ValueError(f"Falha de CRC-16! Calculado=0x{calc_crc:04X} != Esperado=0x{expected_crc:04X}")

    rad2deg = 180.0 / math.pi
    t_s, dt_us = [], []
    gx_r, gy_r, gz_r, gx_f, gy_f, gz_f = [], [], [], [], [], []
    sp_r, sp_p, p_r, i_r, d_r = [], [], [], [], []
    roll, pitch, yaw, vbat = [], [], [], []
    motors = [[], [], [], []]

    t0_us = None
    offset = idx + HEADER_STRUCT.size
    for _ in range(count):
        vals = FRAME_STRUCT.unpack_from(raw, offset)
        offset += 64
        ts_us = vals[0]
        if t0_us is None:
            t0_us = ts_us
        t_s.append(((ts_us - t0_us) & 0xFFFFFFFF) * 1e-6)
        dt_us.append(vals[1])

        gx_r.append(vals[4] * rad2deg)
        gy_r.append(vals[5] * rad2deg)
        gz_r.append(vals[6] * rad2deg)
        gx_f.append(vals[7] * rad2deg)
        gy_f.append(vals[8] * rad2deg)
        gz_f.append(vals[9] * rad2deg)

        sp_r.append(vals[10] * 0.001 * rad2deg)
        sp_p.append(vals[11] * 0.001 * rad2deg)
        p_r.append(vals[12] * 0.1)
        i_r.append(vals[14] * 0.1)
        d_r.append(vals[16] * 0.1)

        roll.append(vals[18] * 0.01)
        pitch.append(vals[19] * 0.01)
        yaw.append(vals[20] * 0.01)

        for m in range(4):
            motors[m].append(vals[21 + m] * 0.1)
        vbat.append(vals[25] * 0.001)

    return BlackboxDataset(
        sample_rate_hz=float(rate_hz or 500),
        t_s=t_s,
        dt_us=dt_us,
        gx_raw_dps=gx_r,
        gy_raw_dps=gy_r,
        gz_raw_dps=gz_r,
        gx_filt_dps=gx_f,
        gy_filt_dps=gy_f,
        gz_filt_dps=gz_f,
        rate_sp_r_dps=sp_r,
        rate_sp_p_dps=sp_p,
        pid_p_r=p_r,
        pid_i_r=i_r,
        pid_d_r=d_r,
        roll_deg=roll,
        pitch_deg=pitch,
        yaw_deg=yaw,
        motors_pct=motors,
        vbat_v=vbat,
    )


def parse_csv_blackbox(text: str) -> BlackboxDataset:
    rate_hz = 500.0
    csv_lines: List[str] = []
    in_block = False

    for raw_line in text.splitlines():
        line = raw_line.strip()
        if not line:
            continue
        if line.startswith("# BLACKBOX_CSV_BEGIN"):
            in_block = True
            for tok in line.split():
                if tok.startswith("rate_hz="):
                    rate_hz = float(tok.split("=", 1)[1])
            continue
        if line.startswith("# BLACKBOX_CSV_END"):
            break
        if line.startswith("#"):
            continue
        if in_block or line.startswith("timestamp_us,") or (csv_lines and "," in line):
            csv_lines.append(line)

    if len(csv_lines) < 2:
        raise ValueError("Nenhuma tabela CSV válida de Blackbox encontrada.")

    rad2deg = 180.0 / math.pi
    t_s, dt_us = [], []
    gx_r, gy_r, gz_r, gx_f, gy_f, gz_f = [], [], [], [], [], []
    sp_r, sp_p, p_r, i_r, d_r = [], [], [], [], []
    roll, pitch, yaw, vbat = [], [], [], []
    motors = [[], [], [], []]

    t0_us = None
    reader = csv.DictReader(io.StringIO("\n".join(csv_lines)))
    for row in reader:
        ts_us = int(row["timestamp_us"])
        if t0_us is None:
            t0_us = ts_us
        t_s.append((ts_us - t0_us) * 1e-6)
        dt_us.append(int(row["dt_us"]))

        gx_r.append(float(row["gx_raw"]) * rad2deg)
        gy_r.append(float(row["gy_raw"]) * rad2deg)
        gz_r.append(float(row["gz_raw"]) * rad2deg)
        gx_f.append(float(row["gx_filt"]) * rad2deg)
        gy_f.append(float(row["gy_filt"]) * rad2deg)
        gz_f.append(float(row["gz_filt"]) * rad2deg)

        sp_r.append(int(row["rate_sp_r_mrads"]) * 0.001 * rad2deg)
        sp_p.append(int(row["rate_sp_p_mrads"]) * 0.001 * rad2deg)
        p_r.append(int(row["pid_p_r"]) * 0.1)
        i_r.append(int(row["pid_i_r"]) * 0.1)
        d_r.append(int(row["pid_d_r"]) * 0.1)

        roll.append(int(row["roll_deg100"]) * 0.01)
        pitch.append(int(row["pitch_deg100"]) * 0.01)
        yaw.append(int(row["yaw_deg100"]) * 0.01)

        for m, col in enumerate(("m1_permil", "m2_permil", "m3_permil", "m4_permil")):
            motors[m].append(int(row[col]) * 0.1)
        vbat.append(int(row["vbat_mv"]) * 0.001)

    return BlackboxDataset(
        sample_rate_hz=rate_hz,
        t_s=t_s,
        dt_us=dt_us,
        gx_raw_dps=gx_r,
        gy_raw_dps=gy_r,
        gz_raw_dps=gz_r,
        gx_filt_dps=gx_f,
        gy_filt_dps=gy_f,
        gz_filt_dps=gz_f,
        rate_sp_r_dps=sp_r,
        rate_sp_p_dps=sp_p,
        pid_p_r=p_r,
        pid_i_r=i_r,
        pid_d_r=d_r,
        roll_deg=roll,
        pitch_deg=pitch,
        yaw_deg=yaw,
        motors_pct=motors,
        vbat_v=vbat,
    )


def compute_dft_spectrum(signal: List[float], fs_hz: float):
    """Calcula magnitude unilateral do espectro (usa numpy.fft se instalado, ou DFT rápida)."""
    n = len(signal)
    if n < 8:
        return [0.0], [0.0]
    mean_v = sum(signal) / n
    centered = [x - mean_v for x in signal]

    try:
        import numpy as np  # type: ignore

        win = np.hanning(n)
        spec = np.abs(np.fft.rfft(np.array(centered) * win)) * (2.0 / max(np.sum(win), 1e-9))
        freqs = np.fft.rfftfreq(n, d=1.0 / fs_hz)
        return freqs.tolist(), spec.tolist()
    except ImportError:
        # Fallback puro em Python caso numpy não esteja presente
        n_use = min(n, 512)
        freqs, mags = [], []
        for k in range(n_use // 2 + 1):
            re = im = 0.0
            for idx in range(n_use):
                w = 0.5 * (1.0 - math.cos(2.0 * math.pi * idx / max(n_use - 1, 1)))
                ang = -2.0 * math.pi * k * idx / n_use
                re += centered[idx] * w * math.cos(ang)
                im += centered[idx] * w * math.sin(ang)
            freqs.append(k * fs_hz / n_use)
            mags.append(math.hypot(re, im) * (4.0 / n_use))
        return freqs, mags


def print_engineering_report(ds: BlackboxDataset) -> dict:
    n = len(ds.t_s)
    dur_s = ds.t_s[-1] - ds.t_s[0] if n > 1 else 0.0
    sorted_dt = sorted(ds.dt_us)
    dt_mean = sum(ds.dt_us) / max(n, 1)
    dt_p99 = sorted_dt[min(int(0.99 * n), n - 1)] if n else 0
    dt_max = sorted_dt[-1] if n else 0

    freqs, mag_raw = compute_dft_spectrum(ds.gx_raw_dps, ds.sample_rate_hz)
    _, mag_flt = compute_dft_spectrum(ds.gx_filt_dps, ds.sample_rate_hz)

    # Energia de vibração estrutural/hélice medida estritamente acima de 20 Hz
    # (separa manobras legítimas de pilotagem <10 Hz da vibração mecânica!)
    hf_sq_raw = sum((m * m) * 0.5 for f, m in zip(freqs, mag_raw) if f >= 20.0)
    hf_sq_flt = sum((m * m) * 0.5 for f, m in zip(freqs, mag_flt) if f >= 20.0)
    rms_raw = math.sqrt(hf_sq_raw)
    rms_flt = math.sqrt(hf_sq_flt)
    atten_db = 20.0 * math.log10(max(rms_flt, 1e-6) / max(rms_raw, 1e-6))

    peak_hz = 0.0
    peak_amp = 0.0
    for f, m in zip(freqs, mag_raw):
        if f >= 20.0 and m > peak_amp:
            peak_amp = m
            peak_hz = f

    rms_track = math.sqrt(
        sum((sp - gy) ** 2 for sp, gy in zip(ds.rate_sp_r_dps, ds.gx_filt_dps)) / max(n, 1)
    )
    avg_motors = [sum(m) / max(n, 1) for m in ds.motors_pct]
    motor_spread = max(avg_motors) - min(avg_motors)

    print("=================================================================")
    print("   RELATORIO DE ENGENHARIA BLACKBOX — QUAD-450 ESP32-S3")
    print("=================================================================")
    print(f"  Quadros analisados   : {n} ({dur_s:.2f} s @ {ds.sample_rate_hz:.0f} Hz)")
    print(f"  Saude Loop Core 1    : media={dt_mean:.1f} us | P99={dt_p99} us | max={dt_max} us")
    print(f"  Vibracao Roll Gyro   : Bruto={rms_raw:.2f} deg/s RMS -> Filtrado={rms_flt:.2f} deg/s RMS ({atten_db:+.1f} dB)")
    print(f"  Pico Dominante FFT   : {peak_hz:.1f} Hz (amplitude {peak_amp:.2f} deg/s)")
    print(f"  Erro Rastreamento PID: {rms_track:.2f} deg/s RMS")
    print(
        f"  Equilibrio Motores   : M1={avg_motors[0]:.1f}% M2={avg_motors[1]:.1f}% "
        f"M3={avg_motors[2]:.1f}% M4={avg_motors[3]:.1f}% (spread={motor_spread:.1f}%)"
    )
    print(f"  Bateria LiPo 3S      : min={min(ds.vbat_v):.2f} V | max={max(ds.vbat_v):.2f} V")
    print("=================================================================")

    return {
        "frames": n,
        "dt_p99_us": dt_p99,
        "peak_hz": peak_hz,
        "atten_db": atten_db,
        "freqs": freqs,
        "mag_raw": mag_raw,
        "mag_flt": mag_flt,
    }


def render_plots(ds: BlackboxDataset, stats: dict, save_png: str | None) -> None:
    try:
        import matplotlib
        if save_png:
            matplotlib.use("Agg")
        import matplotlib.pyplot as plt  # type: ignore
    except ImportError:
        print("[AVISO] `matplotlib` nao instalado; relatorio numerico completo impresso acima.")
        return

    fig, axes = plt.subplots(2, 2, figsize=(14, 9))
    fig.suptitle("Quad-450 ESP32-S3 — Analise de Voo Blackbox PSRAM", fontsize=13, fontweight="bold")

    # 1. FFT de Vibração
    ax = axes[0, 0]
    ax.plot(stats["freqs"], stats["mag_raw"], label="Gyro Roll Bruto", alpha=0.65, color="#d9534f")
    ax.plot(stats["freqs"], stats["mag_flt"], label="Gyro Roll Pos-Biquad", linewidth=1.6, color="#0275d8")
    ax.set_title("1. Espectro FFT de Vibracao (Chassi / Helices)")
    ax.set_xlabel("Frequencia (Hz)")
    ax.set_ylabel("Amplitude (deg/s)")
    ax.grid(True, alpha=0.3)
    ax.legend()

    # 2. Rastreamento PID Roll
    ax = axes[0, 1]
    ax.plot(ds.t_s, ds.rate_sp_r_dps, "--", label="Setpoint Taxa Roll (deg/s)", color="#292b2c")
    ax.plot(ds.t_s, ds.gx_filt_dps, label="Gyro Roll Filtrado (deg/s)", color="#5cb85c")
    ax.plot(ds.t_s, ds.pid_p_r, label="PID P (%)", alpha=0.55)
    ax.plot(ds.t_s, ds.pid_d_r, label="PID D (%)", alpha=0.55)
    ax.set_title("2. Rastreamento de Malha Interna PID (Roll)")
    ax.set_xlabel("Tempo (s)")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper right", fontsize=8)

    # 3. Motores M1..M4 + VBAT
    ax = axes[1, 0]
    for m, lbl in enumerate(("M1 (FR CCW)", "M2 (RR CW)", "M3 (RL CCW)", "M4 (FL CW)")):
        ax.plot(ds.t_s, ds.motors_pct[m], label=lbl, linewidth=1.1)
    ax.set_title("3. Comando dos 4 Motores (%) & Bateria 3S")
    ax.set_xlabel("Tempo (s)")
    ax.set_ylabel("Potencia Motor (%)")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper left", fontsize=8)
    ax2 = ax.twinx()
    ax2.plot(ds.t_s, ds.vbat_v, ":", color="purple", label="VBAT (V)")
    ax2.set_ylabel("VBAT 3S (V)", color="purple")

    # 4. Histograma dt_us
    ax = axes[1, 1]
    ax.hist(ds.dt_us, bins=30, color="#f0ad4e", edgecolor="black", alpha=0.85)
    ax.axvline(1000, color="green", linestyle="--", label="Alvo 1000 us (1 kHz)")
    ax.set_title("4. Determinismo Tempo Real Core 1 (dt_us)")
    ax.set_xlabel("Periodo do Loop (us)")
    ax.set_ylabel("Contagem de Ciclos")
    ax.grid(True, alpha=0.3)
    ax.legend()

    fig.tight_layout()
    if save_png:
        fig.savefig(save_png, dpi=140)
        print(f"[OK] Grafico salvo em: {save_png}")
    else:
        plt.show()


def run_self_test() -> int:
    """Gera um log binário sintético de 1.0s @ 500 Hz com vibração de motor em 120 Hz e valida tudo!"""
    count = 500
    fs = 500
    hdr = HEADER_STRUCT.pack(b"BBOX", 1, 64, fs, 0, count)
    frames = bytearray()
    deg2rad = math.pi / 180.0

    for k in range(count):
        t = k / fs
        ts_us = k * 2000
        # Sinal real de curva suave (15 deg/s @ 2 Hz) + ruído forte de hélice em 120 Hz!
        clean_rads = 15.0 * deg2rad * math.sin(2.0 * math.pi * 2.0 * t)
        vib_120hz = 25.0 * deg2rad * math.sin(2.0 * math.pi * 120.0 * t)
        raw_gx = clean_rads + vib_120hz
        filt_gx = clean_rads + 0.12 * vib_120hz  # Biquad 80 Hz atenuando 120 Hz!

        frame = FRAME_STRUCT.pack(
            ts_us,
            1000 + (k % 5) - 2,
            4,
            0x0B,
            raw_gx, 0.0, 0.0,
            filt_gx, 0.0, 0.0,
            int(round(clean_rads * 1000.0)), 0,
            40, 0, 10, 0, -8, 0,
            500, 0, 0,
            380, 382, 379, 381,
            11650,
        )
        frames.extend(frame)

    body = hdr + bytes(frames)
    crc = struct.pack("<H", crc16_ccitt(body))

    with tempfile.NamedTemporaryFile(suffix=".bin", delete=True) as tmp:
        tmp.write(body + crc)
        tmp.flush()
        ds = parse_binary_blackbox(Path(tmp.name).read_bytes())

    stats = print_engineering_report(ds)
    assert stats["frames"] == 500
    assert abs(stats["peak_hz"] - 120.0) <= 2.5, f"Pico FFT inesperado: {stats['peak_hz']}"
    assert stats["atten_db"] < -10.0, f"Atenuacao esperada < -10 dB, obtido {stats['atten_db']}"
    print("[SELF-TEST PASS] Parser binario BBOX + CRC-16 + FFT 120 Hz validados com sucesso!")
    return 0


def load_blackbox_file(path: Path) -> BlackboxDataset:
    raw = path.read_bytes()
    if b"BBOX" in raw[:256]:
        return parse_binary_blackbox(raw)
    return parse_csv_blackbox(raw.decode("utf-8", errors="replace"))


def main() -> int:
    ap = argparse.ArgumentParser(description="Analisador FFT/PID de Blackbox PSRAM — Quad-450 ESP32-S3")
    ap.add_argument("logfile", nargs="?", help="Arquivo .csv ou .bin exportado pela CLI USB")
    ap.add_argument("--save-png", help="Salva os 4 graficos em arquivo PNG sem abrir janela")
    ap.add_argument("--no-plot", action="store_true", help="Imprime apenas o relatorio numerico no terminal")
    ap.add_argument("--self-test", action="store_true", help="Executa auto-teste end-to-end (CRC16 + FFT)")
    args = ap.parse_args()

    if args.self_test:
        return run_self_test()

    if not args.logfile:
        ap.print_help()
        return 1

    ds = load_blackbox_file(Path(args.logfile))
    stats = print_engineering_report(ds)
    if not args.no_plot:
        render_plots(ds, stats, args.save_png)
    return 0


if __name__ == "__main__":
    sys.exit(main())
