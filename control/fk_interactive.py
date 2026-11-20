"""
fk_interactive.py
Kalkulator FK interaktif di terminal.
Masukkan sudut joint (derajat), langsung tampil posisi EE (X, Y, Z).

Cara pakai:
  python fk_interactive.py
  python fk_interactive.py --step 5        # step +/- per tombol (default 5 deg)
  python fk_interactive.py --start 0 0 0 0 0
"""
import argparse
import sys
import os
import math

import numpy as np

sys.path.insert(0, os.path.dirname(__file__))
from arm_planner import Arm

# =============================================================================
# KONFIGURASI -- samakan dengan run_experiment.py
# =============================================================================
URDF = os.path.join(os.path.dirname(__file__), "arm.urdf")
N_ARM = 5
LIMITS_DEG = [
    [-90,  90],   # yaw
    [-10, 170],   # shoulder
    [-130,  10],  # elbow
    [ -90,  90],  # wrist
    [   0,   0],  # ee (dikunci)
]
JOINT_LABELS = ["yaw", "shoulder", "elbow", "wrist", "ee"]


def print_state(arm, q_deg, step):
    """Tampilkan state FK saat ini."""
    q_rad = np.radians(q_deg)
    T = arm.fk(q_rad)
    xyz = T[:3, 3]
    r = math.sqrt(xyz[0]**2 + xyz[1]**2 + xyz[2]**2)
    valid = arm.valid(q_rad)

    os.system("cls" if os.name == "nt" else "clear")
    print("=" * 58)
    print("   FK INTERAKTIF -- Robot Arm")
    print("=" * 58)
    print(f"\n  {'Joint':<12} {'Nilai':>8}  {'Min':>7}  {'Max':>7}  Bar")
    print(f"  {'-'*54}")
    for i, (label, lim) in enumerate(zip(JOINT_LABELS, arm.lim)):
        lo, hi = math.degrees(lim[0]), math.degrees(lim[1])
        bar_len = 16
        frac = (q_deg[i] - lo) / (hi - lo) if hi > lo else 0.5
        filled = int(max(0, min(bar_len, frac * bar_len)))
        bar = "[" + "#" * filled + "-" * (bar_len - filled) + "]"
        print(f"  [{i}] {label:<10} {q_deg[i]:>+7.1f}  {lo:>6.0f}   {hi:>6.0f}   {bar}")

    print(f"\n  -----------------------------------------------")
    print(f"  End-Effector Position:")
    print(f"    X = {xyz[0]*100:+8.2f} cm   ({xyz[0]:+.4f} m)")
    print(f"    Y = {xyz[1]*100:+8.2f} cm   ({xyz[1]:+.4f} m)")
    print(f"    Z = {xyz[2]*100:+8.2f} cm   ({xyz[2]:+.4f} m)")
    print(f"    R = {r*100:8.2f} cm   (jarak dari origin)")
    status = "OK (valid)" if valid else "!! TABRAKAN / BATAS !!"
    print(f"\n  Status: {status}")
    print(f"  -----------------------------------------------")
    print(f"\n  Kontrol:")
    print(f"    q <idx> <nilai>   set nilai joint (mis: q 0 45)")
    print(f"    q <idx> +         naik {step:.0f} deg  (mis: q 1 +)")
    print(f"    q <idx> -         turun {step:.0f} deg (mis: q 2 -)")
    print(f"    r                 reset semua ke 0")
    print(f"    s                 tampilkan perintah run_experiment")
    print(f"    x                 keluar")
    print()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--step", type=float, default=5.0,
                    help="Step naik/turun per +/- (deg, default 5)")
    ap.add_argument("--start", nargs=N_ARM, type=float, default=None,
                    help=f"Konfigurasi awal {N_ARM} joint (deg)")
    a = ap.parse_args()

    arm = Arm(URDF, n_arm=N_ARM, limits_deg=LIMITS_DEG)
    q_deg = np.array(a.start if a.start else [0.0] * N_ARM)

    print_state(arm, q_deg, a.step)

    while True:
        try:
            cmd = input("  > ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            print("\nKeluar.")
            break

        if not cmd:
            print_state(arm, q_deg, a.step)
            continue

        parts = cmd.split()

        if parts[0] == "x":
            print("Keluar.")
            break

        elif parts[0] == "r":
            q_deg = np.zeros(N_ARM)
            print_state(arm, q_deg, a.step)

        elif parts[0] == "s":
            vals = " ".join(f"{v:.1f}" for v in q_deg)
            xyz = arm.fk(np.radians(q_deg))[:3, 3]
            print(f"\n  -- Salin perintah berikut ke terminal --")
            print(f"  Berdasarkan joint angles:")
            print(f"    python run_experiment.py --port COMx --goal-q {vals}")
            print(f"\n  Berdasarkan posisi EE (via IK):")
            print(f"    python run_experiment.py --port COMx --goal-xyz {xyz[0]:.4f} {xyz[1]:.4f} {xyz[2]:.4f}")
            input("\n  [Enter untuk lanjut] ")
            print_state(arm, q_deg, a.step)

        elif parts[0] == "q" and len(parts) >= 3:
            try:
                idx = int(parts[1])
                if not (0 <= idx < N_ARM):
                    print(f"  !! Index joint harus 0-{N_ARM-1}")
                    continue
                lo = math.degrees(arm.lim[idx, 0])
                hi = math.degrees(arm.lim[idx, 1])
                if parts[2] == "+":
                    q_deg[idx] = np.clip(q_deg[idx] + a.step, lo, hi)
                elif parts[2] == "-":
                    q_deg[idx] = np.clip(q_deg[idx] - a.step, lo, hi)
                else:
                    q_deg[idx] = np.clip(float(parts[2]), lo, hi)
                print_state(arm, q_deg, a.step)
            except ValueError:
                print("  !! Format salah. Contoh: q 0 45  |  q 1 +  |  q 2 -30")
        else:
            print("  !! Perintah tidak dikenal.")


if __name__ == "__main__":
    main()
