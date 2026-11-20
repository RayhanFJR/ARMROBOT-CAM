"""
run_experiment.py
Alur: muat URDF -> definisikan obstacle statis -> tentukan start & tujuan ->
RRT-Connect + trajektori polinom -> verifikasi -> (plot) -> upload ke ESP32 -> jalankan.

Mode tujuan:
  --goal-q  40 20 -30 10 0          : berikan joint angles langsung (deg)
  --goal-xyz 0.25 0.20 0.12         : IK otomatis (posisi EE dalam meter)
  --goal-xyz 0.25 0.20 0.12 --pitch 0 : IK + kunci pitch tool ke 0 deg

Contoh:
  python run_experiment.py --dry-run --plot --goal-xyz 0.25 0.20 0.12
  python run_experiment.py --port COM5 --goal-xyz 0.25 0.20 0.12
  python run_experiment.py --port COM5 --goal-q 40 20 -30 10 0
"""
import argparse
import math
import sys

import numpy as np

from arm_planner import Arm, Box, Sphere, plan_trajectory

# =============================================================================
# KONFIGURASI  (meter, frame base URDF)
# =============================================================================
URDF = "arm.urdf"                   # ganti dengan URDF asli dari MATLAB
EE_NAME = None                      # None = ujung rantai otomatis
N_ARM = 5                           # joint yang direncanakan (gripper terpisah, dalam %)

OBSTACLES = [
    Box(center=[90.22, 0.00, 0.04], size=[0.004, 0.0012, 0.0008]),   # contoh: balok rendah di depan robot (pose home tidak boleh menabraknya)
    # Sphere(center=[0.2, -0.1, 0.1], radius=0.04),
]
LINK_RADIUS = 0.025                 # kira-kira setengah tebal link (m)
MARGIN = 0.010                      # jarak aman tambahan (m)
FLOOR_Z = None                      # None = matikan collision lantai

# Batas gerak (derajat). None = ikut limit URDF.
# URDF ekspor SolidWorks sering kali berisi ±15708 rad (tidak realistis untuk servo fisik).
# Set nilai sesuai batas mekanik / PWM servo aktual.
# Format: [[lo0,hi0], [lo1,hi1], ...] satu baris per joint (urutan sesuai arm.joint_names).
#   joint 0 - yaw      : putar base kiri/kanan
#   joint 1 - shoulder : angkat bahu
#   joint 2 - elbow    : tekuk siku
#   joint 3 - wrist    : tekuk pergelangan
#   joint 4 - ee       : sudah dikunci di URDF (lower=upper=0), tetap sertakan
LIMITS_DEG = [
    [-180, 180],  # yaw
    [-180, 180],  # shoulder
    [-180, 180],  # elbow
    [-180, 180],  # wrist
    [   0,   0],  # ee (dikunci)
]

VMAX_DPS = 60.0                     # batas kecepatan tiap joint (deg/s)
AMAX_DPS2 = 150.0                   # batas percepatan tiap joint (deg/s^2)
DT = 0.02                           # selang sampel trajektori (s)
MAX_POINTS = 1200                   # kapasitas buffer ESP32 (TRJ_MAX di firmware)

# Pencocokan konvensi URDF <-> servo (q_esp = SIGN*q_urdf + OFFSET)
SIGN = (1, 1, 1, 1, 1)
OFFSET = (0, 0, 0, 0, 0)


def plot_all(arm, res, q_start, q_goal, obstacles):
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    t = res["t"]
    fig, ax = plt.subplots(3, 1, sharex=True, figsize=(8, 8))
    ax[0].plot(t, np.degrees(res["q"]))
    ax[0].set_ylabel("q (deg)")
    ax[1].plot(t, np.degrees(res["qd"]))
    ax[1].axhline(VMAX_DPS, color="k", ls="--", lw=0.8)
    ax[1].axhline(-VMAX_DPS, color="k", ls="--", lw=0.8)
    ax[1].set_ylabel("qdot (deg/s)")
    ax[2].plot(t, np.degrees(res["qdd"]))
    ax[2].axhline(AMAX_DPS2, color="k", ls="--", lw=0.8)
    ax[2].axhline(-AMAX_DPS2, color="k", ls="--", lw=0.8)
    ax[2].set_ylabel("qddot (deg/s^2)")
    ax[2].set_xlabel("t (s)")
    ax[0].legend(arm.joint_names, ncol=5, fontsize=8)
    fig.suptitle(f"Trajektori ({res['mode']}), durasi {t[-1]:.2f} s")

    fig2 = plt.figure(figsize=(7, 7))
    a3 = fig2.add_subplot(111, projection="3d")
    ee = np.array([arm.fk(q)[:3, 3] for q in res["q"]])
    a3.plot(ee[:, 0], ee[:, 1], ee[:, 2], "r-", lw=2, label="lintasan EE")
    for q, c in ((q_start, "g"), (q_goal, "b")):
        _, pts = arm.frames(q)
        a3.plot(pts[:, 0], pts[:, 1], pts[:, 2], c + "-o", ms=3)
    for o in obstacles:
        if isinstance(o, Box):
            c, h = o.center, o.size / 2
            v = np.array([[c + h * np.array([sx, sy, sz]) for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]])[0]
            idx = [[0, 1, 3, 2], [4, 5, 7, 6], [0, 1, 5, 4], [2, 3, 7, 6], [0, 2, 6, 4], [1, 3, 7, 5]]
            a3.add_collection3d(Poly3DCollection([v[i] for i in idx], alpha=0.25, facecolor="orange", edgecolor="k"))
        else:
            u, w = np.mgrid[0:2 * np.pi:20j, 0:np.pi:10j]
            a3.plot_wireframe(o.center[0] + o.radius * np.cos(u) * np.sin(w),
                              o.center[1] + o.radius * np.sin(u) * np.sin(w),
                              o.center[2] + o.radius * np.cos(w), color="orange", lw=0.5)
    a3.set_xlabel("x (m)"); a3.set_ylabel("y (m)"); a3.set_zlabel("z (m)")
    allp = np.vstack([ee, np.array([0, 0, 0])])
    r = max(np.ptp(allp, axis=0).max(), 0.3) / 2
    mid = (allp.max(0) + allp.min(0)) / 2
    a3.set_xlim(mid[0] - r, mid[0] + r); a3.set_ylim(mid[1] - r, mid[1] + r); a3.set_zlim(0, 2 * r)
    a3.legend()
    plt.show()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", help="port serial ESP32, mis. COM5 atau /dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--dry-run", action="store_true", help="hitung & verifikasi saja, tanpa kirim ke robot")
    ap.add_argument("--plot", action="store_true")
    ap.add_argument("--mode", choices=["quintic", "spline"], default="quintic")
    ap.add_argument("--start-q", nargs=N_ARM, type=float, help="konfigurasi awal (deg). Default: baca dari ESP / nol")
    ap.add_argument("--goal-q", nargs=N_ARM, type=float, help="tujuan dalam joint (deg)")
    ap.add_argument("--goal-xyz", nargs=3, type=float, help="tujuan ujung gripper (m) -> dihitung dengan IK")
    ap.add_argument("--pitch", type=float, default=None, help="pitch tool (deg) untuk IK, opsional")
    ap.add_argument("--grip", type=float, default=0.0, help="gripper 0..100 %% (0 = buka)")
    ap.add_argument("--seed", type=int, default=0)
    a = ap.parse_args()

    if not a.dry_run and not a.port:
        sys.exit("Isi --port, atau pakai --dry-run.")
    if a.goal_q is None and a.goal_xyz is None:
        sys.exit("Isi --goal-q atau --goal-xyz.")

    arm = Arm(URDF, ee=EE_NAME, n_arm=N_ARM, link_radius=LINK_RADIUS, margin=MARGIN,
              floor_z=FLOOR_Z, obstacles=OBSTACLES, limits_deg=LIMITS_DEG)
    print(f"URDF dimuat: {arm.n} joint {arm.joint_names}, ee = {arm.ee_name}")

    link = None
    if a.port:
        from esp_link import ArmLink
        link = ArmLink(a.port, a.baud, SIGN, OFFSET)
        link.ping()
        print("ESP32 terhubung.")

    # ---- start ----
    if a.start_q is not None:
        q_start = np.radians(a.start_q)
    elif link is not None:
        qd_, _ = link.get_q()
        q_start = np.radians(qd_)
        print("Start dari ESP (posisi terakhir yang diperintahkan, deg):", np.round(qd_, 1))
    else:
        q_start = np.zeros(arm.n)
    # ---- goal ----
    if a.goal_q is not None:
        q_goal = np.radians(a.goal_q)
    else:
        q_goal = arm.solve_ik(a.goal_xyz, a.pitch, q0=q_start, seed=a.seed)
        if q_goal is None:
            sys.exit(
                "IK gagal: target mungkin di luar jangkauan lengan, terlalu dekat obstacle "
                "(jarak aman = LINK_RADIUS + MARGIN), atau pitch tidak mungkin dicapai.\n"
                "Saran: cek LIMITS_DEG sudah sesuai servo fisik, atau coba ubah target xyz."
            )
        print("IK -> q goal (deg):", np.round(np.degrees(q_goal), 1))
        print("      posisi EE (m) :", np.round(arm.fk(q_goal)[:3, 3], 4))

    # ---- planning ----
    vmax = np.full(arm.n, math.radians(VMAX_DPS))
    amax = np.full(arm.n, math.radians(AMAX_DPS2))
    try:
        res = plan_trajectory(arm, q_start, q_goal, vmax, amax, dt=DT, mode=a.mode, seed=a.seed)
    except (ValueError, RuntimeError) as e:
        sys.exit(f"Planning gagal: {e}")
    t = res["t"]
    print(f"Jalur: {len(res['path'])} waypoint | mode: {res['mode']} | durasi: {t[-1]:.2f} s | sampel: {len(t)}")
    print(f"Maks |v| = {np.degrees(np.abs(res['qd']).max()):.1f} deg/s, maks |a| = {np.degrees(np.abs(res['qdd']).max()):.1f} deg/s^2")
    print("Verifikasi collision & limit: OK")

    # ---- resample ke dt kirim (kalau lebih dari kapasitas buffer ESP) ----
    dt_send = max(DT, t[-1] / (MAX_POINTS - 1))
    ts = np.arange(0.0, t[-1], dt_send)
    ts = np.append(ts, t[-1])
    Qs = np.degrees(np.column_stack([np.interp(ts, t, res["q"][:, j]) for j in range(arm.n)]))
    if dt_send > DT:
        print(f"Trajektori panjang: dt kirim dinaikkan ke {dt_send * 1000:.0f} ms (ESP interpolasi linear).")

    if a.plot:
        plot_all(arm, res, q_start, q_goal, OBSTACLES)

    if a.dry_run:
        print("Dry-run selesai (tidak ada yang dikirim).")
        return

    # ---- upload & jalankan ----
    ans = input(f"Kirim {len(ts)} sampel ({t[-1]:.1f} s) dan JALANKAN? Pastikan area aman. [y/N] ")
    if ans.lower() != "y":
        print("Dibatalkan.")
        return
    link.upload(Qs, a.grip, dt_send)
    print("Upload selesai. Menjalankan... (Ctrl+C = STOP)")
    link.go()
    try:
        link.wait_done(timeout=t[-1] + 60)
        print("Selesai.")
    except KeyboardInterrupt:
        link.stop()
        print("STOP dikirim.")
    finally:
        link.close()


if __name__ == "__main__":
    main()
