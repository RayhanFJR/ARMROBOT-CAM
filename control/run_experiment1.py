"""
run_experiment.py
Alur: muat URDF -> definisikan obstacle statis -> tentukan start & tujuan ->
RRT-Connect + trajektori polinom -> verifikasi -> (plot) -> upload ke ESP32 -> jalankan.

Mode tujuan:
  --goal-q  40 20 -30 10 0          : berikan joint angles langsung (deg)
  --goal-xyz 0.25 0.20 0.12         : IK otomatis (posisi EE dalam meter)
  --goal-xyz 0.25 0.20 0.12 --pitch 0 : IK + kunci pitch tool ke 0 deg
  --goal-tracker target.json        : target dari kamera (tombol 'g' di tracker),
                                      dikonversi board -> base, lalu IK
  --z-offset 0.05                   : tambah tinggi EE (m) di atas titik target

Contoh:
  python run_experiment.py --dry-run --plot --goal-xyz 0.25 0.20 0.12
  python run_experiment.py --dry-run --plot --goal-tracker target.json --z-offset 0.05
  python run_experiment.py --port COM5 --goal-tracker target.json --z-offset 0.05
  python run_experiment.py --port COM5 --goal-q 40 20 -30 10 0
"""
import argparse
import json
import math
import sys

import numpy as np

from arm_planner import Arm, Box, Sphere, plan_trajectory, traj_quintic

# =============================================================================
# KONFIGURASI  (meter, frame base URDF)
# =============================================================================
URDF = "arm.urdf"                   # ganti dengan URDF asli dari MATLAB
EE_NAME = None                      # None = ujung rantai otomatis
N_ARM = 5                           # joint yang direncanakan (gripper terpisah, dalam %)

# Obstacle (METER, frame base). Kosong = tidak ada obstacle.
# Contoh balok 4x4x4 cm, 20 cm di depan base:
#   Box(center=[0.20, 0.00, 0.02], size=[0.04, 0.04, 0.04])
OBSTACLES = [
    # Box(center=[0.20, 0.00, 0.02], size=[0.04, 0.04, 0.04]),
    # Sphere(center=[0.2, -0.1, 0.1], radius=0.04),
]
LINK_RADIUS = 0.025                 # kira-kira setengah tebal link (m)
MARGIN = 0.010                      # jarak aman tambahan (m)
FLOOR_Z = None                      # None = matikan collision lantai

# --- Transformasi frame board (ChArUco) -> frame base robot ---------------------
# UKUR dengan penggaris: dari titik base robot ke origin board (pojok (0,0) ChArUco).
BOARD_ORIGIN = (0.20, 0.00, 0.00)   # origin board di frame base (m)
BOARD_YAW_DEG = 0.0                 # rotasi sumbu X board terhadap X base (deg, sekitar Z base)

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
    [ -93,  84],  # yaw       (servo PWM: 75-561, home=330)
    [-100,  77],  # shoulder  (servo PWM: 75-565, home=350)
    [ -79,  98],  # elbow     (servo PWM: 75-565, home=295)
    [ -90,  87],  # wrist     (servo PWM: 75-565, home=325)
    [   0,   0],  # ee (dikunci)
]

VMAX_DPS = 50.0                     # batas kecepatan tiap joint (deg/s)
AMAX_DPS2 = 100.0                   # batas percepatan tiap joint (deg/s^2)
DT = 0.02                           # selang sampel trajektori (s)
MAX_POINTS = 1200                   # kapasitas buffer ESP32 (TRJ_MAX di firmware)

# Pencocokan konvensi URDF <-> servo (q_esp = SIGN*q_urdf + OFFSET)
SIGN = (1, 1, 1, 1, 1)
OFFSET = (0, 0, 0, 0, 0)

# =============================================================================
# PICK & PLACE  — ubah nilai di sini untuk tuning
# =============================================================================
GRIP_CLOSE_Q  = -120      # nilai Q gripper saat capit  (kirim "Q 6 <val>")
GRIP_OPEN_Q   = -200        # nilai Q gripper saat buka
BOOM_UP = 13
WRIST_UP = 30
PICK_DELAY_S  = 0.5      # delay (detik) di posisi pick sebelum capit
PLACE_DELAY_S  = 2.5      # delay (detik) di posisi pick sebelum capit
PICK_TIMEOUT  = 10.0     # timeout tunggu gerakan gripper (detik)


def tracker_to_base(path, z_offset=0.0, scale_x=1.0, scale_y=1.0,
                    offset_x=0.0, offset_y=0.0):
    """Baca target.json dari tracker (mm, frame board) -> posisi (m) di frame base.

    Untuk sekarang: konversi sederhana mm->m, negate X (sumbu X board
    berlawanan dengan X base robot). Y tetap, Z dari z_offset.
    scale_x/scale_y = faktor koreksi empiris (kamera cenderung under-report).
    offset_x/offset_y = offset tetap (mm) di frame base robot.
    """
    with open(path) as f:
        d = json.load(f)
    # mm -> m, negate X, apply scale + offset correction
    return np.array([-d["x_mm"] * scale_x / 1000.0 + offset_x / 1000.0,
                      d["y_mm"] * scale_y / 1000.0 + offset_y / 1000.0,
                      z_offset])

    # --- transformasi penuh board->base (belum diaktifkan) ---
    # Aktifkan blok ini kalau BOARD_ORIGIN dan BOARD_YAW_DEG sudah diukur.
    # p = np.array([-d["x_mm"] / 1000.0, d["y_mm"] / 1000.0,
    #               d.get("plane_z_mm", 0.0) / 1000.0])
    # c, s = math.cos(math.radians(BOARD_YAW_DEG)), math.sin(math.radians(BOARD_YAW_DEG))
    # Rz = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    # P = Rz @ p + np.array(BOARD_ORIGIN, dtype=float)
    # P[2] += z_offset
    # return P


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
    ap.add_argument("--goal-tracker", default=None, help="baca tujuan dari target.json (hasil tracker kamera)")
    ap.add_argument("--z-offset", type=float, default=0.0, help="tambahan tinggi EE (m) di atas target tracker, mis. 0.05")
    ap.add_argument("--safe-z", type=float, default=None,
                    help="ketinggian aman (m) untuk approach 2-fase: yaw swing dulu di safe-z, baru turun ke target. "
                         "Contoh: --safe-z 0.20")
    ap.add_argument("--pitch", type=float, default=None, help="pitch tool (deg) untuk IK, opsional")
    ap.add_argument("--grip", type=float, default=0.0, help="gripper 0..100 %% (0 = buka)")
    ap.add_argument("--scale-x", type=float, default=1.0,
                    help="faktor koreksi skala X (kamera->robot), default 1.0")
    ap.add_argument("--scale-y", type=float, default=1.0,
                    help="faktor koreksi skala Y (kamera->robot), default 1.0")
    ap.add_argument("--offset-x", type=float, default=0.0,
                    help="offset koreksi X dalam mm (frame base robot), default 0")
    ap.add_argument("--offset-y", type=float, default=0.0,
                    help="offset koreksi Y dalam mm (frame base robot), default 0")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--yes", "-y", action="store_true", help="skip konfirmasi, langsung jalankan")
    a = ap.parse_args()

    if not a.dry_run and not a.port:
        sys.exit("Isi --port, atau pakai --dry-run.")
    if a.goal_q is None and a.goal_xyz is None and a.goal_tracker is None:
        sys.exit("Isi --goal-q, --goal-xyz, atau --goal-tracker.")

    # ---- target dari tracker kamera (board -> base) ----
    if a.goal_tracker:
        try:
            a.goal_xyz = tracker_to_base(a.goal_tracker, a.z_offset,
                                         a.scale_x, a.scale_y,
                                         a.offset_x, a.offset_y).tolist()
        except (OSError, KeyError, ValueError) as e:
            sys.exit(f"Gagal baca {a.goal_tracker}: {e}")
        print("Target dari tracker (base, m):", np.round(a.goal_xyz, 4))

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
                "Saran: cek LIMITS_DEG sudah sesuai servo fisik, cek BOARD_ORIGIN/BOARD_YAW_DEG, "
                "atau coba ubah target xyz / --z-offset."
            )
        print("IK -> q goal (deg):", np.round(np.degrees(q_goal), 1))
        print("      posisi EE (m) :", np.round(arm.fk(q_goal)[:3, 3], 4))

    # ---- planning ----
    vmax = np.full(arm.n, math.radians(VMAX_DPS))
    amax = np.full(arm.n, math.radians(AMAX_DPS2))

    # === Approach 2-fase: hover di safe-z, lalu turun ke target ===
    if a.safe_z is not None and a.goal_q is None:
        hover_xyz = list(a.goal_xyz)
        hover_xyz[2] = a.safe_z          # X,Y target tapi Z tinggi (aman)
        print(f"Approach 2-fase: hover di z={a.safe_z} m, lalu turun ke z={a.goal_xyz[2]:.4f} m")

        q_hover = arm.solve_ik(hover_xyz, a.pitch, q0=q_start, seed=a.seed)
        if q_hover is None:
            sys.exit(
                f"IK gagal untuk posisi HOVER {np.round(hover_xyz, 4)}.\n"
                "Saran: turunkan --safe-z atau dekatkan target ke base."
            )
        print("IK hover (deg):", np.round(np.degrees(q_hover), 1),
              "| EE (m):", np.round(arm.fk(q_hover)[:3, 3], 4))

        # Jalur 3 waypoint: start -> hover -> target
        # Setelah trajectory DONE, servo 2 akan diangkat via command "Q 2"
        path = [q_start, q_hover, q_goal]
        t, q, qd, qdd = traj_quintic(path, vmax, amax, dt=DT)
        res = dict(t=t, q=q, qd=qd, qdd=qdd, path=path, mode="quintic-2fase")

    else:
        # === Planning biasa (RRT-Connect) ===
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
    if not a.yes:
        ans = input(f"Kirim {len(ts)} sampel ({t[-1]:.1f} s) dan JALANKAN? Pastikan area aman. [y/N] ")
        if ans.lower() != "y":
            print("Dibatalkan.")
            return
    grip_a = GRIP_OPEN_Q if (a.safe_z is not None and a.goal_q is None) else a.grip
    link.upload(Qs, grip_a, dt_send)
    print("Upload selesai. Menjalankan... (Ctrl+C = STOP)")
    link.go()
    try:
        link.wait_done(timeout=t[-1] + 60)
        print("Trajektori A (pick approach) selesai.")

        # === FULL PICK & PLACE SEQUENCE ===
        if a.safe_z is not None and a.goal_q is None:
            import time as _time

            # --- Step 4: Delay di posisi target ---
            print(">> DELAY %.1f detik (stabilkan posisi)..." % PICK_DELAY_S)
            _time.sleep(PICK_DELAY_S)

            # --- Step 5: Capit (grip close) ---
            print(">> GRIP CLOSE (Q 6 %s)..." % GRIP_CLOSE_Q)
            link.send_move_cmd("Q 6 %s" % GRIP_CLOSE_Q, timeout=PICK_TIMEOUT)
            print(">> Grip selesai.")
            _time.sleep(0.5)   # stabilkan grip

            link.send_move_cmd("Q 2 %s" % BOOM_UP, timeout=PICK_TIMEOUT)
            link.send_move_cmd("Q 4 %s" % WRIST_UP, timeout=PICK_TIMEOUT)

            # --- Step 6-8: Return trajectory ---
            # Hitung posisi home di Cartesian untuk hover balik
            
            home_xyz = arm.fk(q_start)[:3, 3]
            home_hover_xyz = [float(home_xyz[0]), float(home_xyz[1]), a.safe_z]

            q_home_hover = arm.solve_ik(home_hover_xyz, a.pitch,
                                        q0=q_hover, seed=a.seed)
            if q_home_hover is None:
                print("!! IK home-hover gagal, langsung ke HOME (m).")
                link.send_move_cmd("m", timeout=15.0)
            else:
                print(">> RETURN: target -> hover -> home_hover -> home")
                print("   home_hover (deg):", np.round(np.degrees(q_home_hover), 1),
                      "| EE (m):", np.round(arm.fk(q_home_hover)[:3, 3], 4))

                # Baca posisi aktual (setelah grip + gerakan tambahan)
                q_now_deg, _ = link.get_q()
                q_now = np.radians(q_now_deg)
                print("   Posisi aktual (deg):", np.round(q_now_deg, 1))

                # Bangun trajectory B: posisi_skg -> q_hover -> q_home_hover -> q_start
                path_b = [q_now, q_hover, q_home_hover, q_start]
                t_b, q_b, qd_b, qdd_b = traj_quintic(path_b, vmax, amax, dt=DT)
                print("   Traj B: %d sampel, %.1f detik" % (len(t_b), t_b[-1]))

                # Resample
                dt_b = max(DT, t_b[-1] / (MAX_POINTS - 1))
                ts_b = np.arange(0.0, t_b[-1], dt_b)
                ts_b = np.append(ts_b, t_b[-1])
                Qs_b = np.degrees(np.column_stack(
                    [np.interp(ts_b, t_b, q_b[:, j]) for j in range(arm.n)]))

                # Upload & jalankan Traj B (grip tetap CLOSE)
                link.upload(Qs_b, GRIP_CLOSE_Q, dt_b)
                print(">> Traj B upload selesai. Menjalankan return...")
                link.go()
                link.wait_done(timeout=t_b[-1] + 60)
                print(">> Return selesai (sudah di HOME).")

            # --- Step 9: Place ---
            link.send_move_cmd("Q 2 %s" % BOOM_UP, timeout=PICK_TIMEOUT)
            print(">> Menuju PLACE (p)...")
            link.send_move_cmd("p", timeout=15.0)
            print(">> DELAY %.1f detik (stabilkan posisi)..." % PLACE_DELAY_S)
            _time.sleep(PLACE_DELAY_S)
            print(">> GRIP OPEN (Q 6 %s)..." % GRIP_OPEN_Q)
            link.send_move_cmd("Q 6 %s" % GRIP_OPEN_Q, timeout=PICK_TIMEOUT)
            print(">> PLACE selesai.")

            # --- Step 10: Home ---
            print(">> Kembali ke HOME (m)...")
            link.send_move_cmd("m", timeout=15.0)
            print(">> HOME tercapai. Siklus PICK & PLACE SELESAI!")

        else:
            print("Selesai.")

    except KeyboardInterrupt:
        link.stop()
        print("STOP dikirim.")
    finally:
        link.close()


if __name__ == "__main__":
    main()