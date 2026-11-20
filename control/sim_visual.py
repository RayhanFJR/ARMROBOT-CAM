"""
sim_visual.py
Simulasi visual 3D robot arm dengan animasi pergerakan.

Cara pakai:
  python sim_visual.py --goal-xyz 0.10 0.18 0.14
  python sim_visual.py --goal-xyz -0.07 0.22 0.17
  python sim_visual.py --goal-q 28.5 12.4 -26.4 -3.6 0
  python sim_visual.py --goal-xyz 0.10 0.18 0.14 --pitch 0
  python sim_visual.py --start-q 30 10 -20 0 0 --goal-xyz 0.10 0.18 0.14

Fitur:
  Visualisasi 3D lengan robot (link, joint, EE)
  Animasi quintic trajectory dari start ke goal
  Obstacle ditampilkan
  Target marker (bola merah)
  Trail lintasan EE
  Info joint angles dan posisi EE di setiap frame
"""
import argparse
import math
import sys
import os

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from arm_planner import Arm, Box, Sphere, plan_trajectory

# =============================================================================
# KONFIGURASI -- samakan dengan run_experiment.py
# =============================================================================
URDF = os.path.join(os.path.dirname(os.path.abspath(__file__)), "arm.urdf")
N_ARM = 5
OBSTACLES = [
    Box(center=[0.22, 0.00, 0.04], size=[0.04, 0.12, 0.08]),
]
LINK_RADIUS = 0.025
MARGIN = 0.010
FLOOR_Z = None
LIMITS_DEG = [
    [-180, 180],  # yaw
    [-180, 180],  # shoulder
    [-180, 180],  # elbow
    [-180, 180],  # wrist
    [   0,   0],  # ee (dikunci)
]
VMAX_DPS = 60.0
AMAX_DPS2 = 150.0
DT = 0.02

LINK_COLORS = ["#2196F3", "#4CAF50", "#FF9800", "#E91E63", "#9C27B0"]
JOINT_COLOR = "#333333"


def draw_obstacles(ax, obstacles):
    for o in obstacles:
        if isinstance(o, Box):
            c, h = o.center, o.size / 2
            corners = np.array([c + h * np.array([sx, sy, sz])
                               for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])
            faces = [[0,1,3,2],[4,5,7,6],[0,1,5,4],[2,3,7,6],[0,2,6,4],[1,3,7,5]]
            ax.add_collection3d(Poly3DCollection(
                [corners[f] for f in faces],
                alpha=0.2, facecolor="orange", edgecolor="k", lw=0.8))
        else:
            u, w = np.mgrid[0:2*np.pi:16j, 0:np.pi:8j]
            ax.plot_wireframe(
                o.center[0] + o.radius * np.cos(u) * np.sin(w),
                o.center[1] + o.radius * np.sin(u) * np.sin(w),
                o.center[2] + o.radius * np.cos(w),
                color="orange", lw=0.5, alpha=0.3)


def draw_floor(ax, sz=0.35):
    xx, yy = np.meshgrid(np.linspace(-sz, sz, 2), np.linspace(-sz, sz, 2))
    ax.plot_surface(xx, yy, np.zeros_like(xx), alpha=0.08, color="gray")


def draw_arm(ax, arm, q, link_lines, joint_dots, ee_dot):
    _, pts = arm.frames(q)
    for i in range(len(pts) - 1):
        seg = np.array([pts[i], pts[i + 1]])
        link_lines[i].set_data_3d(seg[:, 0], seg[:, 1], seg[:, 2])
    for i in range(len(pts)):
        joint_dots[i].set_data_3d([pts[i, 0]], [pts[i, 1]], [pts[i, 2]])
    ee_dot.set_data_3d([pts[-1, 0]], [pts[-1, 1]], [pts[-1, 2]])
    return pts


def main():
    ap = argparse.ArgumentParser(description="Simulasi visual 3D robot arm")
    ap.add_argument("--start-q", nargs=N_ARM, type=float, default=None,
                    help="Konfigurasi awal (deg)")
    ap.add_argument("--goal-q", nargs=N_ARM, type=float, help="Tujuan joint (deg)")
    ap.add_argument("--goal-xyz", nargs=3, type=float, help="Tujuan EE (m)")
    ap.add_argument("--pitch", type=float, default=None, help="Pitch tool (deg)")
    ap.add_argument("--speed", type=float, default=1.0,
                    help="Kecepatan animasi (1=realtime, 2=2x, 0.5=lambat)")
    ap.add_argument("--seed", type=int, default=0)
    a = ap.parse_args()

    if a.goal_q is None and a.goal_xyz is None:
        sys.exit("Isi --goal-q atau --goal-xyz.")

    arm = Arm(URDF, n_arm=N_ARM, link_radius=LINK_RADIUS, margin=MARGIN,
              floor_z=FLOOR_Z, obstacles=OBSTACLES, limits_deg=LIMITS_DEG)
    print(f"URDF dimuat: {arm.n} joint {arm.joint_names}, ee = {arm.ee_name}")

    q_start = np.radians(a.start_q) if a.start_q else np.zeros(arm.n)

    if a.goal_q is not None:
        q_goal = np.radians(a.goal_q)
    else:
        print(f"Menghitung IK ke target {a.goal_xyz} ...")
        q_goal = arm.solve_ik(a.goal_xyz, a.pitch, q0=q_start, seed=a.seed)
        if q_goal is None:
            sys.exit("IK gagal: target di luar jangkauan atau tabrakan.")
        print(f"IK berhasil -> q (deg): {np.round(np.degrees(q_goal), 1)}")
        print(f"              EE (m) : {np.round(arm.fk(q_goal)[:3, 3], 4)}")

    # Planning trajectory
    vmax = np.full(arm.n, math.radians(VMAX_DPS))
    amax = np.full(arm.n, math.radians(AMAX_DPS2))
    try:
        res = plan_trajectory(arm, q_start, q_goal, vmax, amax, dt=DT, seed=a.seed)
    except (ValueError, RuntimeError) as e:
        sys.exit(f"Planning gagal: {e}")

    t = res["t"]
    Q = res["q"]
    print(f"Trajektori: {len(t)} sampel, durasi {t[-1]:.2f} s")

    # Subsample for smooth animation (~30 fps visual)
    fps = 30
    dt_anim = 1.0 / fps
    step = max(1, int(DT * fps / a.speed))
    idx_anim = np.arange(0, len(t), step)
    if idx_anim[-1] != len(t) - 1:
        idx_anim = np.append(idx_anim, len(t) - 1)
    Q_anim = Q[idx_anim]

    # Pre-compute EE trail
    ee_trail = np.array([arm.fk(q)[:3, 3] for q in Q])

    # ---- Setup figure ----
    fig = plt.figure(figsize=(10, 8))
    fig.patch.set_facecolor("#1a1a2e")
    ax = fig.add_subplot(111, projection="3d")
    ax.set_facecolor("#16213e")
    ax.xaxis.pane.fill = False
    ax.yaxis.pane.fill = False
    ax.zaxis.pane.fill = False

    # Floor and obstacles
    draw_floor(ax)
    draw_obstacles(ax, OBSTACLES)

    # Target marker
    target_pos = arm.fk(q_goal)[:3, 3]
    ax.scatter(*target_pos, s=80, c="red", marker="*", zorder=10, label="Target")

    # Start marker
    start_pos = arm.fk(q_start)[:3, 3]
    ax.scatter(*start_pos, s=40, c="lime", marker="D", zorder=10, label="Start EE")

    # Initialize arm drawing objects
    _, pts0 = arm.frames(q_start)
    n_pts = len(pts0)

    link_lines = []
    for i in range(n_pts - 1):
        col = LINK_COLORS[i % len(LINK_COLORS)]
        ln, = ax.plot([0, 0], [0, 0], [0, 0], "-", color=col, lw=4, solid_capstyle="round")
        link_lines.append(ln)

    joint_dots = []
    for i in range(n_pts):
        d, = ax.plot([0], [0], [0], "o", color=JOINT_COLOR, ms=6, zorder=5)
        joint_dots.append(d)

    ee_dot, = ax.plot([0], [0], [0], "o", color="#FF5722", ms=10, zorder=6, label="EE")

    # EE trail line
    trail_line, = ax.plot([], [], [], "-", color="#FF5722", lw=1.5, alpha=0.6)

    # Ghost start and goal arm (semi-transparent)
    for q_ghost, col_g in [(q_start, "lime"), (q_goal, "cyan")]:
        _, gpts = arm.frames(q_ghost)
        ax.plot(gpts[:, 0], gpts[:, 1], gpts[:, 2], "--o",
                color=col_g, alpha=0.2, ms=2, lw=1.5)

    # Axis setup
    all_ee = np.vstack([ee_trail, [[0, 0, 0]], [target_pos]])
    rng_val = max(np.ptp(all_ee, axis=0).max(), 0.3) / 2 * 1.1
    mid = (all_ee.max(0) + all_ee.min(0)) / 2
    ax.set_xlim(mid[0] - rng_val, mid[0] + rng_val)
    ax.set_ylim(mid[1] - rng_val, mid[1] + rng_val)
    ax.set_zlim(0, 2 * rng_val)
    ax.set_xlabel("X (m)", color="white")
    ax.set_ylabel("Y (m)", color="white")
    ax.set_zlabel("Z (m)", color="white")
    ax.tick_params(colors="gray")
    ax.legend(loc="upper left", fontsize=8, facecolor="#1a1a2e", labelcolor="white")

    # Title text
    title = ax.set_title("", color="white", fontsize=11, pad=15)

    # Info text
    info_text = fig.text(0.02, 0.02, "", fontsize=8, color="#aaaaaa",
                         fontfamily="monospace", verticalalignment="bottom")

    frame_count = len(Q_anim)

    def update(frame):
        q = Q_anim[frame]
        pts = draw_arm(ax, arm, q, link_lines, joint_dots, ee_dot)

        # Trail up to current frame
        real_idx = idx_anim[frame]
        trail_line.set_data_3d(ee_trail[:real_idx + 1, 0],
                               ee_trail[:real_idx + 1, 1],
                               ee_trail[:real_idx + 1, 2])

        # Update title
        time_now = t[real_idx]
        title.set_text(
            f"Robot Arm Simulation  |  t = {time_now:.2f} / {t[-1]:.2f} s  |  "
            f"frame {frame + 1}/{frame_count}")

        # Info
        q_deg = np.degrees(q)
        ee = pts[-1]
        err = np.linalg.norm(ee - target_pos) * 1000
        info_text.set_text(
            f"Joint (deg): [{q_deg[0]:+6.1f}, {q_deg[1]:+6.1f}, {q_deg[2]:+6.1f}, "
            f"{q_deg[3]:+6.1f}, {q_deg[4]:+6.1f}]\n"
            f"EE (cm)    : X={ee[0]*100:+6.1f}  Y={ee[1]*100:+6.1f}  Z={ee[2]*100:+6.1f}  "
            f"R={np.linalg.norm(ee)*100:.1f}\n"
            f"Target (cm): X={target_pos[0]*100:+6.1f}  Y={target_pos[1]*100:+6.1f}  "
            f"Z={target_pos[2]*100:+6.1f}  |  Error: {err:.1f} mm")

        return link_lines + joint_dots + [ee_dot, trail_line]

    # Draw initial frame
    update(0)

    anim = animation.FuncAnimation(fig, update, frames=frame_count,
                                   interval=int(dt_anim * 1000), blit=False,
                                   repeat=True)

    plt.tight_layout()
    print("Menampilkan animasi... Tutup jendela untuk keluar.")
    plt.show()


if __name__ == "__main__":
    main()
