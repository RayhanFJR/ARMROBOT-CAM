"""
arm_planner.py
Planner lengan robot dari URDF: FK, IK numerik (DLS + restart),
collision check (obstacle statis), RRT-Connect di joint space,
dan trajektori polinom (quintic / cubic spline).

Hanya butuh: numpy, scipy.  Satuan: meter, radian di dalam; derajat di antarmuka luar.

Model collision: tiap link dianggap kapsul (segmen antar-frame joint + radius LINK_RADIUS),
obstacle berupa Box (AABB) atau Sphere pada frame base, ditambah lantai z >= floor_z.
Antarmuka in_collision(q) bisa diganti dengan PyBullet/FCL kalau nanti butuh yang lebih presisi.

IK solver (solve_ik):
  - Residual: e = p_target - p_ee(q)       [dimensi 3, atau 4 bila pitch dikunci]
  - Jacobian numerik: J[:, k] = dp/dq_k   (bukan ∂residual/∂q — perhatikan tanda)
  - Update    : dq = Jᵀ (J Jᵀ + λI)⁻¹ e   (Damped Least Squares / Levenberg–Marquardt)
  - Restart acak bila konvergensi gagal
"""
import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass

import numpy as np


# ----------------------------------------------------------------------------
# Utilitas geometri
# ----------------------------------------------------------------------------
def _rpy_to_R(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def _axis_R(axis, ang):
    k = np.asarray(axis, float)
    k = k / np.linalg.norm(k)
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(ang) * K + (1 - math.cos(ang)) * (K @ K)


@dataclass
class Box:
    center: np.ndarray   # (3,) meter, frame base
    size: np.ndarray     # (3,) panjang sisi penuh (x, y, z)

    def __post_init__(self):
        self.center = np.asarray(self.center, float)
        self.size = np.asarray(self.size, float)


@dataclass
class Sphere:
    center: np.ndarray
    radius: float

    def __post_init__(self):
        self.center = np.asarray(self.center, float)


# ----------------------------------------------------------------------------
# Model lengan
# ----------------------------------------------------------------------------
class Arm:
    def __init__(self, urdf_path, ee=None, n_arm=5, link_radius=0.025, margin=0.01,
                 floor_z=0.0, obstacles=None, limits_deg=None):
        root = ET.parse(urdf_path).getroot()
        joints, by_child, parents = {}, {}, set()
        for j in root.findall("joint"):
            o = j.find("origin")
            xyz = [float(v) for v in (o.get("xyz", "0 0 0") if o is not None else "0 0 0").split()]
            rpy = [float(v) for v in (o.get("rpy", "0 0 0") if o is not None else "0 0 0").split()]
            T0 = np.eye(4)
            T0[:3, :3] = _rpy_to_R(*rpy)
            T0[:3, 3] = xyz
            ax = j.find("axis")
            axis = [float(v) for v in ax.get("xyz").split()] if ax is not None else [1.0, 0.0, 0.0]
            lim = j.find("limit")
            lo = hi = None
            if lim is not None and lim.get("lower") is not None:
                lo, hi = float(lim.get("lower")), float(lim.get("upper"))
            d = dict(name=j.get("name"), type=j.get("type"), parent=j.find("parent").get("link"),
                     child=j.find("child").get("link"), T0=T0, axis=np.array(axis), lo=lo, hi=hi)
            d["mov"] = d["type"] in ("revolute", "continuous", "prismatic")
            joints[d["name"]] = d
            by_child[d["child"]] = d
            parents.add(d["parent"])

        links = [l.get("name") for l in root.findall("link")]
        leaves = [l for l in links if l not in parents]
        if ee is None:
            ee = leaves[-1]
            if len(leaves) > 1:
                print(f"[Arm] Ada {len(leaves)} ujung rantai, dipakai: {ee} (set ee=... untuk memilih)")
        chain, cur = [], ee
        while cur in by_child:
            chain.insert(0, by_child[cur])
            cur = by_child[cur]["parent"]
        self.chain = chain
        self.ee_name = ee

        mov = [j for j in chain if j["mov"]]
        if len(mov) < n_arm:
            n_arm = len(mov)
        if len(mov) > n_arm:
            print(f"[Arm] URDF punya {len(mov)} joint bergerak; {n_arm} pertama dipakai planner, sisanya dibekukan di 0.")
        self.n = n_arm
        self.joint_names = [j["name"] for j in mov][:n_arm]

        lim = np.zeros((n_arm, 2))
        for i, j in enumerate(mov[:n_arm]):
            lo, hi = j["lo"], j["hi"]
            if lo is None or j["type"] == "continuous" or hi - lo < 1e-6:
                lo, hi = -math.pi, math.pi
            lim[i] = (lo, hi)
        if limits_deg is not None:
            lim = np.radians(np.asarray(limits_deg, float))
        self.lim = lim

        self.link_radius = link_radius
        self.margin = margin
        self.floor_z = floor_z
        self.obstacles = list(obstacles or [])

    # ---------------- kinematika ----------------
    def frames(self, q):
        """q (rad, panjang n). Return (T_ee, titik-titik origin tiap frame (M,3))."""
        T = np.eye(4)
        pts = [T[:3, 3].copy()]
        k = 0
        for j in self.chain:
            T = T @ j["T0"]
            if j["mov"]:
                a = q[k] if k < self.n else 0.0
                k += 1
                Tj = np.eye(4)
                if j["type"] == "prismatic":
                    Tj[:3, 3] = j["axis"] / np.linalg.norm(j["axis"]) * a
                else:
                    Tj[:3, :3] = _axis_R(j["axis"], a)
                T = T @ Tj
            pts.append(T[:3, 3].copy())
        return T, np.array(pts)

    def fk(self, q):
        return self.frames(q)[0]

    def tool_pitch(self, q):
        """Elevasi sumbu-x frame EE terhadap horizontal (rad); + = menghadap atas."""
        R = self.fk(q)[:3, :3]
        return math.atan2(R[2, 0], math.hypot(R[0, 0], R[1, 0]))

    # ---------------- collision ----------------
    @staticmethod
    def _sample(pts, ds=0.01):
        out = []
        for a, b in zip(pts[1:-1], pts[2:]):
            L = np.linalg.norm(b - a)
            n = max(2, int(L / ds) + 1)
            t = np.linspace(0, 1, n)[:, None]
            out.append(a + (b - a) * t)
        return np.vstack(out)

    def in_collision(self, q):
        _, pts = self.frames(q)
        P = self._sample(pts)
        if self.floor_z is not None and (P[:, 2] < self.floor_z - 1e-9).any():
            return True
        r = self.link_radius + self.margin
        for o in self.obstacles:
            if isinstance(o, Box):
                d = np.linalg.norm(np.maximum(np.abs(P - o.center) - o.size / 2, 0), axis=1)
            else:
                d = np.linalg.norm(P - o.center, axis=1) - o.radius
            if (d < r).any():
                return True
        return False

    def within_limits(self, q, tol=1e-6):
        return bool(np.all(q >= self.lim[:, 0] - tol) and np.all(q <= self.lim[:, 1] + tol))

    def valid(self, q):
        return self.within_limits(q) and not self.in_collision(q)

    def edge_free(self, qa, qb, step=0.05):
        n = max(1, int(np.ceil(np.max(np.abs(qb - qa)) / step)))
        for t in np.linspace(0, 1, n + 1)[1:]:
            if self.in_collision(qa + (qb - qa) * t):
                return False
        return True

    # ---------------- Jacobian numerik ----------------
    def _jacobian_pos(self, q, h=1e-6):
        """Jacobian posisi EE: J (3 x n), J[:,k] = ∂p_ee/∂q_k."""
        p0 = self.fk(q)[:3, 3]
        J = np.zeros((3, self.n))
        for k in range(self.n):
            dq = np.zeros(self.n)
            dq[k] = h
            J[:, k] = (self.fk(q + dq)[:3, 3] - p0) / h
        return J

    def _jacobian_pitch(self, q, h=1e-6):
        """Gradient pitch tool terhadap q: shape (n,)."""
        p0 = self.tool_pitch(q)
        g = np.zeros(self.n)
        for k in range(self.n):
            dq = np.zeros(self.n)
            dq[k] = h
            g[k] = (self.tool_pitch(q + dq) - p0) / h
        return g

    # ---------------- IK numerik (posisi + pitch tool opsional) ----------------
    def solve_ik(self, xyz, pitch_deg=None, q0=None, tries=50, seed=0,
                 pos_tol=1e-3, max_iter=300):
        """
        IK berbasis Damped Least Squares (DLS / Levenberg–Marquardt) dengan restart acak.

        Parameter
        ---------
        xyz       : target posisi EE (m), frame base URDF
        pitch_deg : kunci elevasi pitch tool (deg). None = bebas.
        q0        : tebakan awal (rad). Jika None, dimulai dari tengah range.
        tries     : jumlah restart acak bila belum konvergen
        seed      : RNG seed untuk restart
        pos_tol   : toleransi posisi (m)
        max_iter  : iterasi DLS per percobaan

        Return
        ------
        q (rad) yang valid (dalam limit & bebas tabrakan), atau None bila gagal.
        """
        rng   = np.random.default_rng(seed)
        xyz   = np.asarray(xyz, float)
        mid   = 0.5 * (self.lim[:, 0] + self.lim[:, 1])   # tengah range tiap joint
        wp    = 0.05        # bobot pitch (agar sebanding dengan satuan posisi ~meter)
        lam0  = 1e-3        # damping awal
        lam_max = 0.5       # damping maksimal saat Jacobian kecil

        # ---- kandidat tebakan awal ----
        # t==0 : q0 (dari luar) atau tengah range
        # t==1 : konfigurasi zero
        # t>=2 : acak tapi sedikit bias ke tengah range
        def _init_q(t):
            if t == 0:
                return q0.copy() if q0 is not None else mid.copy()
            if t == 1:
                return np.zeros(self.n)
            # 70 % acak murni, 30 % dibias ke tengah
            alpha = 0.3
            return np.clip(
                alpha * mid + (1 - alpha) * rng.uniform(self.lim[:, 0], self.lim[:, 1]),
                self.lim[:, 0], self.lim[:, 1]
            )

        for t in range(tries):
            q = _init_q(t)

            for _ in range(max_iter):
                # ---- hitung residual posisi ----
                T   = self.fk(q)
                e_p = xyz - T[:3, 3]           # (3,)

                # ---- Jacobian posisi ----
                J = self._jacobian_pos(q)       # (3, n)

                if pitch_deg is not None:
                    # tambah baris untuk pitch
                    e_r = np.array([wp * (math.radians(pitch_deg) - self.tool_pitch(q))])
                    g_r = wp * self._jacobian_pitch(q)[None, :]  # (1, n)
                    e_full = np.concatenate([e_p, e_r])
                    J_full = np.vstack([J, g_r])                 # (4, n)
                else:
                    e_full = e_p
                    J_full = J

                # ---- cek konvergensi ----
                pos_err = np.linalg.norm(e_p)
                pitch_ok = (pitch_deg is None or
                            abs(math.radians(pitch_deg) - self.tool_pitch(q)) < math.radians(1.0))
                if pos_err < pos_tol and pitch_ok:
                    break

                # ---- DLS update: dq = Jᵀ (J Jᵀ + λI)⁻¹ e ----
                # λ adaptif: besar bila J kecil (dekat singularitas)
                jjt_norm = np.linalg.norm(J_full @ J_full.T)
                lam = lam0 + lam_max * math.exp(-jjt_norm / (lam0 + 1e-9))
                A   = J_full @ J_full.T + lam * np.eye(len(e_full))
                dq  = J_full.T @ np.linalg.solve(A, e_full)

                # ---- step size adaptif ----
                step = min(1.0, 0.3 / (np.linalg.norm(dq) + 1e-9))
                q = np.clip(q + step * dq, self.lim[:, 0], self.lim[:, 1])

            # ---- cek hasil ----
            pos_err = np.linalg.norm(xyz - self.fk(q)[:3, 3])
            pitch_ok = (pitch_deg is None or
                        abs(math.radians(pitch_deg) - self.tool_pitch(q)) < math.radians(2.0))
            if pos_err < pos_tol * 2 and pitch_ok and self.valid(q):
                return q
        return None

    # ---------------- RRT-Connect ----------------
    def plan_path(self, qs, qg, max_iter=6000, step=0.15, seed=0, smooth_iter=200):
        qs, qg = np.asarray(qs, float), np.asarray(qg, float)
        if not self.valid(qs):
            raise ValueError("Konfigurasi AWAL tabrakan / di luar limit (q = %s deg). Cek posisi obstacle atau start-q."
                             % np.round(np.degrees(qs), 1))
        if not self.valid(qg):
            raise ValueError("Konfigurasi TUJUAN tabrakan / di luar limit (q = %s deg)." % np.round(np.degrees(qg), 1))
        if self.edge_free(qs, qg):
            return [qs, qg]

        rng = np.random.default_rng(seed)
        Ta, Pa = [qs], [-1]
        Tb, Pb = [qg], [-1]

        def nearest(T, q):
            return int(np.argmin(np.linalg.norm(np.array(T) - q, axis=1)))

        def extend(T, P, q):
            i = nearest(T, q)
            d = q - T[i]
            dist = np.linalg.norm(d)
            new = q if dist <= step else T[i] + d / dist * step
            if not self.edge_free(T[i], new):
                return None, "trapped"
            T.append(new)
            P.append(i)
            return len(T) - 1, ("reached" if dist <= step else "advanced")

        def backtrack(T, P, i):
            out = []
            while i != -1:
                out.append(T[i])
                i = P[i]
            return out[::-1]

        a_is_start = True
        path = None
        for _ in range(max_iter):
            qr = rng.uniform(self.lim[:, 0], self.lim[:, 1])
            ia, st = extend(Ta, Pa, qr)
            if st != "trapped":
                while True:
                    ib, st2 = extend(Tb, Pb, Ta[ia])
                    if st2 != "advanced":
                        break
                if st2 == "reached":
                    pa, pb = backtrack(Ta, Pa, ia), backtrack(Tb, Pb, ib)
                    # pa: akar Ta -> ia ; pb: akar Tb -> ib (ib ~ ia). Susun jadi start -> goal.
                    path = (pa + pb[::-1][1:]) if a_is_start else (pb + pa[::-1][1:])
                    break
            Ta, Pa, Tb, Pb = Tb, Pb, Ta, Pa
            a_is_start = not a_is_start
        if path is None:
            raise RuntimeError("RRT gagal menemukan jalur (naikkan max_iter / cek obstacle).")
        # pastikan arah start -> goal
        if np.linalg.norm(path[0] - qs) > np.linalg.norm(path[-1] - qs):
            path = path[::-1]
        path[0], path[-1] = qs, qg

        # shortcut smoothing
        for _ in range(smooth_iter):
            if len(path) <= 2:
                break
            i, j = sorted(rng.choice(len(path), 2, replace=False))
            if j - i < 2:
                continue
            if self.edge_free(path[i], path[j]):
                path = path[:i + 1] + path[j:]
        return path


# ----------------------------------------------------------------------------
# Trajektori
# ----------------------------------------------------------------------------
def _finish_grid(total, dt):
    t = np.arange(0.0, total, dt)
    return np.append(t, total)


def traj_quintic(waypoints, vmax, amax, dt=0.02, tmin=0.4):
    """Quintic rest-to-rest per segmen (berhenti di tiap waypoint -> pasti tetap di garis joint-space
    antar waypoint, jadi aman terhadap collision yang sudah dicek)."""
    W = np.asarray(waypoints, float)
    Ts = []
    for a, b in zip(W[:-1], W[1:]):
        D = np.abs(b - a)
        Ts.append(max(tmin, np.max(1.875 * D / vmax), np.max(np.sqrt(5.7735 * D / amax))))
    edges = np.concatenate([[0.0], np.cumsum(Ts)])
    t = _finish_grid(edges[-1], dt)
    q = np.zeros((len(t), W.shape[1]))
    qd, qdd = np.zeros_like(q), np.zeros_like(q)
    for k, T in enumerate(Ts):
        m = (t >= edges[k] - 1e-12) & (t <= edges[k + 1] + 1e-12)
        tau = (t[m] - edges[k]) / T
        s = 10 * tau**3 - 15 * tau**4 + 6 * tau**5
        ds = (30 * tau**2 - 60 * tau**3 + 30 * tau**4) / T
        dds = (60 * tau - 180 * tau**2 + 120 * tau**3) / T**2
        D = W[k + 1] - W[k]
        q[m] = W[k] + np.outer(s, D)
        qd[m] = np.outer(ds, D)
        qdd[m] = np.outer(dds, D)
    return t, q, qd, qdd


def traj_spline(waypoints, vmax, amax, dt=0.02, tmin=0.4):
    """Cubic spline melalui semua waypoint (tidak berhenti di via-point), v=0 di ujung.
    Waktu diskalakan sampai batas kecepatan/percepatan terpenuhi."""
    from scipy.interpolate import CubicSpline
    W = np.asarray(waypoints, float)
    seg = np.maximum(np.max(np.abs(np.diff(W, axis=0)) / vmax, axis=1) * 1.5, tmin)
    tk = np.concatenate([[0.0], np.cumsum(seg)])
    cs = CubicSpline(tk, W, bc_type="clamped")
    tt = np.linspace(0, tk[-1], 3000)
    rv = np.max(np.abs(cs(tt, 1)) / vmax)
    ra = np.max(np.abs(cs(tt, 2)) / amax)
    s = max(1.0, rv, math.sqrt(ra)) * 1.02
    tk = tk * s
    cs = CubicSpline(tk, W, bc_type="clamped")
    t = _finish_grid(tk[-1], dt)
    return t, cs(t), cs(t, 1), cs(t, 2)


def plan_trajectory(arm, q_start, q_goal, vmax, amax, dt=0.02, mode="quintic", seed=0):
    """Return dict(t, q, qd, qdd, path, mode). Semua dalam radian."""
    path = arm.plan_path(q_start, q_goal, seed=seed)
    vmax, amax = np.asarray(vmax, float), np.asarray(amax, float)
    used = mode
    t = q = qd = qdd = None
    if mode == "spline" and len(path) > 2:
        t, q, qd, qdd = traj_spline(path, vmax, amax, dt)
        ok = bool(np.all(q >= arm.lim[:, 0] - 1e-6) and np.all(q <= arm.lim[:, 1] + 1e-6))
        ok = ok and not any(arm.in_collision(x) for x in q[::2])
        if not ok:
            print("[plan] Spline menyimpang ke zona terlarang -> fallback ke quintic via-point berhenti.")
            used = "quintic"
            t = None
    else:
        used = "quintic"
    if t is None:
        t, q, qd, qdd = traj_quintic(path, vmax, amax, dt)

    bad = [i for i, x in enumerate(q) if arm.in_collision(x)]
    if bad:
        raise RuntimeError(f"Verifikasi gagal: {len(bad)} sampel trajektori tabrakan.")
    if not (np.all(q >= arm.lim[:, 0] - 1e-6) and np.all(q <= arm.lim[:, 1] + 1e-6)):
        raise RuntimeError("Verifikasi gagal: trajektori melewati batas joint.")
    return dict(t=t, q=q, qd=qd, qdd=qdd, path=path, mode=used)
