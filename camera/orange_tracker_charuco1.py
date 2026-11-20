"""
Tracking benda ORANYE -> koordinat X,Y (mm) di bidang board ChArUco.

  1. python calibrate_camera.py --source 3      (sekali saja -> calib.npz)
  2. Letakkan board RATA di bidang kerja end effector, kamera diam.
  3. python orange_tracker_charuco1.py --source 3
  4. Tekan 'b' (board harus terlihat) -> pose board disimpan ke pose.npz.
     Setelah itu board boleh diambil, ASAL kamera TIDAK bergeser.
  5. Taruh objek oranye, tekan 'g' -> posisi (median 15 frame) disimpan ke target.json
     lalu dipakai: python run_experiment.py --goal-tracker target.json

Tombol: b = set pose board (rata-rata 30 frame), g = simpan target.json,
        r = rekam CSV, q = keluar

Yang dilacak adalah blob oranye TERBESAR di gambar; posisinya = titik tengah blob.
Slider H min/H max/S min/V min untuk menyetel warna oranye sesuai pencahayaan
(lihat jendela "mask": blob oranye harus putih, yang lain hitam).

Koordinat (X,Y) dalam mm pada sistem koordinat board (sumbu digambar di layar:
merah = X, hijau = Y, biru = Z). --plane-z adalah tinggi titik oranye terhadap
bidang board (mm). Z board mengarah masuk ke board, jadi benda yang lebih dekat
ke kamera dari board memakai nilai NEGATIF. Titik tengah blob diukur pada
bidang ini, jadi isi sesuai tinggi permukaan oranye yang dilacak.
"""
import argparse
import csv
import json
import time
import math
import subprocess
import os
import sys
from collections import deque

import cv2
cv2.utils.logging.setLogLevel(cv2.utils.logging.LOG_LEVEL_ERROR)
import numpy as np

from charuco_common import (add_board_args, make_detector, open_cap,
                            detect_charuco)

MIN_AREA = 30      # piksel; blob oranye lebih kecil dari ini diabaikan
HIST_LEN = 15      # jumlah frame terakhir untuk median target
HIST_MIN = 5       # minimal frame valid sebelum 'g' diterima
TARGET_FILE = "target.json"


def make_mask(hsv, h_lo, h_hi, s_min, v_min):
    mask = cv2.inRange(hsv, np.array((h_lo, s_min, v_min)),
                       np.array((h_hi, 255, 255)))
    k = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
    return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)


def find_blob(mask):
    """Titik tengah kontur terbesar (px) dan area-nya, atau (None, 0)."""
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts:
        return None, 0
    c = max(cnts, key=cv2.contourArea)
    area = cv2.contourArea(c)
    if area < MIN_AREA:
        return None, 0
    m = cv2.moments(c)
    if m["m00"] == 0:
        return None, 0
    return (m["m10"] / m["m00"], m["m01"] / m["m00"]), area


def pixel_to_board(pt, K, dist, R, t, z0):
    """Piksel -> (X, Y) mm di bidang Z=z0 pada frame board (ray-plane)."""
    p = np.array([[pt]], dtype=np.float64)
    n = cv2.undistortPoints(p, K, dist).reshape(2)   # koordinat ternormalisasi
    d_cam = np.array([n[0], n[1], 1.0])
    origin = -R.T @ t                                # posisi kamera di frame board
    d_board = R.T @ d_cam
    s = (z0 - origin[2]) / d_board[2]
    P = origin + s * d_board
    return float(P[0]), float(P[1])


def main():
    ap = argparse.ArgumentParser()
    add_board_args(ap)
    ap.add_argument("--calib", default="calib.npz")
    ap.add_argument("--pose", default="pose.npz")
    ap.add_argument("--target", default=TARGET_FILE,
                    help="file keluaran target (tombol g)")
    ap.add_argument("--plane-z", type=float, default=0.0,
                    help="tinggi titik oranye terhadap board (mm)")
    ap.add_argument("--manual-pose", action="store_true",
                    help="set pose board manual via slider (tanpa ChArUco board)")
    ap.add_argument("--depth", type=float, default=645.0,
                    help="kedalaman baseplate dari kamera (mm), default 645")
    ap.add_argument("--scale-x", type=float, default=1.0,
                    help="faktor koreksi skala X (kamera->robot)")
    ap.add_argument("--scale-y", type=float, default=1.0,
                    help="faktor koreksi skala Y (kamera->robot)")
    ap.add_argument("--offset-x", type=float, default=0.0,
                    help="offset koreksi X dalam mm (frame base robot)")
    ap.add_argument("--offset-y", type=float, default=0.0,
                    help="offset koreksi Y dalam mm (frame base robot)")
    ap.add_argument("--auto-run", action="store_true",
                    help="tekan g = langsung jalankan planning + kirim ke robot")
    ap.add_argument("--port", default=None,
                    help="port serial ESP32 (wajib kalau --auto-run)")
    ap.add_argument("--z-offset", type=float, default=0.05,
                    help="tinggi EE di atas target (m), default 0.05")
    ap.add_argument("--ar-pitch", type=float, default=None,
                    help="pitch tool (deg) untuk IK saat auto-run, opsional")
    ap.add_argument("--safe-z", type=float, default=None,
                    help="ketinggian aman (m) approach 2-fase: swing dulu, baru turun. Contoh: 0.20")
    ap.add_argument("--control-dir",
                    default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "control"),
                    help="direktori program kontrol (berisi run_experiment1.py)")
    a = ap.parse_args()

    if a.auto_run and not a.port:
        sys.exit("--auto-run memerlukan --port (port serial ESP32).")

    c = np.load(a.calib)
    K, dist = c["K"].astype(np.float64), c["dist"].astype(np.float64)
    board, det = make_detector(a)
    cap = open_cap(a)

    ok, frame = cap.read()
    if ok and (frame.shape[1], frame.shape[0]) != tuple(c["size"]):
        print("PERINGATAN: resolusi %dx%d != resolusi kalibrasi %s. "
              "Hasil akan salah!" % (frame.shape[1], frame.shape[0],
                                     tuple(c["size"])))

    rvec = tvec = None
    try:
        p = np.load(a.pose)
        rvec, tvec = p["rvec"], p["tvec"]
        print("Pose board dimuat dari", a.pose)
    except OSError:
        pass

    samples = []
    n_collect = 30
    collecting = False
    recording, fh, writer, t0 = False, None, None, time.time()
    hist = deque(maxlen=HIST_LEN)       # posisi (X,Y) mm terakhir
    msg, msg_until = "", 0.0            # pesan sementara di layar

    win = "Orange tracker (ChArUco)"
    cv2.namedWindow(win)
    cv2.createTrackbar("H min", win, 5, 179, lambda v: None)
    cv2.createTrackbar("H max", win, 25, 179, lambda v: None)
    cv2.createTrackbar("S min", win, 120, 255, lambda v: None)
    cv2.createTrackbar("V min", win, 120, 255, lambda v: None)

    # --- trackbar manual pose (hanya muncul kalau --manual-pose) ---
    if a.manual_pose:
        cv2.createTrackbar("Origin X", win, 500, 1000, lambda v: None)   # 500 = 0 mm
        cv2.createTrackbar("Origin Y", win, 500, 1000, lambda v: None)   # 500 = 0 mm
        cv2.createTrackbar("Depth Z", win, int(a.depth), 1500, lambda v: None)
        # Rotasi board->kamera: 180 deg sekitar X (Y flip, Z flip)
        R_manual = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]], dtype=np.float64)
        rvec_manual, _ = cv2.Rodrigues(R_manual)
        print("Mode MANUAL POSE aktif. Atur Origin X/Y dan Depth Z lewat slider.")
        print("  X = kanan dari kamera, Y = atas dari kamera, Z = kedalaman")
        print("  Tekan b untuk simpan pose manual ke", a.pose)

    while True:
        ok, frame = cap.read()
        if not ok:
            break
        t_frame = time.time()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # --- manual pose: update rvec/tvec dari slider tiap frame ---
        if a.manual_pose:
            ox = cv2.getTrackbarPos("Origin X", win) - 500   # mm, 0 = tengah
            oy = cv2.getTrackbarPos("Origin Y", win) - 500   # mm, 0 = tengah
            dz = cv2.getTrackbarPos("Depth Z", win)          # mm
            if dz < 10: dz = 10  # minimal 10 mm
            rvec = rvec_manual.copy()
            tvec = np.array([[float(ox)], [float(-oy)], [float(dz)]], dtype=np.float64)
            # oy di-negate karena slider "Origin Y" = atas = -Y_kamera

        # --- estimasi pose board via ChArUco (saat diminta, skip di manual mode) ---
        if collecting and not a.manual_pose:
            cc, ci, _, _ = detect_charuco(det, gray)
            if ci is not None and len(ci) >= 6:
                obj, img = board.matchImagePoints(cc, ci)
                good, rv, tv = cv2.solvePnP(obj, img, K, dist,
                                            flags=cv2.SOLVEPNP_ITERATIVE)
                if good:
                    samples.append((rv.reshape(3), tv.reshape(3)))
            if len(samples) >= n_collect:
                rvec = np.mean([s[0] for s in samples], axis=0).reshape(3, 1)
                tvec = np.mean([s[1] for s in samples], axis=0).reshape(3, 1)
                np.savez(a.pose, rvec=rvec, tvec=tvec)
                collecting = False
                print("Pose board tersimpan ->", a.pose)

        # --- deteksi oranye ---
        hsv = cv2.cvtColor(cv2.GaussianBlur(frame, (5, 5), 0), cv2.COLOR_BGR2HSV)
        mask = make_mask(hsv,
                         cv2.getTrackbarPos("H min", win),
                         cv2.getTrackbarPos("H max", win),
                         cv2.getTrackbarPos("S min", win),
                         cv2.getTrackbarPos("V min", win))
        blob, area = find_blob(mask)

        lines = []
        pos = None
        if rvec is None:
            if a.manual_pose:
                lines.append("Atur slider Origin/Depth, tekan 'b' untuk simpan")
            else:
                lines.append("Tekan 'b' dengan board terlihat untuk set pose")
        else:
            R, _ = cv2.Rodrigues(rvec)
            t = tvec.reshape(3)
            axis_len = a.square_mm * 2 if not a.manual_pose else 50.0
            cv2.drawFrameAxes(frame, K, dist, rvec, tvec, axis_len)
            if a.manual_pose:
                lines.append("Pose: X=%.0f Y=%.0f Depth=%.0f mm" %
                             (tvec[0, 0], tvec[1, 0], tvec[2, 0]))
            if blob:
                pos = pixel_to_board(blob, K, dist, R, t, a.plane_z)
                lines.append("Oranye: X=%.2f  Y=%.2f mm" % pos)
            else:
                lines.append("Oranye tidak terdeteksi")
            if recording and pos:
                writer.writerow([round(t_frame, 4), round(t_frame - t0, 4),
                                 round(pos[0], 3), round(pos[1], 3)])

        # riwayat untuk median target (reset kalau blob hilang)
        if pos:
            hist.append(pos)
        else:
            hist.clear()

        if blob:
            cv2.drawMarker(frame, (int(blob[0]), int(blob[1])), (255, 0, 255),
                           cv2.MARKER_CROSS, 24, 2)
            cv2.circle(frame, (int(blob[0]), int(blob[1])), 14, (0, 165, 255), 2)
        for i, ln in enumerate(lines):
            cv2.putText(frame, ln, (10, 30 + 28 * i), cv2.FONT_HERSHEY_SIMPLEX,
                        0.8, (0, 255, 255), 2)
        if time.time() < msg_until:
            cv2.putText(frame, msg, (10, 30 + 28 * len(lines)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        if collecting:
            cv2.putText(frame, "Mengambil pose %d/%d" % (len(samples), n_collect),
                        (10, frame.shape[0] - 20), cv2.FONT_HERSHEY_SIMPLEX,
                        0.8, (0, 165, 255), 2)
        if recording:
            cv2.circle(frame, (frame.shape[1] - 25, 25), 10, (0, 0, 255), -1)

        cv2.imshow(win, frame)
        cv2.imshow("mask (oranye = putih)", cv2.resize(mask, None, fx=0.4, fy=0.4))

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        elif key == ord("b"):
            if a.manual_pose:
                # Simpan pose manual langsung
                np.savez(a.pose, rvec=rvec, tvec=tvec)
                msg = "Pose manual tersimpan (depth=%.0f mm)" % tvec[2, 0]
                msg_until = time.time() + 2.0
                print("Pose manual tersimpan ->", a.pose,
                      "| origin_x=%.0f origin_y=%.0f depth=%.0f mm" %
                      (tvec[0, 0], tvec[1, 0], tvec[2, 0]))
            else:
                samples.clear()
                collecting = True
        elif key == ord("g"):
            if rvec is None:
                print("Set pose board dulu (tekan 'b').")
            elif len(hist) < HIST_MIN:
                print("Objek belum stabil terdeteksi (%d/%d frame)." %
                      (len(hist), HIST_MIN))
            else:
                x = float(np.median([p_[0] for p_ in hist]))
                y = float(np.median([p_[1] for p_ in hist]))
                with open(a.target, "w") as f:
                    json.dump({"x_mm": x, "y_mm": y,
                               "plane_z_mm": a.plane_z}, f, indent=2)
                msg = "Target: X=%.1f Y=%.1f mm" % (x, y)
                msg_until = time.time() + 2.0
                print("Target tersimpan ->", a.target, "| X=%.1f Y=%.1f mm "
                      "(plane_z=%.1f)" % (x, y, a.plane_z))
                # --- auto-run: jalankan planning + kirim ke robot ---
                if a.auto_run:
                    target_abs = os.path.abspath(a.target)
                    script = os.path.join(os.path.abspath(a.control_dir), "run_experiment1.py")
                    cmd = [sys.executable, script,
                           "--port", a.port,
                           "--goal-tracker", target_abs,
                           "--z-offset", str(a.z_offset),
                           "--yes",
                           "--scale-x", str(a.scale_x),
                           "--scale-y", str(a.scale_y),
                           "--offset-x", str(a.offset_x),
                           "--offset-y", str(a.offset_y)]
                    if a.ar_pitch is not None:
                        cmd += ["--pitch", str(a.ar_pitch)]
                    if a.safe_z is not None:
                        cmd += ["--safe-z", str(a.safe_z)]
                    print(">>> Auto-run:", " ".join(cmd))
                    subprocess.Popen(cmd, cwd=os.path.abspath(a.control_dir))
                    msg = "Auto-run dimulai!"
                    msg_until = time.time() + 3.0
        elif key == ord("r"):
            recording = not recording
            if recording:
                fh = open("orange_log.csv", "w", newline="")
                writer = csv.writer(fh)
                writer.writerow(["t_epoch", "t_s", "x_mm", "y_mm"])
                t0 = time.time()
                print("Rekam -> orange_log.csv")
            elif fh:
                fh.close()
                print("Rekam berhenti")

    if fh and not fh.closed:
        fh.close()
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()