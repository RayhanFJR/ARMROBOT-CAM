"""
Tahap 2: tracking LED merah/hijau -> koordinat X,Y (mm) di bidang board ChArUco.

  1. python calibrate_camera.py --source 3     (sekali saja -> calib.npz)
  2. Letakkan board RATA di bidang kerja end effector, kamera diam.
  3. python led_tracker_charuco.py --source 3
  4. Tekan 'b' (board harus terlihat) -> pose board disimpan ke pose.npz.
     Setelah itu board boleh diambil, ASAL kamera TIDAK bergeser.

Tombol: b = set pose board (rata-rata 30 frame), r = rekam CSV, q = keluar

Koordinat (X,Y) dalam mm pada sistem koordinat board (sumbu digambar di layar:
merah = X, hijau = Y, biru = Z). --plane-z adalah tinggi bidang LED terhadap
bidang board dalam mm (0 jika LED sebidang dengan board). Z board mengarah
masuk ke board, jadi LED yang lebih dekat ke kamera memakai nilai NEGATIF.
"""
import argparse
import csv
import math
import time

import cv2
import numpy as np

from charuco_common import (add_board_args, make_detector, open_cap,
                            detect_charuco)

RED_1 = ((0, 120, 120), (10, 255, 255))
RED_2 = ((170, 120, 120), (179, 255, 255))
GREEN = ((40, 100, 100), (85, 255, 255))
MIN_AREA = 8


def make_masks(hsv, s_min, v_min):
    def r(lo, hi):
        return (np.array((lo[0], max(lo[1], s_min), max(lo[2], v_min))),
                np.array(hi))
    red = cv2.inRange(hsv, *r(*RED_1)) | cv2.inRange(hsv, *r(*RED_2))
    grn = cv2.inRange(hsv, *r(*GREEN))
    k = np.ones((3, 3), np.uint8)
    return (cv2.morphologyEx(red, cv2.MORPH_OPEN, k),
            cv2.morphologyEx(grn, cv2.MORPH_OPEN, k))


def find_led(mask):
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts:
        return None
    c = max(cnts, key=cv2.contourArea)
    if cv2.contourArea(c) < MIN_AREA:
        return None
    m = cv2.moments(c)
    return (m["m10"] / m["m00"], m["m01"] / m["m00"]) if m["m00"] else None


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
    ap.add_argument("--plane-z", type=float, default=0.0,
                    help="tinggi bidang LED terhadap board (mm)")
    a = ap.parse_args()

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

    samples = []          # sampel pose saat 'b' ditekan
    n_collect = 30
    collecting = False
    recording, fh, writer, t0 = False, None, None, time.time()

    win = "LED tracker (ChArUco)"
    cv2.namedWindow(win)
    cv2.createTrackbar("S min", win, 120, 255, lambda v: None)
    cv2.createTrackbar("V min", win, 120, 255, lambda v: None)

    while True:
        ok, frame = cap.read()
        if not ok:
            break
        t_frame = time.time()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # --- estimasi pose board (saat diminta) ---
        if collecting:
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

        # --- deteksi LED ---
        hsv = cv2.cvtColor(cv2.GaussianBlur(frame, (5, 5), 0), cv2.COLOR_BGR2HSV)
        red_m, grn_m = make_masks(hsv, cv2.getTrackbarPos("S min", win),
                                  cv2.getTrackbarPos("V min", win))
        red, grn = find_led(red_m), find_led(grn_m)

        lines = []
        rp = gp = ee = ang = None
        if rvec is None:
            lines.append("Tekan 'b' dengan board terlihat untuk set pose")
        else:
            R, _ = cv2.Rodrigues(rvec)
            t = tvec.reshape(3)
            cv2.drawFrameAxes(frame, K, dist, rvec, tvec, a.square_mm * 2)
            if red:
                rp = pixel_to_board(red, K, dist, R, t, a.plane_z)
                lines.append("Merah : X=%.2f  Y=%.2f mm" % rp)
            if grn:
                gp = pixel_to_board(grn, K, dist, R, t, a.plane_z)
                lines.append("Hijau : X=%.2f  Y=%.2f mm" % gp)
            if rp and gp:
                ee = ((rp[0] + gp[0]) / 2, (rp[1] + gp[1]) / 2)
                ang = math.degrees(math.atan2(gp[1] - rp[1], gp[0] - rp[0]))
                lines.insert(0, "EE: X=%.2f  Y=%.2f mm | sudut %.1f deg"
                             % (ee[0], ee[1], ang))
            if recording and ee:
                writer.writerow([round(t_frame, 4), round(t_frame - t0, 4),
                                 *rp, *gp, *ee, ang])

        if red:
            cv2.circle(frame, (int(red[0]), int(red[1])), 10, (0, 0, 255), 2)
        if grn:
            cv2.circle(frame, (int(grn[0]), int(grn[1])), 10, (0, 255, 0), 2)
        for i, ln in enumerate(lines):
            cv2.putText(frame, ln, (10, 30 + 28 * i), cv2.FONT_HERSHEY_SIMPLEX,
                        0.8, (0, 255, 255), 2)
        if collecting:
            cv2.putText(frame, "Mengambil pose %d/%d" % (len(samples), n_collect),
                        (10, frame.shape[0] - 20), cv2.FONT_HERSHEY_SIMPLEX,
                        0.8, (0, 165, 255), 2)
        if recording:
            cv2.circle(frame, (frame.shape[1] - 25, 25), 10, (0, 0, 255), -1)

        cv2.imshow(win, frame)
        cv2.imshow("mask (kiri=merah, kanan=hijau)", cv2.resize(
            np.hstack([red_m, grn_m]), None, fx=0.3, fy=0.3))

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        elif key == ord("b"):
            samples.clear()
            collecting = True
        elif key == ord("r"):
            recording = not recording
            if recording:
                fh = open("led_log.csv", "w", newline="")
                writer = csv.writer(fh)
                writer.writerow(["t_epoch", "t_s", "xr", "yr", "xg", "yg",
                                 "x_ee", "y_ee", "angle_deg"])
                t0 = time.time()
            elif fh:
                fh.close()

    if fh and not fh.closed:
        fh.close()
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()