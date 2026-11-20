"""
Tracking benda ORANYE -> koordinat X,Y (mm) di bidang board ChArUco.

  1. python calibrate_camera.py --source 3      (sekali saja -> calib.npz)
  2. Letakkan board RATA di bidang kerja end effector, kamera diam.
  3. python orange_tracker_charuco.py --source 3
  4. Tekan 'b' (board harus terlihat) -> pose board disimpan ke pose.npz.
     Setelah itu board boleh diambil, ASAL kamera TIDAK bergeser.

Tombol: b = set pose board (rata-rata 30 frame), r = rekam CSV, q = keluar

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
import time

import cv2
import numpy as np

from charuco_common import (add_board_args, make_detector, open_cap,
                            detect_charuco)

MIN_AREA = 30  # piksel; blob oranye lebih kecil dari ini diabaikan


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
    ap.add_argument("--plane-z", type=float, default=0.0,
                    help="tinggi titik oranye terhadap board (mm)")
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

    samples = []
    n_collect = 30
    collecting = False
    recording, fh, writer, t0 = False, None, None, time.time()

    win = "Orange tracker (ChArUco)"
    cv2.namedWindow(win)
    cv2.createTrackbar("H min", win, 5, 179, lambda v: None)
    cv2.createTrackbar("H max", win, 25, 179, lambda v: None)
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
            lines.append("Tekan 'b' dengan board terlihat untuk set pose")
        else:
            R, _ = cv2.Rodrigues(rvec)
            t = tvec.reshape(3)
            cv2.drawFrameAxes(frame, K, dist, rvec, tvec, a.square_mm * 2)
            if blob:
                pos = pixel_to_board(blob, K, dist, R, t, a.plane_z)
                lines.append("Oranye: X=%.2f  Y=%.2f mm" % pos)
            else:
                lines.append("Oranye tidak terdeteksi")
            if recording and pos:
                writer.writerow([round(t_frame, 4), round(t_frame - t0, 4),
                                 round(pos[0], 3), round(pos[1], 3)])

        if blob:
            cv2.drawMarker(frame, (int(blob[0]), int(blob[1])), (255, 0, 255),
                           cv2.MARKER_CROSS, 24, 2)
            cv2.circle(frame, (int(blob[0]), int(blob[1])), 14, (0, 165, 255), 2)
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
        cv2.imshow("mask (oranye = putih)", cv2.resize(mask, None, fx=0.4, fy=0.4))

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        elif key == ord("b"):
            samples.clear()
            collecting = True
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