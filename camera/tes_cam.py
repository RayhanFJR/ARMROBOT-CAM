"""
Deteksi LED merah & hijau dari GoPro -> koordinat kartesian end effector.

Pemakaian:
  python led_tracker.py --source 0 --led-dist-mm 50
  python led_tracker.py --source "udp://@:8554" --led-dist-mm 50

Kontrol (di jendela video):
  Klik kiri : set titik origin (0,0) kartesian
  c         : kalibrasi skala (butuh --led-dist-mm, kedua LED harus terdeteksi)
  r         : mulai/stop rekam ke CSV
  q / ESC   : keluar

Keluaran per frame:
  - posisi merah (xr, yr), hijau (xg, yg)
  - posisi end effector = titik tengah kedua LED
  - orientasi (derajat) = arah vektor merah -> hijau
Sumbu: x ke kanan, y ke atas (sumbu y gambar dibalik). Satuan mm jika sudah
dikalibrasi, selain itu piksel.

Asumsi: gerakan end effector berada di satu bidang yang tegak lurus sumbu
optik kamera (2D). Untuk 3D butuh kalibrasi kamera / stereo.
"""
import argparse
import csv
import math
import time

import cv2
import numpy as np

# Rentang HSV (H: 0-179). Sesuaikan dengan pencahayaan lewat trackbar.
RED_1 = ((0, 120, 120), (10, 255, 255))
RED_2 = ((170, 120, 120), (179, 255, 255))
GREEN = ((40, 100, 100), (85, 255, 255))
MIN_AREA = 8  # piksel, buang noise yang lebih kecil dari ini


def make_masks(hsv, s_min, v_min):
    def rng(lo, hi):
        return (lo[0], max(lo[1], s_min), max(lo[2], v_min)), hi

    r1 = cv2.inRange(hsv, *map(np.array, rng(*RED_1)))
    r2 = cv2.inRange(hsv, *map(np.array, rng(*RED_2)))
    g = cv2.inRange(hsv, *map(np.array, rng(*GREEN)))
    red = cv2.bitwise_or(r1, r2)
    k = np.ones((3, 3), np.uint8)
    red = cv2.morphologyEx(red, cv2.MORPH_OPEN, k)
    g = cv2.morphologyEx(g, cv2.MORPH_OPEN, k)
    return red, g


def find_led(mask):
    """Centroid kontur terbesar, atau None."""
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts:
        return None
    c = max(cnts, key=cv2.contourArea)
    if cv2.contourArea(c) < MIN_AREA:
        return None
    m = cv2.moments(c)
    if m["m00"] == 0:
        return None
    return (m["m10"] / m["m00"], m["m01"] / m["m00"])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--source", default="0",
                    help="index kamera (GoPro mode webcam) atau URL stream")
    ap.add_argument("--led-dist-mm", type=float, default=None,
                    help="jarak asli antar LED (mm) untuk kalibrasi skala")
    ap.add_argument("--scale", type=float, default=None,
                    help="skala langsung (mm per piksel), alternatif kalibrasi")
    ap.add_argument("--width", type=int, default=1280)
    args = ap.parse_args()

    src = int(args.source) if args.source.isdigit() else args.source
    cap = cv2.VideoCapture(src)
    if not cap.isOpened():
        raise SystemExit("Gagal membuka sumber video: %s" % args.source)
    if isinstance(src, int):
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(args.width * 9 / 16))

    origin = None          # (px, py) di gambar
    scale = args.scale     # mm per piksel (None = tampilkan piksel)
    recording = False
    writer = None
    fh = None
    t0 = time.time()

    win = "LED tracker"
    cv2.namedWindow(win)
    cv2.createTrackbar("S min", win, 120, 255, lambda v: None)
    cv2.createTrackbar("V min", win, 120, 255, lambda v: None)

    def on_mouse(event, x, y, flags, param):
        nonlocal origin
        if event == cv2.EVENT_LBUTTONDOWN:
            origin = (x, y)

    cv2.setMouseCallback(win, on_mouse)

    while True:
        ok, frame = cap.read()
        if not ok:
            print("Frame tidak terbaca, berhenti.")
            break

        blur = cv2.GaussianBlur(frame, (5, 5), 0)
        hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
        s_min = cv2.getTrackbarPos("S min", win)
        v_min = cv2.getTrackbarPos("V min", win)
        red_mask, green_mask = make_masks(hsv, s_min, v_min)

        red = find_led(red_mask)
        green = find_led(green_mask)

        if origin is None:
            origin = (frame.shape[1] / 2, frame.shape[0] / 2)

        def to_cart(p):
            """piksel -> kartesian (x kanan, y atas), terhadap origin."""
            f = scale if scale else 1.0
            return ((p[0] - origin[0]) * f, (origin[1] - p[1]) * f)

        unit = "mm" if scale else "px"
        info = []
        ee = None
        angle = None

        if red:
            cv2.circle(frame, (int(red[0]), int(red[1])), 8, (0, 0, 255), 2)
        if green:
            cv2.circle(frame, (int(green[0]), int(green[1])), 8, (0, 255, 0), 2)

        if red and green:
            rc, gc = to_cart(red), to_cart(green)
            ee = ((rc[0] + gc[0]) / 2, (rc[1] + gc[1]) / 2)
            angle = math.degrees(math.atan2(gc[1] - rc[1], gc[0] - rc[0]))
            cv2.line(frame, (int(red[0]), int(red[1])),
                     (int(green[0]), int(green[1])), (255, 255, 0), 1)
            mid = ((red[0] + green[0]) / 2, (red[1] + green[1]) / 2)
            cv2.drawMarker(frame, (int(mid[0]), int(mid[1])), (255, 0, 255),
                           cv2.MARKER_CROSS, 18, 2)
            info.append("EE  : (%.1f, %.1f) %s" % (ee[0], ee[1], unit))
            info.append("Sudut: %.1f deg" % angle)
        if red:
            rc = to_cart(red)
            info.append("Merah : (%.1f, %.1f)" % rc)
        if green:
            gc = to_cart(green)
            info.append("Hijau : (%.1f, %.1f)" % gc)
        if not (red and green):
            info.append("LED belum lengkap terdeteksi")

        # gambar sumbu origin
        ox, oy = int(origin[0]), int(origin[1])
        cv2.arrowedLine(frame, (ox, oy), (ox + 60, oy), (255, 255, 255), 1)
        cv2.arrowedLine(frame, (ox, oy), (ox, oy - 60), (255, 255, 255), 1)
        cv2.putText(frame, "x", (ox + 64, oy + 5), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (255, 255, 255), 1)
        cv2.putText(frame, "y", (ox + 5, oy - 64), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (255, 255, 255), 1)

        for i, line in enumerate(info):
            cv2.putText(frame, line, (10, 25 + 22 * i),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        if recording and ee is not None:
            writer.writerow([round(time.time() - t0, 3),
                             *[round(v, 3) for v in to_cart(red)],
                             *[round(v, 3) for v in to_cart(green)],
                             round(ee[0], 3), round(ee[1], 3), round(angle, 3)])
        if recording:
            cv2.circle(frame, (frame.shape[1] - 20, 20), 8, (0, 0, 255), -1)

        small = cv2.resize(np.hstack([
            cv2.cvtColor(red_mask, cv2.COLOR_GRAY2BGR),
            cv2.cvtColor(green_mask, cv2.COLOR_GRAY2BGR)]), None, fx=0.3, fy=0.3)
        cv2.imshow("mask (kiri=merah, kanan=hijau)", small)

        cv2.imshow(win, frame)
        key = cv2.waitKey(1) & 0xFF
        if key in (ord("q"), 27):
            break
        elif key == ord("c"):
            if red and green and args.led_dist_mm:
                d_px = math.hypot(green[0] - red[0], green[1] - red[1])
                scale = args.led_dist_mm / d_px
                print("Skala: %.4f mm/px" % scale)
            else:
                print("Kalibrasi butuh kedua LED terdeteksi + --led-dist-mm")
        elif key == ord("r"):
            recording = not recording
            if recording:
                fh = open("led_log.csv", "w", newline="")
                writer = csv.writer(fh)
                writer.writerow(["t_s", "xr", "yr", "xg", "yg", "x_ee", "y_ee",
                                 "angle_deg"])
                t0 = time.time()
                print("Rekam -> led_log.csv")
            elif fh:
                fh.close()
                print("Rekam berhenti")

    if fh and not fh.closed:
        fh.close()
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()  