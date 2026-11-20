"""
Tahap 1: kalibrasi intrinsik kamera (K + distorsi) dengan board ChArUco.

  python calibrate_camera.py --source 3

Default board: 7x5 kotak, DICT_6X6_50, kotak 49 mm, marker 43 mm
(ubah lewat argumen jika board berbeda).

Tombol:
  SPASI : ambil frame (butuh >= 8 sudut board terdeteksi)
  k     : hitung kalibrasi (butuh >= 15 frame) dan simpan ke calib.npz
  q     : keluar

Tips: gerakkan/miringkan board di berbagai posisi & sudut, sebarkan ke seluruh
area frame (termasuk pinggir). Ambil 20-40 frame. Jangan ubah mode lensa
GoPro (pakai Linear, HyperSmooth/stabilisasi OFF) setelah kalibrasi.
"""
import argparse

import cv2
import numpy as np

from charuco_common import (add_board_args, make_detector, open_cap,
                            detect_charuco)


def main():
    ap = argparse.ArgumentParser()
    add_board_args(ap)
    ap.add_argument("--out", default="calib.npz")
    a = ap.parse_args()

    board, det = make_detector(a)
    cap = open_cap(a)
    objs, imgs = [], []
    size = None

    while True:
        ok, frame = cap.read()
        if not ok:
            break
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        size = (gray.shape[1], gray.shape[0])
        cc, ci, mc, mi = detect_charuco(det, gray)
        n = 0 if ci is None else len(ci)

        vis = frame.copy()
        if mc is not None and len(mc):
            cv2.aruco.drawDetectedMarkers(vis, mc, mi)
        if n:
            for p in cc.reshape(-1, 2):
                cv2.circle(vis, (int(p[0]), int(p[1])), 6, (0, 255, 0), -1)
        cv2.putText(vis, "sudut: %d | frame tersimpan: %d" % (n, len(objs)),
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        cv2.imshow("kalibrasi", vis)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        elif key == ord(" ") and n >= 8:
            o, i = board.matchImagePoints(cc, ci)
            objs.append(o)
            imgs.append(i)
            print("frame %d disimpan (%d sudut)" % (len(objs), n))
        elif key == ord("k"):
            if len(objs) < 15:
                print("Butuh minimal 15 frame, baru %d" % len(objs))
                continue
            rms, K, dist, rvecs, tvecs = cv2.calibrateCamera(
                objs, imgs, size, None, None)
            print("RMS reprojection error: %.3f px" % rms)
            if rms > 1.0:
                print("PERINGATAN: error > 1 px. Ambil lebih banyak frame "
                      "dengan sudut beragam, atau cek ukuran board.")
            np.savez(a.out, K=K, dist=dist, size=np.array(size), rms=rms)
            print("Tersimpan ->", a.out)

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()