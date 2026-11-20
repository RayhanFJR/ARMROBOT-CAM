"""Fungsi bersama untuk kalibrasi & tracking berbasis ChArUco."""
import cv2
import numpy as np


def add_board_args(ap):
    # HARUS sama persis dengan board yang kamu cetak
    ap.add_argument("--squares-x", type=int, default=7)
    ap.add_argument("--squares-y", type=int, default=5)
    ap.add_argument("--square-mm", type=float, default=49.0,
                    help="sisi kotak hitam/putih (mm), ukur dari hasil cetak")
    ap.add_argument("--marker-mm", type=float, default=43.0,
                    help="sisi marker ArUco (mm), ukur dari hasil cetak")
    ap.add_argument("--dict", default="DICT_6X6_50")
    ap.add_argument("--source", default="0")
    # resolusi kalibrasi & tracking HARUS sama
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=720)


def make_detector(a):
    d = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, a.dict))
    board = cv2.aruco.CharucoBoard((a.squares_x, a.squares_y),
                                   a.square_mm, a.marker_mm, d)
    return board, cv2.aruco.CharucoDetector(board)


def detect_charuco(det, gray):
    """detectBoard dengan bentuk array dinormalkan (aman untuk OpenCV 5)."""
    cc, ci, mc, mi = det.detectBoard(gray)
    if cc is None or ci is None or len(ci) == 0:
        return None, None, mc, mi
    cc = np.asarray(cc, np.float32).reshape(-1, 1, 2)
    ci = np.asarray(ci, np.int32).reshape(-1, 1)
    if len(cc) != len(ci):
        return None, None, mc, mi
    return cc, ci, mc, mi


def open_cap(a):
    # Sama persis dengan cara led_tracker.py membuka kamera (terbukti jalan).
    src = int(a.source) if a.source.isdigit() else a.source
    cap = cv2.VideoCapture(src)
    if not cap.isOpened():
        raise SystemExit("Gagal membuka sumber video: %s" % a.source)
    if isinstance(src, int):
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, a.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, a.height)
    return cap