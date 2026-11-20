import argparse

import cv2


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--source", default="0")
    ap.add_argument("--image", default=None)
    ap.add_argument("--width", type=int, default=1920)
    ap.add_argument("--height", type=int, default=1080)
    a = ap.parse_args()

    if a.image:
        frame = cv2.imread(a.image)
        if frame is None:
            raise SystemExit("Gagal membaca gambar: %s" % a.image)
    else:
        src = int(a.source) if a.source.isdigit() else a.source
        cap = cv2.VideoCapture(src)
        if not cap.isOpened():
            raise SystemExit("Gagal membuka sumber video: %s" % a.source)
        if isinstance(src, int):
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, a.width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, a.height)
        frame = None
        for _ in range(30):
            ok, f = cap.read()
            if ok:
                frame = f
        cap.release()
        if frame is None:
            raise SystemExit("Tidak ada frame yang terbaca")

    cv2.imwrite("frame.png", frame)
    print("Frame disimpan ke frame.png (%dx%d)" % (frame.shape[1], frame.shape[0]))

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    varian = {"normal": gray,
              "cermin": cv2.flip(gray, 1),
              "invert": cv2.bitwise_not(gray)}

    p = cv2.aruco.DetectorParameters()
    p.adaptiveThreshWinSizeMax = 63
    p.minMarkerPerimeterRate = 0.01
    p.polygonalApproxAccuracyRate = 0.05

    hasil = []
    for name in dir(cv2.aruco):
        if not name.startswith("DICT_"):
            continue
        d = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, name))
        det = cv2.aruco.ArucoDetector(d, p)
        for vname, img in varian.items():
            _, ids, _ = det.detectMarkers(img)
            n = 0 if ids is None else len(ids)
            idl = [] if ids is None else sorted(ids.flatten().tolist())
            hasil.append((n, name, vname, idl))

    hasil.sort(key=lambda r: -r[0])
    print("5 kandidat teratas:")
    for n, name, vname, idl in hasil[:5]:
        print("  %-20s %-7s marker: %2d  ID: %s" % (name, vname, n, idl))


if __name__ == "__main__":
    main()