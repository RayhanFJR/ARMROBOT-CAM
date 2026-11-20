"""
esp_link.py
Jembatan serial Python <-> ESP32 (firmware robot_arm_traj.ino).

Konvensi sudut:
  Python/URDF   : q_urdf (derajat)
  Dikirim ke ESP: q_esp = SIGN * q_urdf + OFFSET   (derajat; q_esp = 0 = servoHome di firmware)
Atur SIGN / OFFSET di sini untuk mencocokkan nol dan arah putar URDF dengan servo,
tanpa perlu flash ulang firmware.
"""
import time

import numpy as np
import serial


class ArmLink:
    def __init__(self, port, baud=115200, sign=(1, 1, 1, 1, 1), offset=(0, 0, 0, 0, 0), timeout=2.0):
        self.sign = np.asarray(sign, float)
        self.offset = np.asarray(offset, float)
        self.s = serial.Serial(port, baud, timeout=timeout)
        time.sleep(2.0)                    # ESP32 reset saat port dibuka
        self.s.reset_input_buffer()

    # ---------- konversi ----------
    def to_esp(self, q_urdf_deg):
        return np.asarray(q_urdf_deg, float) * self.sign + self.offset

    def from_esp(self, q_esp_deg):
        return (np.asarray(q_esp_deg, float) - self.offset) / self.sign

    # ---------- serial dasar ----------
    def _tx(self, line):
        self.s.write((line + "\n").encode())

    def _wait(self, prefixes, timeout=3.0, echo=False):
        """Baca baris sampai ada yang diawali salah satu prefix. 'ERR' selalu menghentikan."""
        t0 = time.time()
        while time.time() - t0 < timeout:
            raw = self.s.readline().decode(errors="ignore").strip()
            if not raw:
                continue
            if echo:
                print("  ESP>", raw)
            if raw.startswith("ERR"):
                raise RuntimeError(raw)
            if any(raw.startswith(p) for p in prefixes):
                return raw
        raise TimeoutError(f"Tidak ada balasan {prefixes} dalam {timeout}s")

    # ---------- perintah ----------
    def ping(self):
        self._tx("PING")
        self._wait(["PONG"])
        return True

    def get_q(self):
        """Posisi terakhir yang DIPERINTAHKAN (bukan feedback sensor). Return (q_urdf_deg[5], grip%)."""
        self._tx("GETQ")
        r = self._wait(["Q "]).split()
        v = [float(x) for x in r[1:7]]
        return self.from_esp(v[:5]), v[5]

    def upload(self, q_urdf_deg, grip, dt_s, verbose=True):
        """q_urdf_deg: (N,5) derajat. grip: skalar atau (N,) persen. dt_s: selang sampel (detik)."""
        Q = self.to_esp(q_urdf_deg)
        N = len(Q)
        g = np.full(N, float(grip)) if np.isscalar(grip) else np.asarray(grip, float)
        self._tx(f"TRJ BEGIN {N} {int(round(dt_s * 1000))}")
        self._wait(["OK BEGIN"])
        for i in range(N):
            row = " ".join(f"{v:.2f}" for v in Q[i]) + f" {g[i]:.2f}"
            self._tx("TRJ P " + row)
            self._wait(["K"])
            if verbose and (i % 100 == 99 or i == N - 1):
                print(f"  upload {i + 1}/{N}")
        self._tx("TRJ END")
        self._wait(["OK END"])

    def go(self):
        self._tx("TRJ GO")

    def wait_done(self, timeout=60.0, echo=True):
        """Tunggu 'DONE' dari ESP. Return True kalau selesai."""
        self._wait(["DONE"], timeout=timeout, echo=echo)
        return True

    def stop(self):
        self._tx("TRJ STOP")

    def send_move_cmd(self, cmd, timeout=15.0, echo=True):
        """Kirim perintah gerak (mis. 'm' untuk home, 'p' untuk place) dan tunggu selesai."""
        self._tx(cmd)
        self._wait(["Gerakan selesai"], timeout=timeout, echo=echo)
        return True

    def close(self):
        self.s.close()
