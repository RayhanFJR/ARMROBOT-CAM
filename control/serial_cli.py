"""
Serial CLI — komunikasi langsung PC <-> ESP32 untuk debug.

Pemakaian:
  python serial_cli.py COM15
  python serial_cli.py COM15 --baud 115200

Ketik perintah, tekan Enter untuk kirim. Respon ESP muncul real-time.
Ketik 'exit' atau Ctrl+C untuk keluar.
"""
import argparse
import sys
import threading
import serial


def reader(ser):
    """Thread: baca terus dari serial, cetak ke layar."""
    while True:
        try:
            line = ser.readline().decode(errors="ignore").strip()
            if line:
                print(f"  ESP> {line}")
        except serial.SerialException:
            break
        except Exception:
            break


def main():
    ap = argparse.ArgumentParser(description="Serial CLI untuk debug ESP32")
    ap.add_argument("port", help="port serial, misal COM15")
    ap.add_argument("--baud", type=int, default=115200)
    a = ap.parse_args()

    print(f"Menghubungkan ke {a.port} @ {a.baud}...")
    ser = serial.Serial(a.port, a.baud, timeout=0.5)
    import time; time.sleep(2.0)   # tunggu ESP32 reset
    ser.reset_input_buffer()
    print(f"Terhubung! Ketik perintah, Enter untuk kirim. 'exit' untuk keluar.\n")

    # Thread pembaca (background)
    t = threading.Thread(target=reader, args=(ser,), daemon=True)
    t.start()

    try:
        while True:
            cmd = input(">> ")
            if cmd.strip().lower() == "exit":
                break
            ser.write((cmd + "\n").encode())
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        ser.close()
        print("\nKoneksi ditutup.")


if __name__ == "__main__":
    main()
