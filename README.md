# 🦾 Vision-Guided Robot Arm Pick-and-Place System

Sistem otomatisasi lengan robot 5/6-DOF berbasis computer vision (ChArUco calibration & color tracking) dan kendali gerak terencana (Quintic Polynomial Trajectory & Inverse Kinematics) yang dikomunikasikan secara real-time ke mikrokontroler ESP32.

---

## 📋 Daftar Isi
- [Fitur Utama](#-fitur-utama)
- [Struktur Direktori](#-struktur-direktori)
- [Kebutuhan Perangkat](#-kebutuhan-perangkat)
  - [Hardware](#hardware)
  - [Software](#software)
- [Instalasi](#-instalasi)
- [Alur Kerja Sistem (Pipeline)](#-alur-kerja-sistem-pipeline)
- [Panduan Penggunaan](#-panduan-penggunaan)
  - [1. Kalibrasi Kamera](#1-kalibrasi-kamera)
  - [2. Tracking Target & Auto-Run Robot](#2-tracking-target--auto-run-robot)
  - [3. Debug Serial Langsung (CLI)](#3-debug-serial-langsung-cli)
- [Parameter & Konfigurasi Penting](#-parameter--konfigurasi-penting)
  - [Argumen Command-Line](#argumen-command-line)
  - [Penyesuaian di `run_experiment1.py`](#penyesuaian-di-run_experiment1py)
- [Troubleshooting](#-troubleshooting)

---

## ✨ Fitur Utama

- **Kalibrasi Bidang Kerja Otomatis & Manual**:
  - Penentuan pose baseplate 3D menggunakan papan ChArUco (OpenCV SolvePnP).
  - Mode manual (`--manual-pose`) menggunakan slider GUI untuk mengatur depth & origin tanpa papan fisik.
- **Deteksi Objek Real-Time**:
  - Color blob detector (HSV) dengan slider ambang batas interaktif.
  - Estimasi posisi Cartesian 3D dunia nyata ($X, Y$ dalam mm) via *ray-plane intersection*.
- **Perencanaan Gerak Aman (2-Phase Motion Planning)**:
  - Trajektori quintic polynomial 5-order yang mulus tanpa akselerasi mendadak.
  - Lintasan bertahap dengan ketinggian aman (`--safe-z`): *Swing at Safe Height* $\rightarrow$ *Descend to Target* untuk menghindari obstacle.
- **Siklus Penuh Pick-and-Place**:
  - *Start* $\rightarrow$ *Hover* $\rightarrow$ *Descend* $\rightarrow$ *Grip Close* $\rightarrow$ *Ascend Safe Z* $\rightarrow$ *Hover Home* $\rightarrow$ *Home* $\rightarrow$ *Place* $\rightarrow$ *Home*.
- **Kompensasi Offset & Skala Empiris**:
  - Parameter `--scale-x`, `--scale-y`, `--offset-x`, dan `--offset-y` untuk menyamakan skala kamera dengan jangkauan fisik robot secara presisi.
- **Protokol Serial Efisien (ESP32)**:
  - Buffer streaming titik trajektori (`TRJ BEGIN`, `TRJ P`, `TRJ GO`).
  - Perintah gerak individual servo (`Q <servo> <deg>`) dan macro posisi (`m` = Home, `p` = Place).

---

## 📂 Struktur Direktori

```text
Hanif/
├── camera/
│   ├── calibrate_camera.py       # Skrip kalibrasi matriks intrinsik kamera -> calib.npz
│   ├── charuco_common.py         # Helper pembuatan board & deteksi ChArUco
│   ├── orange_tracker_charuco1.py # Skrip tracking objek oranye + pipeline auto-run
│   ├── calib.npz                 # Matriks kamera K dan koefisien distorsi
│   ├── pose.npz                  # Vektor rotasi & translasi bidang board (rvec, tvec)
│   └── target.json               # Koordinat target terakhir yang disimpan
├── control/
│   ├── arm.urdf                  # Model kinematika 3D robot dari SolidWorks
│   ├── arm_planner.py            # Engine IK, FK, RRT-Connect, dan Traj Quintic
│   ├── esp_link.py               # Handler komunikasi serial Python <-> ESP32
│   ├── run_experiment1.py        # Controller utama eksekusi trajektori & pick-place
│   ├── serial_cli.py             # Tool CLI interaktif untuk manual test perintah ESP32
│   └── robot_arm_traj/
│       └── robot_arm_traj.ino    # Firmware C++ ESP32 + driver PCA9685
├── requirements.txt              # Daftar dependensi Python
└── README.md                     # Dokumentasi proyek
```

---

## 🛠 Kebutuhan Perangkat

### Hardware
1. **Lengan Robot 5/6-DOF** berbasis servo (misal: MG996R / DS3218 / sejenis).
2. **PCA9685 16-Channel PWM Driver** (komunikasi I2C ke ESP32).
3. **ESP32 DevKit V1** (koneksi USB serial ke PC).
4. **Power Supply Eksternal** (5V / 6V DC minimum 5A-10A untuk servo).
5. **Kamera USB / Webcam** (posisi *top-down* menghadap bidang kerja).
6. **ChArUco Board Printout** (papan kalibrasi papan catur ArUco).

### Software
- **OS**: Windows / Linux / macOS
- **Python**: versi `3.10`
- **Arduino IDE** (dengan core board ESP32 & library `Adafruit_PWMServoDriver`)

---

## 📦 Instalasi

1. **Clone atau buka repositori:**
   ```bash
   cd c:\Users\LENOVO\Downloads\Hanif
   ```

2. **Buat dan aktifkan Virtual Environment (venv):**
   ```bash
   # Windows PowerShell
   python -m venv venv
   .\venv\Scripts\Activate.ps1
   ```

3. **Install Dependensi:**
   ```bash
   pip install -r requirements.txt
   ```

4. **Upload Firmware ke ESP32:**
   - Buka `control/robot_arm_traj/robot_arm_traj.ino` di Arduino IDE.
   - Pastikan konfigurasi pin I2C (`SDA_PIN`, `SCL_PIN`) dan mapping channel servo sesuai hardware.
   - Upload ke board ESP32 kamu.

---

## 🔄 Alur Kerja Sistem (Pipeline)

```text
[Kamera USB]
     │
     ▼
[Deteksi Oranye & ChArUco] ──(tekan 'g')──► [target.json (x, y mm)]
                                                    │
                                                    ▼
                                           [run_experiment1.py]
                                           ├── Transformasi Board -> Base Robot
                                           ├── Inverse Kinematics (IK)
                                           └── Trajectory Quintic Polynomial
                                                    │
                                              (Serial USB)
                                                    ▼
                                            [ESP32 Controller]
                                           ├── Interpolasi Trajektori
                                           ├── Gerak Servo via PCA9685
                                           └── Eksekusi Capit & Place
```

---

## 🚀 Panduan Penggunaan

### 1. Kalibrasi Kamera
Lakukan kalibrasi sekali saja untuk mendapatkan parameter intrinsik kamera (`calib.npz`):
```bash
cd camera
python calibrate_camera.py --source 3
```
- Arahkan board ChArUco ke berbagai sudut frame kamera.
- Tekan `c` untuk mengambil sample, atau tunggu hingga proses kalkulasi selesai.

---

### 2. Tracking Target & Auto-Run Robot

Jalankan program tracker terintegrasi. Skrip ini langsung memicu eksekusi robot saat target dikunci:

```bash
cd camera
python orange_tracker_charuco1.py --source 3 --auto-run --port COM15 --z-offset 0.060 --safe-z 0.22
```

#### Langkah Operasi di Layar Kamera:
1. **Atur Threshold Warna Oranye**:
   - Sesuaikan slider `H min`, `H max`, `S min`, `V min` hingga objek oranye berwarna putih pada jendela *mask*.
2. **Kunci Bidang Kerja (Pose)**:
   - Letakkan papan ChArUco rata di alas meja kerja.
   - Tekan **`b`** pada keyboard $\rightarrow$ program mengumpulkan 30 frame dan menyimpan `pose.npz`.
   - Angkat papan ChArUco (kamera tidak boleh bergeser).
3. **Kirim Target & Jalankan Robot**:
   - Letakkan objek target oranye di area jangkauan.
   - Tekan **`g`** $\rightarrow$ posisi dikunci (median 15 frame) $\rightarrow$ otomatis memicu `run_experiment1.py` untuk menggerakkan robot mengambil benda!

> **Opsi Kompensasi Offset/Skala:**
> Jika robot jatuh terlalu jauh / kurang presisi di sumbu tertentu:
> ```bash
> python orange_tracker_charuco1.py --source 3 --auto-run --port COM15 --z-offset 0.060 --safe-z 0.22 --offset-y -20 --offset-x 0
> ```

---

### 3. Debug Serial Langsung (CLI)

Gunakan utilitas CLI interaktif untuk menguji respon servo dan troubleshooting mikrokontroler tanpa menjalankan GUI:

```bash
cd control
python serial_cli.py COM15
```

Contoh perintah yang didukung:
- `PING` $\rightarrow$ Mengecek komunikasi (`PONG`).
- `GETQ` $\rightarrow$ Membaca sudut joint terakhir.
- `m` $\rightarrow$ Menggerakkan lengan ke posisi **HOME**.
- `p` $\rightarrow$ Menggerakkan lengan ke posisi **PLACE**.
- `Q <servo> <derajat>` $\rightarrow$ Menggerakkan satu servo spesifik (contoh: `Q 6 -120` untuk capit, `Q 6 -200` untuk lepas).
- `exit` $\rightarrow$ Menutup koneksi.

---

## ⚙️ Parameter & Konfigurasi Penting

### Argumen Command-Line

| Parameter | Default | Keterangan |
|---|---|---|
| `--source` | `0` | Index kamera USB (misal `0`, `1`, `3`). |
| `--port` | `None` | Port Serial ESP32 (contoh: `COM15` atau `/dev/ttyUSB0`). |
| `--auto-run` | Flag | Otomatis jalankan eksekusi robot begitu tombol `g` ditekan. |
| `--z-offset` | `0.05` | Ketinggian end-effector di atas permukaan alas saat pick (meter). |
| `--safe-z` | `None` | Ketinggian melayang saat fase *swing* untuk mencegah tabrakan (meter). |
| `--scale-x`, `--scale-y` | `1.0` | Faktor pengali skala koordinat jika ada perbedaan optik. |
| `--offset-x`, `--offset-y`| `0.0` | Nilai pergeseran tetap translasi dalam satuan **milimeter (mm)**. |
| `--manual-pose` | Flag | Mode estimasi alas via slider GUI tanpa perlu papan ChArUco fisik. |
| `--depth` | `645.0` | Jarak kamera ke alas (mm) untuk mode manual pose. |

### Penyesuaian di `run_experiment1.py`

Untuk mengubah timing dan batas mekanik, buka [run_experiment1.py](file:///c:/Users/LENOVO/Downloads/Hanif/control/run_experiment1.py):

- **Batas Sudut Servo (`LIMITS_DEG`)**:
  Disesuaikan ketat dengan range PWM fisik servo pada firmware Arduino guna mencegah servo mentok/rusak.
- **Konstanta Gripper & Delay**:
  ```python
  GRIP_CLOSE_Q  = -120     # Nilai sudut/PWM saat mencapit benda
  GRIP_OPEN_Q   = -200     # Nilai sudut/PWM saat gripper terbuka
  PICK_DELAY_S  = 2.5      # Jeda waktu (detik) stabilisasi sebelum menjepit
  PICK_TIMEOUT  = 10.0     # Batas waktu tunggu servo capit selesai
  ```
- **Custom Gerakan Pra-Naik**:
  Tersedia blok komentar di Step 5b untuk menyisipkan perintah kustom (misal: memiringkan pergelangan tangan sebelum naik).

---

## ❓ Troubleshooting

1. **Error: `ERR: titik X servo Y di luar batas`**:
   - Trajektori meminta sudut di luar jangkauan fisik PWM servo.
   - Turunkan nilai `--safe-z`, atau geser objek target sedikit lebih dekat ke titik tengah jangkauan robot.
2. **Kamera melacak warna lain**:
   - Buka jendela `mask (oranye = putih)` dan atur kembali slider HSV hingga hanya bola/objek yang tampak putih solid.
3. **Robot overshoot konstan**:
   - Masukkan kompensasi offset dengan flag `--offset-y <nilai_mm>` (misal `--offset-y -20`).
4. **Gripper menutup sebelum mulai gerak**:
   - Pastikan nilai `GRIP_OPEN_Q` dan `GRIP_CLOSE_Q` tidak terbalik sesuai rakitan mekanik capit kamu.
