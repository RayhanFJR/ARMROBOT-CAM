/*
  Robot Arm 6-DOF (ESP32 + PCA9685, servo MG996R)
  Versi dengan PLAYBACK TRAJEKTORI dari Python (obstacle statis: upload sekali, jalankan).
  Dasar: sketch lama (pin, channel, kalibrasi PWM, FK/IK, perintah manual) -- semua tetap ada.

  Urutan servo: 1..5 = q1 yaw, q2 shoulder, q3 elbow, q4 wrist pitch, q5 wrist roll
                6    = gripper (0..100 %, 0 = buka)

  Perintah Serial (Newline, 115200 baud):
    b / s / m / 1-6 / + / -     seperti sebelumnya (free, lock/stop, home, manual)
    J, Q, P, FK, SPD, ?         seperti sebelumnya
  Baru (dipakai Python):
    PING                        -> PONG
    GETQ                        -> "Q q1 q2 q3 q4 q5 grip" (posisi terakhir yang DIPERINTAHKAN, deg)
    TRJ BEGIN n dt_ms           mulai upload n titik, selang dt_ms (n <= TRJ_MAX)   -> OK BEGIN
    TRJ P q1 q2 q3 q4 q5 grip   satu titik (deg, grip %)                            -> K
    TRJ END                     selesai upload                                       -> OK END n
    TRJ GO                      jalan: pindah ke titik pertama (jika jauh) lalu playback -> ... DONE
    TRJ STOP                    hentikan playback, tahan posisi                      -> OK STOP
    TRJ STAT                    status buffer/playback

  Keamanan: titik trajektori yang melewati batas PWM servo DITOLAK saat upload (tidak dijepit diam-diam).
*/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// ==========================================
// 1. KONFIGURASI ESP32 DAN PCA9685
// ==========================================
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

#define SDA_PIN 21
#define SCL_PIN 22

const int SERVO_COUNT = 6;

const int servoChannels[SERVO_COUNT] = { 0, 2, 4, 6, 8, 10 };

// Data kalibrasi PWM (dari program lama)
const int servoMin[SERVO_COUNT]  = {  75,  75,  75,  75,  75,  75 };
const int servoHome[SERVO_COUNT] = { 330, 350, 295, 325, 325, 275 };
const int servoMax[SERVO_COUNT]  = { 561, 565, 565, 565, 565, 567 };
const int placePWM[SERVO_COUNT] = { 330, 400, 295, 180, 325, 275 };

// ==========================================
// 2. PARAMETER KINEMATIKA
// ==========================================
const float LEN_A1 = 23.5;    // offset radial poros yaw -> poros shoulder (mm)
const float LEN_D1 = 23.0;    // tinggi poros yaw -> poros shoulder (mm)
const float LEN_A2 = 104.5;   // shoulder -> elbow (mm)
const float LEN_L3 = 207.54;  // elbow -> wrist (mm)
const float LEN_D5 = 60.0;    // wrist -> ujung gripper / EE (mm)

// ANGGAPAN: rentang servoMin..servoMax = 180 derajat. UKUR dan sesuaikan.
const float SERVO_RANGE_DEG = 180.0;

// Arah putar positif tiap servo (ganti -1 untuk membalik)
const int DIR[SERVO_COUNT] = { 1, 1, 1, 1, 1, 1 };

// Gripper: 0% = terbuka, 100% = tertutup. WAJIB DIKALIBRASI sebelum dipakai.
const int GRIP_OPEN_PWM  = 325;
const int GRIP_CLOSE_PWM = 450;

// ==========================================
// 3. PENGATURAN GERAKAN
// ==========================================
const unsigned long HOME_DURATION   = 1000;
const unsigned long PLACE_DURATION   = 500;
const unsigned long UPDATE_INTERVAL = 10;
const int MANUAL_STEP = 5;

float speedDps = 45.0;   // kecepatan gerak J / Q / pindah ke titik awal trajektori (derajat/detik)

int currentPWM[SERVO_COUNT] = { 325, 325, 325, 325, 325, 325 };
int startPWM[SERVO_COUNT];
int targetPWM[SERVO_COUNT];

bool moving = false;
bool freeMode = false;

int selectedServo = 0;

unsigned long moveStartTime = 0;
unsigned long moveDuration = HOME_DURATION;
unsigned long lastUpdateTime = 0;

char lineBuf[96];
uint8_t lineLen = 0;

// ==========================================
// 3b. BUFFER & STATUS TRAJEKTORI
// ==========================================
const int TRJ_MAX = 1200;                       // samakan dengan MAX_POINTS di Python
int16_t trjBuf[TRJ_MAX][SERVO_COUNT];           // derajat * 100 (gripper: persen * 100)
int  trjCount = 0;                              // titik valid siap diputar
int  trjLoaded = 0;                             // titik yang sudah diterima saat upload
int  trjExpected = 0;
unsigned long trjDtMs = 20;
bool trjLoading = false;
bool trjPlaying = false;
bool trjGoingStart = false;                     // sedang pindah ke titik pertama
unsigned long trjStartTime = 0;

// ==========================================
// 4. KONVERSI SUDUT <-> PWM
// ==========================================
float ticksPerDeg(int i) {
  return (servoMax[i] - servoMin[i]) / SERVO_RANGE_DEG;
}

// q (derajat, gripper dalam %) -> nilai PWM
int qToPWM(int i, float q) {
  if (i == 5) {
    float t = GRIP_OPEN_PWM + (GRIP_CLOSE_PWM - GRIP_OPEN_PWM) * q / 100.0f;
    return (int)(t + 0.5f);
  }
  float t = servoHome[i] + DIR[i] * q * ticksPerDeg(i);
  return (int)(t + 0.5f);
}

// nilai PWM -> q (derajat, gripper dalam %)
float pwmToQ(int i, int value) {
  if (i == 5) {
    return 100.0f * (value - GRIP_OPEN_PWM) / (float)(GRIP_CLOSE_PWM - GRIP_OPEN_PWM);
  }
  return DIR[i] * (value - servoHome[i]) / ticksPerDeg(i);
}

// ==========================================
// 5. FORWARD KINEMATICS (rumus tertutup dari tabel DH)
// ==========================================
void printFK(const int *vals, const __FlashStringHelper *label) {
  float q[SERVO_COUNT];
  for (int i = 0; i < SERVO_COUNT; i++) q[i] = pwmToQ(i, vals[i]);

  float q1 = q[0] * DEG_TO_RAD;
  float q2 = q[1] * DEG_TO_RAD;
  float q3 = q[2] * DEG_TO_RAD;
  float q4 = q[3] * DEG_TO_RAD;
  float b = q2 + q3;   // sudut forearm terhadap horizontal
  float f = b + q4;    // sudut tool terhadap horizontal

  float u = LEN_A1 - LEN_A2 * sin(q2) + LEN_L3 * cos(b) + LEN_D5 * cos(f);  // jarak radial
  float w = LEN_D1 + LEN_A2 * cos(q2) + LEN_L3 * sin(b) + LEN_D5 * sin(f);  // tinggi

  Serial.print(label);
  Serial.print(F(" q(deg)=["));
  for (int i = 0; i < 5; i++) {
    Serial.print(q[i], 1);
    Serial.print(i < 4 ? F(" ") : F("]"));
  }
  Serial.print(F(" grip=")); Serial.print(q[5], 0); Serial.println(F("%"));
  Serial.print(F("   EE: X=")); Serial.print(u * cos(q1), 1);
  Serial.print(F(" Y="));      Serial.print(u * sin(q1), 1);
  Serial.print(F(" Z="));      Serial.print(w, 1);
  Serial.print(F(" mm | pitch tool=")); Serial.print(q[1] + q[2] + q[3], 1);
  Serial.println(F(" deg"));
}

// ==========================================
// 5b. INVERSE KINEMATICS (analitik, elbow-up)
// ==========================================
float wrapDeg(float a) {
  while (a > 180.0f)  a -= 360.0f;
  while (a < -180.0f) a += 360.0f;
  return a;
}

bool solveIK(float X, float Y, float Z, float pitchDeg, float q[4]) {
  float u = sqrt(X * X + Y * Y);              // jarak radial target dari sumbu yaw
  if (u < 1.0f) return false;                 // tepat di atas sumbu yaw, q1 tak terdefinisi

  float phi = pitchDeg * DEG_TO_RAD;

  // Titik wrist relatif terhadap poros shoulder (mundur dari ujung gripper sejauh LEN_D5)
  float uw = u - LEN_A1 - LEN_D5 * cos(phi);
  float ww = Z - LEN_D1 - LEN_D5 * sin(phi);

  // Dua link planar (LEN_A2 dan LEN_L3): hukum cosinus
  float c = (uw * uw + ww * ww - LEN_A2 * LEN_A2 - LEN_L3 * LEN_L3) / (2.0f * LEN_A2 * LEN_L3);
  if (c < -1.0f || c > 1.0f) return false;    // di luar jangkauan

  float thb   = -acos(c);                     // sudut forearm relatif lengan atas (elbow-up)
  float alpha = atan2(ww, uw) - atan2(LEN_L3 * sin(thb), LEN_A2 + LEN_L3 * cos(thb));

  float q1 = atan2(Y, X);
  float q2 = alpha - HALF_PI;                 // lengan atas tegak = q2 0
  float q3 = thb + HALF_PI;                   // forearm lurus ke depan = q3 0
  float q4 = phi - q2 - q3;

  q[0] = wrapDeg(q1 * RAD_TO_DEG);
  q[1] = wrapDeg(q2 * RAD_TO_DEG);
  q[2] = wrapDeg(q3 * RAD_TO_DEG);
  q[3] = wrapDeg(q4 * RAD_TO_DEG);
  return true;
}

// ==========================================
// 6. BATASI NILAI PWM
// ==========================================
int limitPWM(int servoIndex, int value) {
  if (value < servoMin[servoIndex]) value = servoMin[servoIndex];
  if (value > servoMax[servoIndex]) value = servoMax[servoIndex];
  return value;
}

// ==========================================
// 7. AKTIFKAN / NONAKTIFKAN SINYAL SERVO
// ==========================================
void enableServoSignals() {
  for (int i = 0; i < SERVO_COUNT; i++) {
    pwm.setPWM(servoChannels[i], 0, currentPWM[i]);
  }
}

void disableServoSignals() {
  // Menghentikan pulsa kendali channel PCA9685.
  // Catu daya servo tetap menyala.
  for (int i = 0; i < SERVO_COUNT; i++) {
    pwm.setPWM(servoChannels[i], 0, 4096);
  }
}

// ==========================================
// 8. KIRIM PWM KE SERVO
// ==========================================
void writeServo(int servoIndex, int value) {
  value = limitPWM(servoIndex, value);
  currentPWM[servoIndex] = value;

  if (!freeMode) {
    pwm.setPWM(servoChannels[servoIndex], 0, currentPWM[servoIndex]);
  }
}

// ==========================================
// 9. GERAKAN MENUJU TARGET (semua servo tiba bersamaan)
// ==========================================
void startMoveToTargets(const int targets[], unsigned long duration) {
  for (int i = 0; i < SERVO_COUNT; i++) {
    startPWM[i] = currentPWM[i];
    targetPWM[i] = limitPWM(i, targets[i]);
  }

  moveDuration = duration;
  moveStartTime = millis();
  lastUpdateTime = 0;
  moving = true;
}

// Durasi gerak dari selisih terbesar dan kecepatan speedDps
unsigned long durationFor(const int targets[]) {
  float maxDeg = 0;
  for (int i = 0; i < SERVO_COUNT; i++) {
    float d = fabs((float)(limitPWM(i, targets[i]) - currentPWM[i])) / ticksPerDeg(i);
    if (d > maxDeg) maxDeg = d;
  }
  unsigned long ms = (unsigned long)(maxDeg / speedDps * 1000.0f);
  if (ms < 100) ms = 100;
  return ms;
}

void updateMovement() {
  if (!moving || freeMode) return;

  unsigned long now = millis();
  if (now - lastUpdateTime < UPDATE_INTERVAL) return;
  lastUpdateTime = now;

  unsigned long elapsed = now - moveStartTime;
  float progress = (float)elapsed / (float)moveDuration;
  if (progress > 1.0f) progress = 1.0f;

  for (int i = 0; i < SERVO_COUNT; i++) {
    float position = startPWM[i] + ((float)targetPWM[i] - startPWM[i]) * progress;
    writeServo(i, (int)(position + 0.5f));
  }

  if (progress >= 1.0f) {
    for (int i = 0; i < SERVO_COUNT; i++) writeServo(i, targetPWM[i]);
    moving = false;
    Serial.println("Gerakan selesai.");
  }
}

// ==========================================
// 9b. PLAYBACK TRAJEKTORI
// ==========================================
void abortTraj() {
  trjPlaying = false;
  trjGoingStart = false;
  trjLoading = false;
}

void startTrajPlayback() {
  trjPlaying = true;
  trjStartTime = millis();
  lastUpdateTime = 0;
  moving = false;
  Serial.println("PLAY");
}

void trajGo() {
  if (freeMode) { Serial.println("ERR: mode bebas aktif, ketik s dulu"); return; }
  if (trjPlaying || trjGoingStart) { Serial.println("ERR: trajektori sudah berjalan"); return; }
  if (trjCount < 2) { Serial.println("ERR: buffer kosong, upload dulu"); return; }

  // pindah ke titik pertama dulu kalau posisi sekarang jauh darinya
  int t[SERVO_COUNT];
  bool far = false;
  for (int i = 0; i < SERVO_COUNT; i++) {
    t[i] = qToPWM(i, trjBuf[0][i] / 100.0f);
    if (abs(currentPWM[i] - t[i]) > 2) far = true;
  }
  if (far) {
    unsigned long d = durationFor(t);
    if (d < 500) d = 500;
    startMoveToTargets(t, d);
    trjGoingStart = true;
    Serial.println("START_MOVE");
  } else {
    startTrajPlayback();
  }
}

void updateTrajectory() {
  if (trjGoingStart && !moving) {              // gerak ke titik awal sudah selesai
    trjGoingStart = false;
    startTrajPlayback();
    return;
  }
  if (!trjPlaying || freeMode) return;

  unsigned long now = millis();
  if (now - lastUpdateTime < UPDATE_INTERVAL) return;
  lastUpdateTime = now;

  float idxf = (float)(now - trjStartTime) / (float)trjDtMs;
  int i0 = (int)idxf;

  if (i0 >= trjCount - 1) {                    // selesai: tahan di titik terakhir
    for (int j = 0; j < SERVO_COUNT; j++) {
      writeServo(j, qToPWM(j, trjBuf[trjCount - 1][j] / 100.0f));
    }
    trjPlaying = false;
    Serial.println("DONE");
    return;
  }

  float fr = idxf - i0;
  for (int j = 0; j < SERVO_COUNT; j++) {
    float q = ((1.0f - fr) * trjBuf[i0][j] + fr * trjBuf[i0 + 1][j]) / 100.0f;
    writeServo(j, qToPWM(j, q));
  }
}

void handleTrj(const char *sub) {
  if (!strcmp(sub, "BEGIN")) {
    if (freeMode) { Serial.println("ERR: mode bebas aktif, ketik s dulu"); return; }
    if (trjPlaying || trjGoingStart) { Serial.println("ERR: trajektori sedang berjalan"); return; }
    char *tn = strtok(NULL, " ,");
    char *td = strtok(NULL, " ,");
    if (!tn || !td) { Serial.println("ERR: TRJ BEGIN n dt_ms"); return; }
    int n = atoi(tn);
    long dt = atol(td);
    if (n < 2 || n > TRJ_MAX) { Serial.println("ERR: n harus 2..1200"); return; }
    if (dt < 5 || dt > 1000)  { Serial.println("ERR: dt_ms harus 5..1000"); return; }
    trjExpected = n;
    trjDtMs = (unsigned long)dt;
    trjLoaded = 0;
    trjCount = 0;
    trjLoading = true;
    Serial.println("OK BEGIN");

  } else if (!strcmp(sub, "P")) {
    if (!trjLoading) { Serial.println("ERR: kirim TRJ BEGIN dulu"); return; }
    if (trjLoaded >= trjExpected) { Serial.println("ERR: titik melebihi n"); trjLoading = false; return; }
    float v[SERVO_COUNT];
    for (int i = 0; i < SERVO_COUNT; i++) {
      char *t = strtok(NULL, " ,");
      if (!t) { trjLoading = false; trjCount = 0; Serial.println("ERR: butuh 6 nilai"); return; }
      v[i] = atof(t);
    }
    for (int i = 0; i < SERVO_COUNT; i++) {
      int p = qToPWM(i, v[i]);
      if (p < servoMin[i] || p > servoMax[i]) {      // tolak, jangan jepit diam-diam
        trjLoading = false;
        trjCount = 0;
        Serial.print("ERR: titik ");
        Serial.print(trjLoaded);
        Serial.print(" servo ");
        Serial.print(i + 1);
        Serial.println(" di luar batas");
        return;
      }
      trjBuf[trjLoaded][i] = (int16_t)lroundf(v[i] * 100.0f);
    }
    trjLoaded++;
    Serial.println("K");

  } else if (!strcmp(sub, "END")) {
    if (!trjLoading || trjLoaded != trjExpected) {
      trjLoading = false;
      trjCount = 0;
      Serial.println("ERR: jumlah titik tidak sesuai");
      return;
    }
    trjCount = trjLoaded;
    trjLoading = false;
    Serial.print("OK END ");
    Serial.println(trjCount);

  } else if (!strcmp(sub, "GO")) {
    trajGo();

  } else if (!strcmp(sub, "STOP")) {
    abortTraj();
    moving = false;
    Serial.println("OK STOP");

  } else if (!strcmp(sub, "STAT")) {
    Serial.print("TRJ count="); Serial.print(trjCount);
    Serial.print(" dt_ms=");    Serial.print(trjDtMs);
    Serial.print(" loading=");  Serial.print(trjLoading);
    Serial.print(" playing=");  Serial.print(trjPlaying);
    Serial.print(" toStart=");  Serial.println(trjGoingStart);

  } else {
    Serial.println("ERR: TRJ BEGIN|P|END|GO|STOP|STAT");
  }
}

// ==========================================
// 10. HOME, FREE MOVEMENT, LOCK
// ==========================================
void moveToHome() {
  abortTraj();
  freeMode = false;
  moving = false;
  enableServoSignals();

  int homeTargets[SERVO_COUNT];
  for (int i = 0; i < SERVO_COUNT; i++) homeTargets[i] = servoHome[i];

  startMoveToTargets(homeTargets, HOME_DURATION);

  Serial.println();
  Serial.println("Robot menuju HOME.");
  Serial.println("Target durasi: 2 detik.");
  Serial.println("Pastikan posisi lengan aman sebelum menjalankan gerakan.");
}

void moveToPlace() {
  abortTraj();
  freeMode = false;
  moving = false;
  enableServoSignals();

  startMoveToTargets(placePWM, PLACE_DURATION);

  Serial.println();
  Serial.println("Robot menuju PLACE.");
  Serial.println("Target durasi: 2 detik.");
  Serial.println("Pastikan posisi lengan aman sebelum menjalankan gerakan.");
}

void enterFreeMode() {
  abortTraj();
  moving = false;
  freeMode = true;
  disableServoSignals();

  Serial.println();
  Serial.println("FREE MOVEMENT AKTIF.");
  Serial.println("Sinyal kendali servo dihentikan.");
  Serial.println("Lengan dapat dicoba digerakkan dengan tangan.");
  Serial.println("Ketik s untuk mengaktifkan kembali sinyal servo.");
}

void lockServos() {
  abortTraj();
  moving = false;

  if (freeMode) {
    freeMode = false;
    enableServoSignals();

    Serial.println();
    Serial.println("LOCK AKTIF.");
    Serial.println("Sinyal kendali servo diaktifkan kembali.");
    Serial.println("PERINGATAN: servo mungkin bergerak ke PWM tersimpan.");
  } else {
    Serial.println("Gerakan program dihentikan.");
    Serial.println("Sinyal servo tetap aktif.");
  }
}

// ==========================================
// 11. KONTROL MANUAL 1-6, + DAN -
// ==========================================
void manualControl(char cmd) {
  if (cmd >= '1' && cmd <= '6') {
    selectedServo = cmd - '1';
    Serial.print("Servo terpilih: ");
    Serial.println(selectedServo + 1);
    return;
  }

  if (cmd == '+' || cmd == '-') {
    if (freeMode) {
      Serial.println("Mode bebas aktif. Ketik s untuk mengaktifkan sinyal servo.");
      return;
    }
    if (moving || trjPlaying || trjGoingStart) {
      Serial.println("Robot sedang bergerak. Ketik s untuk menghentikan gerakan.");
      return;
    }

    int change = (cmd == '+') ? MANUAL_STEP : -MANUAL_STEP;
    writeServo(selectedServo, currentPWM[selectedServo] + change);

    Serial.print("Servo ");
    Serial.print(selectedServo + 1);
    Serial.print(" PWM = ");
    Serial.print(currentPWM[selectedServo]);
    Serial.print(" (q = ");
    Serial.print(pwmToQ(selectedServo, currentPWM[selectedServo]), 1);
    Serial.println(selectedServo == 5 ? " %)" : " deg)");
  }
}

// ==========================================
// 12. BANTUAN
// ==========================================
void printHelp() {
  Serial.println();
  Serial.println("===== ROBOT ARM 6-DOF =====");
  Serial.println("b   : Free Movement");
  Serial.println("s   : LOCK / aktifkan sinyal servo / STOP gerakan & trajektori");
  Serial.println("m   : Kembali ke HOME (target 2 detik)");
  Serial.println("p   : Ke pose PLACE (target 2 detik)");
  Serial.println("1-6 : Pilih servo, lalu + / - (5 tick)");
  Serial.println("J q1 q2 q3 q4 q5 grip : gerak semua joint (deg, grip 0..100)");
  Serial.println("Q n nilai             : gerak satu joint (n = 1..6)");
  Serial.println("P x y z pitch [roll] [grip] : IK, ujung gripper ke (x,y,z) mm");
  Serial.println("FK  : posisi EE saat ini");
  Serial.println("SPD nilai : kecepatan (deg/detik)");
  Serial.println("PING / GETQ / TRJ BEGIN|P|END|GO|STOP|STAT : untuk Python");
  Serial.println("?   : Bantuan");
  Serial.println("===========================");
}

// ==========================================
// 13. PERINTAH SERIAL
// ==========================================
void handleKey(char c) {
  if (c == 'b' || c == 'B') { enterFreeMode(); return; }
  if (c == 's' || c == 'S') { lockServos();    return; }
  if (c == 'm' || c == 'M') { moveToHome();    return; }
  if (c == 'p' || c == 'P') { moveToPlace();   return; }
  if (c == '?')             { printHelp();     return; }
  if ((c >= '1' && c <= '6') || c == '+' || c == '-') { manualControl(c); return; }
  Serial.println("Perintah tidak dikenal. Ketik ? untuk bantuan.");
}

bool busyTraj() {
  if (trjPlaying || trjGoingStart) {
    Serial.println("Trajektori sedang berjalan. Ketik s untuk menghentikan.");
    return true;
  }
  return false;
}

void handleLine(char *line) {
  while (*line == ' ') line++;
  if (*line == 0) return;

  // Perintah satu karakter (b, s, m, 1-6, +, -, ?)
  if (strlen(line) == 1) { handleKey(line[0]); return; }

  char *cmd = strtok(line, " ,");
  if (!cmd) return;
  for (char *p = cmd; *p; ++p) *p = toupper(*p);

  if (!strcmp(cmd, "PING")) {
    Serial.println("PONG");

  } else if (!strcmp(cmd, "GETQ")) {
    Serial.print("Q");
    for (int i = 0; i < SERVO_COUNT; i++) {
      Serial.print(" ");
      Serial.print(pwmToQ(i, currentPWM[i]), 2);
    }
    Serial.println();

  } else if (!strcmp(cmd, "TRJ")) {
    char *sub = strtok(NULL, " ,");
    if (!sub) { Serial.println("ERR: TRJ BEGIN|P|END|GO|STOP|STAT"); return; }
    for (char *p = sub; *p; ++p) *p = toupper(*p);
    handleTrj(sub);

  } else if (!strcmp(cmd, "J")) {
    if (freeMode) { Serial.println("Mode bebas aktif. Ketik s dulu."); return; }
    if (busyTraj()) return;
    float v[SERVO_COUNT];
    for (int i = 0; i < SERVO_COUNT; i++) {
      char *t = strtok(NULL, " ,");
      if (!t) { Serial.println("ERR: butuh 6 nilai (q1..q5 grip)"); return; }
      v[i] = atof(t);
    }
    int targets[SERVO_COUNT];
    for (int i = 0; i < SERVO_COUNT; i++) {
      targets[i] = qToPWM(i, v[i]);
      if (limitPWM(i, targets[i]) != targets[i]) {
        Serial.print("Batas servo "); Serial.print(i + 1);
        Serial.println(": nilai dijepit ke batas PWM.");
      }
    }
    startMoveToTargets(targets, durationFor(targets));
    printFK(targetPWM, F("Target"));

  } else if (!strcmp(cmd, "Q")) {
    if (freeMode) { Serial.println("Mode bebas aktif. Ketik s dulu."); return; }
    if (busyTraj()) return;
    char *tn = strtok(NULL, " ,");
    char *tv = strtok(NULL, " ,");
    if (!tn || !tv) { Serial.println("ERR: Q n nilai"); return; }
    int n = atoi(tn);
    if (n < 1 || n > SERVO_COUNT) { Serial.println("ERR: n harus 1..6"); return; }

    int targets[SERVO_COUNT];
    for (int i = 0; i < SERVO_COUNT; i++) targets[i] = moving ? targetPWM[i] : currentPWM[i];
    targets[n - 1] = qToPWM(n - 1, atof(tv));
    if (limitPWM(n - 1, targets[n - 1]) != targets[n - 1]) {
      Serial.println("Batas servo: nilai dijepit ke batas PWM.");
    }
    startMoveToTargets(targets, durationFor(targets));
    printFK(targetPWM, F("Target"));

  } else if (!strcmp(cmd, "P")) {
    if (freeMode) { Serial.println("Mode bebas aktif. Ketik s dulu."); return; }
    if (busyTraj()) return;
    float vals[6];
    int n = 0;
    char *t;
    while (n < 6 && (t = strtok(NULL, " ,")) != NULL) vals[n++] = atof(t);
    if (n < 4) { Serial.println("ERR: P x y z pitch [roll] [grip]"); return; }

    float q[4];
    if (!solveIK(vals[0], vals[1], vals[2], vals[3], q)) {
      Serial.println("IK GAGAL: target di luar jangkauan.");
      return;
    }

    // roll dan gripper: pakai nilai yang diberikan, kalau tidak ada pertahankan yang sekarang
    const int *base = moving ? targetPWM : currentPWM;
    float roll = (n >= 5) ? vals[4] : pwmToQ(4, base[4]);
    float grip = (n >= 6) ? vals[5] : pwmToQ(5, base[5]);

    int targets[SERVO_COUNT];
    for (int i = 0; i < 4; i++) targets[i] = qToPWM(i, q[i]);
    targets[4] = qToPWM(4, roll);
    targets[5] = qToPWM(5, grip);

    // Tolak kalau ada joint di luar batas servo (tidak dijepit diam-diam)
    for (int i = 0; i < SERVO_COUNT; i++) {
      if (limitPWM(i, targets[i]) != targets[i]) {
        Serial.print("IK DITOLAK: servo "); Serial.print(i + 1);
        Serial.print(" di luar batas (q = "); Serial.print(i < 4 ? q[i] : (i == 4 ? roll : grip), 1);
        Serial.println(").");
        return;
      }
    }

    startMoveToTargets(targets, durationFor(targets));
    Serial.print("IK: q1..q4 = [");
    for (int i = 0; i < 4; i++) { Serial.print(q[i], 1); Serial.print(i < 3 ? " " : "]\n"); }
    printFK(targetPWM, F("Target"));

  } else if (!strcmp(cmd, "FK")) {
    printFK(currentPWM, F("Saat ini"));

  } else if (!strcmp(cmd, "SPD")) {
    char *t = strtok(NULL, " ,");
    if (!t) { Serial.println("ERR: SPD nilai"); return; }
    speedDps = constrain(atof(t), 5.0, 200.0);
    Serial.print("Kecepatan = "); Serial.print(speedDps, 0); Serial.println(" deg/s");

  } else {
    Serial.println("Perintah tidak dikenal. Ketik ? untuk bantuan.");
  }
}

void readSerial() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (lineLen) { lineBuf[lineLen] = 0; handleLine(lineBuf); lineLen = 0; }
    } else if (lineLen < sizeof(lineBuf) - 1) {
      lineBuf[lineLen++] = c;
    }
  }
}

// ==========================================
// 14. SETUP
// ==========================================
void setup() {
  Serial.begin(115200);

  Wire.begin(SDA_PIN, SCL_PIN);

  pwm.begin();
  pwm.setPWMFreq(50);

  delay(300);

  // Inisialisasi perintah PWM ke nilai Home.
  // Posisi fisik servo belum tentu berada di Home.
  for (int i = 0; i < SERVO_COUNT; i++) {
    pwm.setPWM(servoChannels[i], 0, servoHome[i]);
  }

  Serial.println();
  Serial.println("Robot Arm 6-DOF siap (versi trajektori).");
  Serial.println("Baud rate: 115200");
  Serial.println("Perhatian: posisi fisik awal servo belum diketahui.");

  printHelp();
}

// ==========================================
// 15. LOOP UTAMA
// ==========================================
void loop() {
  updateMovement();
  updateTrajectory();
  readSerial();
}
