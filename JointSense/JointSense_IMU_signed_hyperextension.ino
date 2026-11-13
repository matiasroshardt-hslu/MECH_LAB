/*
  JointSense – IMU prototype, main program VERSION 2 (Team TM-38, PDP1)       As of 13.11.2026
  ------------------------------------------------------------------------------------------
  What is new compared with version 1 (JointSense_IMU):
    1. More accurate data: gyro offset measured only while the arm is really still (repeats
       until it is), slow automatic offset tracking whenever the arm rests, plausibility checks
       on every reading, median of 5 on the angle (removes single extreme values).
    2. Vibration ONLY for real hyperextension: angle <= -10 deg for at least 150 ms, and only
       after the calibration (0 deg + at least one point >= 45 deg away). Ends at -5 deg.
       The 120-degree bending warning of version 1 is switched off.
    3. Multi-point calibration: hold the elbow at a known angle and type that angle
       (e.g. 0, 45, 90, 135). Up to 8 points; between the points the angle is interpolated
       linearly. Every point is averaged over 1 s and only accepted if the arm is held still.
       The calibration and the limit are SAVED (Preferences) and loaded at every start.
    4. A missing sensor no longer stops the program: it reports the error, blinks and retries.
    5. Measuring tools: "m" = 2-s measurement, "l" = CSV log at 100 Hz, "r" = raw gyro data.

  STATUS: signed-angle revision; NOT yet verified on real hardware.
  Angle is kept continuous across +/-180 deg; mounting axes and gyro polarity must be checked.

  Hardware and wiring: unchanged (file "2 Wiring IMU Prototype.png").
    Adafruit ESP32 Feather V2, MPU-6050 #1 upper arm 0x68, MPU-6050 #2 forearm 0x69 (AD0 closed),
    DRV2605L 0x5A + vibration disc, all on the STEMMA QT chain. Button SW38 = GPIO38 (LOW = pressed).
    Mounting: both MPUs with the same orientation: X along the arm (towards the hand),
    Y parallel to the elbow axis, Z away from the arm.

  Angle convention: 0 deg = arm straight, bending = positive, hyperextension = negative.

  Operation:
    SW38 short (< 2 s)  = calibration point 0 deg  (arm straight, hold still)
    SW38 long (>= 2 s)  = calibration point 90 deg (elbow at 90 deg, hold still; release after the click)
    Serial Monitor (115200 baud, "New Line"):
      c          = guided calibration (0 deg -> 90 deg -> more angles -> check)
      <number>   = calibration point at this angle, e.g. 0, 45, 90, 135, -10  (Enter alone = 0)
      t          = show the calibration table        clear = delete the whole calibration
      g -12      = warning limit -12 deg             m     = measure the current angle for 2 s
      b          = measure the gyro offset again     r     = raw gyro data on/off (axis check)
      l          = CSV log on/off                    s     = status          h = help
    LED (red, GPIO13): on = warning, blinks slowly = not calibrated, blinks fast = sensor error.

  Arduino IDE:
    Board package: "esp32 by Espressif Systems" -> board "Adafruit Feather ESP32 V2"
    Libraries:     Adafruit MPU6050 (+ Adafruit Unified Sensor, Adafruit BusIO), Adafruit DRV2605 Library
    (Preferences is part of the board package, no extra library needed.)
*/

#include <stddef.h>
#include <string.h>
#include <Wire.h>
#include <Preferences.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_DRV2605.h>

// ======================= SETTINGS =======================
// Warning
const float DEFAULT_WARN_LIMIT = -10.0;  // [deg] warning at angle <= limit (hyperextension); "g -12" changes it (saved)
const float HYSTERESIS = 5.0;            // [deg] warning ends only at angle >= limit + HYSTERESIS
const unsigned long WARN_ON_MS = 150;    // angle must stay beyond the limit this long before it vibrates
const unsigned long WARN_OFF_MS = 100;   // angle must stay back in range this long before the warning ends
const unsigned long VIB_PAUSE_MS = 1000; // time between two warning vibrations
const bool WARN_NEEDS_CALIBRATION = true; // true = no warnings before 0 deg + a point >= 45 deg away are calibrated
const bool FLEX_WARNING = false;         // true = also warn at strong bending (>= ANGLE_MAX)
const float ANGLE_MAX = 120.0;           // [deg] only used with FLEX_WARNING = true
// Vibration feedback
const bool CONFIRM_VIBRATION = true;     // short clicks for start/calibration (false = vibrate ONLY for warnings)
const bool ERROR_VIBRATION = false;      // true = double click every 3 s on sensor errors
const uint8_t EFFECT_WARN = 14;          // Strong Buzz 100 %
const uint8_t EFFECT_OK = 1;             // Strong Click 100 %
const uint8_t EFFECT_ERROR = 10;         // Double Click 100 %
// Sensor fusion / filter
const float ALPHA = 0.98;                // complementary filter: share of the gyro (rest: accelerometer)
const int MEDIAN_N = 5;                  // median window on the angle (odd, 3..9) – removes single extreme values
const float SPIKE_DEG = 15.0;            // [deg] deviation from the median that is counted as an outlier
const float ACC_MIN_G = 0.2, ACC_MAX_G = 3.5;  // plausible acceleration magnitude [g]
const float GYRO_MAX_DPS = 480.0;        // plausible rotation rate [deg/s] (range is +-500)
const bool GYRO_MEDIAN3 = true;          // median of 3 on each gyro rate (removes single gyro spikes, +10 ms delay)
const int SIGN = 1;                      // direction before calibration (-1 if bending counts negative)
// Gyro offset
const unsigned long BIAS_MS = 2000;      // averaging time for the gyro offset at start
const float STILL_GYRO_SD = 0.5;         // [deg/s] max. gyro noise while "still"
const float STILL_ACC_TOL = 0.08;        // [g] max. deviation of |a| from 1 g while "still"
const bool BIAS_TRACKING = true;         // adjust the offset slowly whenever the arm rests
const float BIAS_TRACK_RATE = 2.0;       // [deg/s] below this both sensors count as "resting"
const float BIAS_TRACK_GAIN = 0.002;     // per sample (100 Hz) -> time constant approx. 5 s
// Calibration
const int MAX_POINTS = 8;
const unsigned long CAL_SETTLE_MS = 300;
const unsigned long CAL_MS = 1000;
const float CAL_MAX_SD = 0.5;            // [deg] max. standard deviation while calibrating (= hold still)
const float CAL_MAX_RANGE = 2.0;         // [deg] max. difference max-min while calibrating
const float CAL_MIN_SPAN = 45.0;
const float CAL_MIN_STEP_TRUE = 5.0;
const float CAL_MIN_STEP_RAW = 2.0;
const unsigned long MEASURE_MS = 2000;
// Hardware
const int BUTTON_PIN = 38;               // built-in button SW38, LOW = pressed
const int LED_PIN = 13;                  // red LED on the Feather
const unsigned long LONG_MS = 2000;
const uint8_t ADR_UPPER = 0x68, ADR_LOWER = 0x69, ADR_DRV = 0x5A;
// Timing
const unsigned long LOOP_US = 10000;     // 10 ms = 100 Hz
const unsigned long OUTPUT_MS = 50;      // 20 Hz output for the Serial Plotter
const unsigned long SENSOR_TIMEOUT_MS = 500;
// =========================================================

Adafruit_MPU6050 imuUpper;  // upper arm 0x68
Adafruit_MPU6050 imuLower;  // forearm   0x69
Adafruit_DRV2605 drv;
Preferences prefs;
bool upperOk = false, lowerOk = false, drvOk = false;

// ---------- calibration table (saved with Preferences) ----------
const uint32_t CAL_MAGIC = 0x4A53494DUL;  // "JSIM"
const uint16_t CAL_VERSION = 3;
struct CalData {
  uint32_t magic;
  uint16_t version;
  uint16_t size;
  float warnLimit;
  uint8_t n;
  uint8_t spare[3];
  float trueDeg[MAX_POINTS];    // known angle, sorted ascending
  float rawDeg[MAX_POINTS];     // fused IMU angle at this position
  uint32_t check;
};
CalData cal;
bool calFromFlash = false;
float startRaw = 0;

// ---------- measurement state ----------
struct Reading { float tilt; float rateY; float accG; bool accelOk; float gx, gz; };  // one IMU reading
float biasUpper = 0, biasLower = 0;  // gyro offsets [deg/s]
bool biasStill = false;              // offset was measured while still
bool biasMeasured = false;           // offset was measured at all
float angleRaw = 0;                  // continuous (unwrapped) fused relative angle [deg]
bool fusionInit = false;
float medBuf[MEDIAN_N];
int medCount = 0, medPos = 0;
float med = 0;                       // median of the fused angle
float angleAccelLast = 0;
struct Stats { unsigned long samples, implausible, spikes, accelUsed, biasUpdates; };
Stats st = {0, 0, 0, 0, 0};
bool sensorError = false;
unsigned long lastSample = 0, lastRestart = 0, lastMicros = 0, lastFusionUs = 0;
float lastGyroU = 0, lastGyroL = 0;  // raw gyro y [deg/s] for the "r" output
float gyHistU[3] = {0, 0, 0}, gyHistL[3] = {0, 0, 0};
int gyCount = 0;
float gxU = 0, gzU = 0, gxL = 0, gzL = 0;

// ---------- warning / operation ----------
bool warnActive = false, beyondTiming = false, backTiming = false;
unsigned long beyondSince = 0, backSince = 0, lastVib = 0, lastOutput = 0;
bool csvLog = false, rawOut = false;
bool buttonPressed = false, longSignaled = false;
unsigned long buttonSince = 0;
char inputBuf[24];
int inputLen = 0;

// ======================= calibration table =======================
uint32_t checksum(const CalData& c) {
  const uint8_t* p = (const uint8_t*)&c;
  uint32_t h = 2166136261UL;
  for (size_t i = 0; i < offsetof(CalData, check); i++) { h ^= p[i]; h *= 16777619UL; }
  return h;
}

void calDefaults() {
  memset(&cal, 0, sizeof(cal));
  cal.magic = CAL_MAGIC; cal.version = CAL_VERSION; cal.size = sizeof(CalData);
  cal.warnLimit = DEFAULT_WARN_LIMIT;
  cal.n = 0;
  cal.check = checksum(cal);
}

bool tableValid(const CalData& c, const char** why) {
  for (int i = 1; i < c.n; i++) {
    if (c.trueDeg[i] - c.trueDeg[i - 1] < CAL_MIN_STEP_TRUE) { *why = "angles too close together"; return false; }
    if (fabs(c.rawDeg[i] - c.rawDeg[i - 1]) < CAL_MIN_STEP_RAW) { *why = "measured angle hardly changed between the two points"; return false; }
  }
  if (c.n >= 3) {
    bool up = c.rawDeg[1] > c.rawDeg[0];
    for (int i = 2; i < c.n; i++) {
      if ((c.rawDeg[i] > c.rawDeg[i - 1]) != up) { *why = "values do not follow the angle order (sensor slipped or wrong angle typed?)"; return false; }
    }
  }
  return true;
}

bool calLoad() {
  CalData c;
  prefs.begin("jointsense", true);
  size_t len = prefs.getBytesLength("cal");
  bool ok = (len == sizeof(CalData)) && (prefs.getBytes("cal", &c, sizeof(c)) == sizeof(c));
  prefs.end();
  if (!ok) return false;
  if (c.magic != CAL_MAGIC || c.version != CAL_VERSION || c.size != sizeof(CalData) || c.check != checksum(c)) return false;
  if (c.n > MAX_POINTS || !(c.warnLimit >= -45.0 && c.warnLimit <= 0.0)) return false;
  const char* why;
  if (!tableValid(c, &why)) return false;
  cal = c;
  return true;
}

bool calSave() {
  cal.magic = CAL_MAGIC; cal.version = CAL_VERSION; cal.size = sizeof(CalData);
  cal.check = checksum(cal);
  prefs.begin("jointsense", false);
  size_t w = prefs.putBytes("cal", &cal, sizeof(cal));
  prefs.end();
  return w == sizeof(cal);
}

int zeroIndex() {
  for (int i = 0; i < cal.n; i++) if (fabs(cal.trueDeg[i]) < 0.01) return i;
  return -1;
}

bool fullyCalibrated() {
  if (zeroIndex() < 0 || cal.n < 2) return false;
  return (cal.trueDeg[cal.n - 1] - cal.trueDeg[0]) >= CAL_MIN_SPAN;
}

bool warningsEnabled() { return !WARN_NEEDS_CALIBRATION || fullyCalibrated(); }

float rawToAngle(float r) {
  if (cal.n == 0) return SIGN * (r - startRaw);
  if (cal.n == 1) return cal.trueDeg[0] + SIGN * (r - cal.rawDeg[0]);
  bool up = cal.rawDeg[1] > cal.rawDeg[0];
  int seg = -1;
  for (int i = 0; i < cal.n - 1; i++) {
    float lo = cal.rawDeg[i], hi = cal.rawDeg[i + 1];
    if ((up && r >= lo && r <= hi) || (!up && r <= lo && r >= hi)) { seg = i; break; }
  }
  if (seg < 0) seg = ((up && r < cal.rawDeg[0]) || (!up && r > cal.rawDeg[0])) ? 0 : cal.n - 2;
  float t = (r - cal.rawDeg[seg]) / (cal.rawDeg[seg + 1] - cal.rawDeg[seg]);
  return cal.trueDeg[seg] + t * (cal.trueDeg[seg + 1] - cal.trueDeg[seg]);
}

void printTable() {
  Serial.print("Calibration table ("); Serial.print(cal.n); Serial.print(" of "); Serial.print(MAX_POINTS);
  Serial.println(" points), angle [deg] -> IMU angle [deg]:");
  if (cal.n == 0) { Serial.println("  (empty)"); return; }
  for (int i = 0; i < cal.n; i++) {
    Serial.print("  "); Serial.print(cal.trueDeg[i], 1); Serial.print(" -> "); Serial.print(cal.rawDeg[i], 2);
    if (i > 0) {
      Serial.print("   (factor "); Serial.print((cal.trueDeg[i] - cal.trueDeg[i - 1]) / (cal.rawDeg[i] - cal.rawDeg[i - 1]), 4);
      Serial.print(" to the previous point)");
    }
    Serial.println();
  }
}

// ======================= helpers =======================
void vibrate(uint8_t effect) {
  if (!drvOk) return;
  drv.setWaveform(0, effect);
  drv.setWaveform(1, 0);  // end of sequence
  drv.go();
}

void confirmClick(int count) {
  if (!CONFIRM_VIBRATION) return;
  for (int i = 0; i < count; i++) { if (i) delay(150); vibrate(EFFECT_OK); }
}

void resetWarning() { warnActive = false; beyondTiming = false; backTiming = false; }

bool buttonDown() { return digitalRead(BUTTON_PIN) == LOW; }

float wrap180(float w) {
  while (w > 180.0) w -= 360.0;
  while (w < -180.0) w += 360.0;
  return w;
}

bool i2cPresent(uint8_t adr) {
  Wire.beginTransmission(adr);
  return Wire.endTransmission() == 0;
}

void configureImu(Adafruit_MPU6050& imu) {
  imu.setAccelerometerRange(MPU6050_RANGE_4_G);
  imu.setGyroRange(MPU6050_RANGE_500_DEG);
  imu.setFilterBandwidth(MPU6050_BAND_21_HZ);
}

bool initDevices() {
  upperOk = imuUpper.begin(ADR_UPPER, &Wire);
  lowerOk = imuLower.begin(ADR_LOWER, &Wire);
  if (upperOk) configureImu(imuUpper);
  if (lowerOk) configureImu(imuLower);
  if (!drvOk) {
    drvOk = drv.begin(&Wire);
    if (drvOk) { drv.selectLibrary(1); drv.setMode(DRV2605_MODE_INTTRIG); }
  }
  return upperOk && lowerOk;
}

// ======================= sensor reading =======================

// Tilt about Y from the accelerometer [deg]; true if the reading is plausible
bool readImu(Adafruit_MPU6050& imu, Reading& r) {
  sensors_event_t a, g, t;
  if (!imu.getEvent(&a, &g, &t)) return false;
  float ax = a.acceleration.x, ay = a.acceleration.y, az = a.acceleration.z;
  float acc = sqrt(ax * ax + ay * ay + az * az) / 9.81;
  float gyDps = g.gyro.y * RAD_TO_DEG;
  if (!(acc >= ACC_MIN_G && acc <= ACC_MAX_G) || fabs(gyDps) > GYRO_MAX_DPS || isnan(gyDps)) return false;
  r.tilt = atan2(-ax, az) * RAD_TO_DEG;
  r.rateY = gyDps;
  r.accG = acc;
  float xz = sqrt(ax * ax + az * az) / 9.81;
  r.accelOk = (xz > 0.5) && (fabs(acc - 1.0) < 0.25);     // elbow axis not vertical, arm not accelerating
  r.gx = g.gyro.x * RAD_TO_DEG;
  r.gz = g.gyro.z * RAD_TO_DEG;
  return true;
}

void medianReset() { medCount = 0; medPos = 0; gyCount = 0; }

float median3(float a, float b, float c) {
  if (a > b) { float t = a; a = b; b = t; }
  if (b > c) { float t = b; b = c; c = t; }
  if (a > b) { float t = a; a = b; b = t; }
  return b;
}

// keeps the last 3 gyro rates and returns their median (or the newest value at the start)
float gyroFilter(float* hist, float x) {
  hist[0] = hist[1]; hist[1] = hist[2]; hist[2] = x;
  if (!GYRO_MEDIAN3 || gyCount < 3) return x;
  return median3(hist[0], hist[1], hist[2]);
}

float medianPush(float x) {
  // Unwrap around the previous median to avoid discontinuity at +/-180 degrees.
  if (medCount > 0) x = med + wrap180(x - med);
  medBuf[medPos] = x;
  medPos = (medPos + 1) % MEDIAN_N;
  if (medCount < MEDIAN_N) medCount++;
  float tmp[MEDIAN_N];
  for (int i = 0; i < medCount; i++) tmp[i] = medBuf[i];
  for (int i = 1; i < medCount; i++) {
    float k = tmp[i]; int j = i - 1;
    while (j >= 0 && tmp[j] > k) { tmp[j + 1] = tmp[j]; j--; }
    tmp[j + 1] = k;
  }
  if (medCount % 2) return tmp[medCount / 2];
  return 0.5f * (tmp[medCount / 2 - 1] + tmp[medCount / 2]);
}

// one fusion step (dt in s). true = both readings valid
bool sampleStep(float dt) {
  Reading u, l;
  bool okU = upperOk && readImu(imuUpper, u);
  bool okL = lowerOk && readImu(imuLower, l);
  if (!okU || !okL) { st.implausible++; return false; }
  lastGyroU = u.rateY; lastGyroL = l.rateY; gxU = u.gx; gzU = u.gz; gxL = l.gx; gzL = l.gz;
  if (gyCount < 3) gyCount++;
  float gU = gyroFilter(gyHistU, u.rateY), gL = gyroFilter(gyHistL, l.rateY);   // spike-free gyro rates
  float rateU = gU - biasUpper, rateL = gL - biasLower;
  float angleAccel = wrap180(l.tilt - u.tilt);
  angleAccelLast = angleAccel;
  if (!fusionInit) { angleRaw = angleAccel; fusionInit = true; }
  // 1) gyro: elbow rate = difference of the two Y rates
  angleRaw += (rateL - rateU) * dt;
  // 2) accelerometer: drift-free correction, only when usable
  if (u.accelOk && l.accelOk) {
    angleRaw += (1.0 - ALPHA) * wrap180(angleAccel - angleRaw);
    st.accelUsed++;
  }
  // 3) slow offset tracking while resting (uses the spike-free rates; skipped while a spike is in progress)
  if (BIAS_TRACKING && fabs(rateU) < BIAS_TRACK_RATE && fabs(rateL) < BIAS_TRACK_RATE &&
      fabs(u.rateY - gU) < BIAS_TRACK_RATE && fabs(l.rateY - gL) < BIAS_TRACK_RATE &&
      fabs(u.accG - 1.0) < STILL_ACC_TOL && fabs(l.accG - 1.0) < STILL_ACC_TOL) {
    biasUpper += BIAS_TRACK_GAIN * (gU - biasUpper);
    biasLower += BIAS_TRACK_GAIN * (gL - biasLower);
    st.biasUpdates++;
  }
  // 4) median against single extreme values
  med = medianPush(angleRaw);
  if (fabs(wrap180(angleRaw - med)) > SPIKE_DEG) st.spikes++;
  st.samples++;
  return true;
}

// One fusion step with the real time since the last step (max. 50 ms)
bool fuse() {
  unsigned long nowUs = micros();
  float dt = (nowUs - lastFusionUs) / 1000000.0;
  lastFusionUs = nowUs;
  if (dt > 0.05) dt = 0.05;               // after a pause do not integrate a big jump
  return sampleStep(dt);
}

// Reads at 100 Hz for durationMs (keeps the fusion running while waiting)
void keepSampling(unsigned long durationMs) {
  unsigned long t0 = millis(), next = micros();
  while (millis() - t0 < durationMs) {
    if ((long)(micros() - next) >= 0) { next += LOOP_US; fuse(); }
    else delay(1);
  }
}

// Gyro offset: average while still; repeats until the arm is still (max. 5 tries)
void measureGyroBias() {
  for (int attempt = 1; attempt <= 5; attempt++) {
    Serial.println("Keep the arm completely still: measuring gyro offset (2 s) ...");
    digitalWrite(LED_PIN, HIGH);
    double sU = 0, qU = 0, sL = 0, qL = 0; float maxAccDev = 0; int n = 0;
    unsigned long t0 = millis();
    while (millis() - t0 < BIAS_MS) {
      Reading u, l;
      if (upperOk && lowerOk && readImu(imuUpper, u) && readImu(imuLower, l)) {
        sU += u.rateY; qU += (double)u.rateY * u.rateY;
        sL += l.rateY; qL += (double)l.rateY * l.rateY;
        float dev = fabs(u.accG - 1.0) > fabs(l.accG - 1.0) ? fabs(u.accG - 1.0) : fabs(l.accG - 1.0);
        if (dev > maxAccDev) maxAccDev = dev;
        n++;
      }
      delay(10);
    }
    digitalWrite(LED_PIN, LOW);
    if (n < 50) { Serial.println("  Too few readings – check the sensors."); biasStill = false; return; }
    biasMeasured = true;
    float mU = sU / n, mL = sL / n;
    float sdU = sqrt(fabs(qU / n - mU * mU)), sdL = sqrt(fabs(qL / n - mL * mL));
    Serial.print("  offset upper/lower [deg/s]: "); Serial.print(mU, 3); Serial.print(" / "); Serial.print(mL, 3);
    Serial.print("   noise: "); Serial.print(sdU, 3); Serial.print(" / "); Serial.println(sdL, 3);
    biasUpper = mU; biasLower = mL;
    if (sdU <= STILL_GYRO_SD && sdL <= STILL_GYRO_SD && maxAccDev <= STILL_ACC_TOL) { biasStill = true; Serial.println("  Gyro offset ok."); return; }
    Serial.print("  The arm moved (attempt "); Serial.print(attempt); Serial.println(" of 5). Hold still ...");
    delay(500);
  }
  biasStill = false;
  Serial.println("  WARNING: never still – offset may be inaccurate. Type b + Enter to measure again.");
}

bool startSensors() {
  bool ok = initDevices();
  medianReset();
  fusionInit = false;
  lastFusionUs = micros();
  return ok;
}

bool measure(unsigned long durationMs, bool useAngle, float& mean, float& sd, float& minV, float& maxV, int& n) {
  unsigned long t0 = millis(), next = micros();
  double m = 0, m2 = 0;
  n = 0; minV = 1e9; maxV = -1e9;
  while (millis() - t0 < durationMs) {
    if ((long)(micros() - next) < 0) { delay(1); continue; }
    next += LOOP_US;
    if (fuse()) {
      float x = useAngle ? rawToAngle(med) : med;
      n++;
      double d = x - m;
      m += d / n;
      m2 += d * (x - m);
      if (x < minV) minV = x;
      if (x > maxV) maxV = x;
    }
  }
  mean = m;
  sd = (n > 1) ? sqrt(m2 / (n - 1)) : 0;
  return n >= (int)(durationMs * 1000UL / LOOP_US) / 2;
}

// ======================= calibration =======================
bool measurePoint(float angleDeg, float& value) {
  Serial.print(">> Calibration point "); Serial.print(angleDeg, 1); Serial.println(" deg: hold still, measuring 1 s ...");
  digitalWrite(LED_PIN, HIGH);
  keepSampling(CAL_SETTLE_MS);
  float mean, sd, minV, maxV; int n;
  bool enough = measure(CAL_MS, false, mean, sd, minV, maxV, n);
  digitalWrite(LED_PIN, LOW);
  Serial.print("   IMU angle mean "); Serial.print(mean, 2);
  Serial.print(" deg, std dev "); Serial.print(sd, 3);
  Serial.print(", range "); Serial.print(maxV - minV, 2);
  Serial.print(", n = "); Serial.println(n);
  if (!enough) { Serial.println(">> REJECTED: too few sensor values (check the sensors)."); return false; }
  if (sd > CAL_MAX_SD || (maxV - minV) > CAL_MAX_RANGE) {
    Serial.println(">> REJECTED: the arm moved. Hold still and try again.");
    return false;
  }
  value = mean;
  return true;
}

void saveAndReport() {
  if (calSave()) Serial.println("   Saved (survives power-off).");
  else Serial.println("   WARNING: saving failed – calibration only valid until power-off.");
}

bool addPoint(float angleDeg) {
  if (angleDeg < -180.0 || angleDeg > 180.0) { Serial.println(">> Angle not accepted (allowed -180 ... 180 deg)."); return false; }
  if (sensorError) { Serial.println(">> Not possible: sensor error."); return false; }
  float v;
  if (!measurePoint(angleDeg, v)) return false;
  CalData c = cal;
  bool isZero = fabs(angleDeg) < 0.01;
  int zi = zeroIndex();
  if (isZero && zi >= 0) {                        // re-zero: shift the whole table
    float shift = v - c.rawDeg[zi];
    for (int i = 0; i < c.n; i++) c.rawDeg[i] += shift;
    Serial.print("   0 deg set again: whole table shifted by "); Serial.print(shift, 2); Serial.println(" deg.");
  } else {
    int idx = -1;
    for (int i = 0; i < c.n; i++) if (fabs(c.trueDeg[i] - angleDeg) < 0.5) idx = i;
    if (idx >= 0) c.rawDeg[idx] = v;
    else {
      if (c.n >= MAX_POINTS) { Serial.println(">> REJECTED: table full (8 points). Type clear to start again."); return false; }
      int pos = c.n;
      while (pos > 0 && c.trueDeg[pos - 1] > angleDeg) { c.trueDeg[pos] = c.trueDeg[pos - 1]; c.rawDeg[pos] = c.rawDeg[pos - 1]; pos--; }
      c.trueDeg[pos] = angleDeg; c.rawDeg[pos] = v; c.n++;
    }
  }
  const char* why = "";
  if (!tableValid(c, &why)) { Serial.print(">> REJECTED: "); Serial.println(why); return false; }
  cal = c;
  resetWarning();
  Serial.print(">> Calibrated: "); Serial.print(angleDeg, 1); Serial.println(" deg.");
  printTable();
  if (!fullyCalibrated()) {
    if (zeroIndex() < 0) Serial.println("   Still missing: 0 deg (arm straight, SW38 short or Enter).");
    else Serial.println("   Still missing: a point at least 45 deg away, e.g. 90 deg (SW38 for 2 s or type 90).");
  } else Serial.println("   Warnings are ACTIVE.");
  saveAndReport();
  confirmClick(isZero ? 1 : 2);
  return true;
}

// ======================= input =======================
bool readLine(char* out, int outSize) {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      inputBuf[inputLen] = 0;
      strncpy(out, inputBuf, outSize - 1);
      out[outSize - 1] = 0;
      inputLen = 0;
      return true;
    }
    if (inputLen < (int)sizeof(inputBuf) - 1) inputBuf[inputLen++] = c;
  }
  return false;
}

bool isNumber(const char* s, float& value) {
  char* end;
  value = strtod(s, &end);
  if (end == s) return false;
  while (*end == ' ') end++;
  return *end == 0;
}

char waitAction(unsigned long timeoutMs, char* line, int lineSize) {
  unsigned long t0 = millis();
  bool down = false, longSig = false;
  unsigned long since = 0, lastS = 0;
  line[0] = 0;
  while (millis() - t0 < timeoutMs) {
    if (millis() - lastS >= LOOP_US / 1000) { lastS = millis(); fuse(); }   // keep the fusion running
    if (readLine(line, lineSize)) {
      char* p = line; while (*p == ' ') p++;
      return (*p == 'x' || *p == 'X') ? 'x' : 'E';
    }
    bool g = buttonDown();
    if (g && !down) { down = true; since = millis(); longSig = false; }
    if (g && down && !longSig && millis() - since >= LONG_MS) { longSig = true; confirmClick(1); }
    if (!g && down) {
      down = false;
      unsigned long d = millis() - since;
      if (d >= 30) return (d >= LONG_MS) ? 'L' : 'K';
    }
    delay(2);
  }
  return 0;
}

bool guidedPoint(const char* text, float angleDeg) {
  char line[24];
  for (int attempt = 1; attempt <= 3; attempt++) {
    Serial.println(text);
    Serial.println("   Then press SW38 briefly or Enter (x = abort).");
    char a = waitAction(120000, line, sizeof(line));
    if (a == 0 || a == 'x') { Serial.println(">> Guided calibration aborted."); return false; }
    if (addPoint(angleDeg)) return true;
    Serial.print("   Attempt "); Serial.print(attempt); Serial.println(" of 3.");
  }
  Serial.println(">> Guided calibration stopped after 3 attempts.");
  return false;
}

void measureCommand() {
  Serial.println();
  Serial.println("================================");
  Serial.println("  JOINTSENSE - ANGLE MEASUREMENT");
  Serial.println("================================");
  Serial.println("Measuring for 2 seconds...");
  Serial.println("Please hold the jig still.");

  keepSampling(CAL_SETTLE_MS);

  float mean, sd, minV, maxV;
  int n;

  bool enough = measure(MEASURE_MS, true,
                        mean, sd, minV, maxV, n);

  Serial.println();
  Serial.println("========== RESULTS =============");

  Serial.print("Angle mean:       ");
  Serial.print(mean, 2);
  Serial.println(" deg");

  Serial.print("Standard dev.:    ");
  Serial.print(sd, 3);
  Serial.println(" deg");

  Serial.print("Minimum angle:    ");
  Serial.print(minV, 2);
  Serial.println(" deg");

  Serial.print("Maximum angle:    ");
  Serial.print(maxV, 2);
  Serial.println(" deg");

  Serial.print("Valid samples:    ");
  Serial.println(n);

  if (!enough) {
    Serial.println("WARNING: Too few valid sensor readings!");
  }

  if (!fullyCalibrated()) {
    Serial.println("WARNING: Calibration incomplete.");
  }

  Serial.println("================================");
  Serial.println("Press ENTER to continue...");

  // Wait until the user presses Enter
  while (true) {
    if (Serial.available() > 0) {
      char c = Serial.read();
      if (c == '\n') break;
    }
    delay(10);
  }

  // Clear remaining serial input
  while (Serial.available() > 0) {
    Serial.read();
  }

  Serial.println("Ready for next measurement.");
}
void guidedCalibration() {
  Serial.println();
  Serial.println("=== Guided calibration ===");
  Serial.println("Mount both sensors exactly as they will be used. Use a goniometer.");
  if (!guidedPoint("Step 1: straighten the arm completely (0 deg) and hold it still.", 0.0)) return;
  if (!guidedPoint("Step 2: bend the elbow to exactly 90 deg and hold it still.", 90.0)) return;
  char line[24];
  while (true) {
    Serial.println("Step 3 (optional, more accuracy): hold the elbow at another known angle and type it");
    Serial.println("   (e.g. 30, 45, 60, 120, 135, -10) + Enter. Only Enter or SW38 = finished.");
    char a = waitAction(120000, line, sizeof(line));
    if (a == 0 || a == 'x' || a == 'K') break;
    float v;
    char* p = line; while (*p == ' ') p++;
    if (a == 'E' && *p == 0) break;
    if (a == 'E' && isNumber(p, v)) { addPoint(v); continue; }
    Serial.println("   Please type a number (angle in deg) or only Enter.");
  }
  Serial.println("Step 4: check – straighten the arm again, hold still, then press SW38 or Enter.");
  char a = waitAction(120000, line, sizeof(line));
  if (a == 0 || a == 'x') return;
  measureCommand();
  Serial.println("=== Calibration complete. The check value should be 0 deg +- 2 deg. ===");
}

void printHelp() {
  Serial.println("SW38: short = point 0 deg | 2 s = point 90 deg.   Serial: c = guided calibration");
  Serial.println("  <angle> + Enter = calibration point (e.g. 0, 45, 90, 135) | t = table | clear = delete calibration");
  Serial.println("  g -12 = limit | m = measure 2 s | b = gyro offset | r = raw gyro on/off | l = CSV log | s = status | h = help");
}

void printStatus() {
  Serial.print("Status: limit "); Serial.print(cal.warnLimit, 1);
  Serial.print(" deg | warnings "); Serial.print(warningsEnabled() ? "ACTIVE" : "OFF (calibrate 0 deg + 90 deg first)");
  Serial.print(" | vibration "); Serial.print(drvOk ? "ok" : "MISSING");
  Serial.println(calFromFlash ? " | calibration loaded" : "");
  Serial.print("  Sensors: upper arm 0x68 "); Serial.print(upperOk ? "ok" : "MISSING");
  Serial.print(", forearm 0x69 "); Serial.print(lowerOk ? "ok" : "MISSING");
  Serial.print(" | gyro offset upper/lower "); Serial.print(biasUpper, 3); Serial.print(" / "); Serial.print(biasLower, 3);
  Serial.println(biasStill ? " deg/s (measured still)" : " deg/s (NOT measured still -> b)");
  printTable();
  Serial.print("  Values: "); Serial.print(st.samples);
  Serial.print(" | implausible readings rejected "); Serial.print(st.implausible);
  Serial.print(" | spikes removed by median "); Serial.print(st.spikes);
  Serial.print(" | accel corrections "); Serial.print(st.accelUsed);
  Serial.print(" | offset updates "); Serial.println(st.biasUpdates);
}

void setLimit(float g) {
  if (g < -45.0 || g > 0.0) { Serial.println(">> Limit not accepted (allowed -45 ... 0 deg)."); return; }
  cal.warnLimit = g;
  resetWarning();
  Serial.print(">> New warning limit: "); Serial.print(cal.warnLimit, 1); Serial.println(" deg");
  saveAndReport();
}

void handleCommand(char* cmd) {
  while (*cmd == ' ') cmd++;
  float v;
  if (strcmp(cmd, "clear") == 0) {
    float limit = cal.warnLimit;
    calDefaults();
    cal.warnLimit = limit;
    calFromFlash = false;
    startRaw = med;
    resetWarning();
    Serial.println(">> Calibration deleted. Warnings are OFF until 0 deg + 90 deg are calibrated.");
    saveAndReport();
    return;
  }
  if (*cmd == 0) { addPoint(0.0); return; }
  if (isNumber(cmd, v)) { addPoint(v); return; }
  if (cmd[0] == 'g' || cmd[0] == 'G') { setLimit(atof(cmd + 1)); return; }
  if (cmd[0] == 'c' || cmd[0] == 'C') { guidedCalibration(); return; }
  if (cmd[0] == 't' || cmd[0] == 'T') { printTable(); return; }
  if (cmd[0] == 'm' || cmd[0] == 'M') { measureCommand(); return; }
  if (cmd[0] == 'b' || cmd[0] == 'B') { measureGyroBias(); return; }
  if (cmd[0] == 'r' || cmd[0] == 'R') { rawOut = !rawOut; csvLog = false; Serial.println(rawOut ? ">> Raw gyro output on." : ">> Raw gyro output off."); return; }
  if (cmd[0] == 'l' || cmd[0] == 'L') {
    csvLog = !csvLog; rawOut = false;
    if (csvLog) Serial.println("CSV;time_ms;angle_deg;imu_angle_deg;accel_angle_deg;warning");
    else Serial.println(">> CSV log off.");
    return;
  }
  if (cmd[0] == 's' || cmd[0] == 'S') { printStatus(); return; }
  if (cmd[0] == 'h' || cmd[0] == 'H') { printHelp(); return; }
  Serial.println("Unknown – h = help");
}

void checkSerial() {
  char line[24];
  while (readLine(line, sizeof(line))) handleCommand(line);
}

void checkButton(unsigned long now) {
  bool pressed = buttonDown();
  if (pressed && !buttonPressed) {
    buttonPressed = true;
    longSignaled = false;
    buttonSince = now;
  } else if (pressed && buttonPressed) {
    if (!longSignaled && now - buttonSince >= LONG_MS) {
      longSignaled = true;
      confirmClick(1);                            // click: release now = 90 degrees
    }
  } else if (!pressed && buttonPressed) {
    buttonPressed = false;
    unsigned long duration = now - buttonSince;
    if (duration < 30) return;
    if (duration >= LONG_MS) addPoint(90.0);
    else addPoint(0.0);
  }
}

// ======================= setup =======================
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println();
  Serial.println("JointSense IMU prototype v2 (Feather ESP32 V2) – hyperextension warning");

#if defined(NEOPIXEL_I2C_POWER)
  pinMode(NEOPIXEL_I2C_POWER, OUTPUT);       // power for the STEMMA QT port
  digitalWrite(NEOPIXEL_I2C_POWER, HIGH);
  delay(10);
#endif
  pinMode(BUTTON_PIN, INPUT);                // GPIO 38 has a pull-up on the Feather board
  pinMode(LED_PIN, OUTPUT);

  calFromFlash = calLoad();
  if (!calFromFlash) calDefaults();

  Wire.begin();
  Wire.setClock(100000);                     // 100 kHz: more robust with ~60 cm of cable
  Serial.print("I2C: upper arm 0x68 "); Serial.print(i2cPresent(ADR_UPPER) ? "found" : "MISSING");
  Serial.print(" | forearm 0x69 "); Serial.print(i2cPresent(ADR_LOWER) ? "found" : "MISSING (AD0 jumper?)");
  Serial.print(" | DRV2605L 0x5A "); Serial.println(i2cPresent(ADR_DRV) ? "found" : "MISSING");

  bool ok = startSensors();
  Serial.println(drvOk ? "DRV2605L: ok" : "NOTE: DRV2605L missing -> no vibration, warning only via the LED.");
  if (ok) {
    measureGyroBias();
    keepSampling(500);                       // let the fusion settle on the accelerometer angle
    startRaw = med;
    Serial.println("Sensors: ok");
  } else {
    sensorError = true;
    Serial.println("ERROR: MPU-6050 missing -> check QT cables / AD0 jumper. Retrying every 2 s.");
  }
  lastSample = millis();

  if (calFromFlash) {
    Serial.print("Calibration loaded ("); Serial.print(cal.n); Serial.println(" points).");
    Serial.println("Tip: after putting the sensors on again, set 0 deg once (SW38 short).");
  } else {
    Serial.println("No saved calibration. Type c + Enter for the guided calibration.");
  }
  if (!warningsEnabled()) Serial.println("Warnings are OFF until 0 deg + 90 deg are calibrated (LED blinks slowly).");
  printHelp();
  confirmClick(1);
  lastMicros = micros();
  lastFusionUs = micros();
}

// ======================= loop =======================
void loop() {
  unsigned long nowUs = micros();
  if (nowUs - lastMicros < LOOP_US) return;
  lastMicros = nowUs;
  unsigned long now = millis();

  // 1) read + fuse, detect sensor failure
  if (!sensorError && fuse()) lastSample = now;
  if (!sensorError && now - lastSample > SENSOR_TIMEOUT_MS) {
    sensorError = true;
    resetWarning();
    lastVib = now - 3000;
    Serial.println("ERROR: no plausible sensor values (QT cable loose?).");
  }

  // 2) operation
  checkSerial();
  checkButton(now);

  // 3) sensor error: LED blinks fast, re-init every 2 s
  if (sensorError) {
    if (ERROR_VIBRATION && now - lastVib >= 3000) { lastVib = now; vibrate(EFFECT_ERROR); }
    if (now - lastRestart >= 2000) {
      lastRestart = now;
      if (startSensors()) {
        Serial.println(">> Sensors deliver values again.");
        if (!biasMeasured) measureGyroBias();   // e.g. a sensor was missing at the start
        keepSampling(300);
        if (cal.n == 0) startRaw = med;
        lastSample = millis();
        sensorError = false;
      }
    }
    digitalWrite(LED_PIN, ((now / 125) % 2) ? HIGH : LOW);
    return;
  }

  // 4) angle from the calibration table (median-filtered)
  float angle = rawToAngle(med);

  // 5) warning: only when calibrated, only after WARN_ON_MS beyond the limit
  if (!warningsEnabled()) {
    resetWarning();
  } else {
    bool beyond = (angle <= cal.warnLimit) || (FLEX_WARNING && angle >= ANGLE_MAX);
    bool back = (angle >= cal.warnLimit + HYSTERESIS) && (!FLEX_WARNING || angle <= ANGLE_MAX - HYSTERESIS);
    if (!warnActive) {
      if (beyond) {
        if (!beyondTiming) { beyondTiming = true; beyondSince = now; }
        if (now - beyondSince >= WARN_ON_MS) { warnActive = true; backTiming = false; lastVib = now - VIB_PAUSE_MS; }
      } else beyondTiming = false;
    } else {
      if (back) {
        if (!backTiming) { backTiming = true; backSince = now; }
        if (now - backSince >= WARN_OFF_MS) { warnActive = false; beyondTiming = false; }
      } else backTiming = false;
    }
  }
  if (warnActive && now - lastVib >= VIB_PAUSE_MS) {
    lastVib = now;
    vibrate(EFFECT_WARN);
  }

  if (warnActive) digitalWrite(LED_PIN, HIGH);
  else if (!warningsEnabled()) digitalWrite(LED_PIN, ((now / 500) % 2) ? HIGH : LOW);
  else digitalWrite(LED_PIN, LOW);

  // 6) output
  if (csvLog) {
    Serial.print("CSV;"); Serial.print(now); Serial.print(";"); Serial.print(angle, 2);
    Serial.print(";"); Serial.print(med, 2); Serial.print(";"); Serial.print(angleAccelLast, 2);
    Serial.print(";"); Serial.println(warnActive ? 1 : 0);
  } else if (now - lastOutput >= OUTPUT_MS) {
    lastOutput = now;
    if (rawOut) {
      Serial.print("upper_gx:"); Serial.print(gxU, 1); Serial.print(",upper_gy:"); Serial.print(lastGyroU, 1);
      Serial.print(",upper_gz:"); Serial.print(gzU, 1); Serial.print(",lower_gx:"); Serial.print(gxL, 1);
      Serial.print(",lower_gy:"); Serial.print(lastGyroL, 1); Serial.print(",lower_gz:"); Serial.println(gzL, 1);
    } else {
      Serial.print("Angle:"); Serial.print(angle, 1);
      Serial.print(",Limit:"); Serial.print(cal.warnLimit, 1);
      if (FLEX_WARNING) { Serial.print(",Max:"); Serial.print(ANGLE_MAX, 0); }
      Serial.println();
    }
  }
}

