# JointSense — signed elbow angle prototype

**Experimental firmware; not hardware-validated.** Based on Team TM-38's supplied `JointSense_IMU_v2_edit_peter.ino`.

Desired convention: flexion positive, straight arm 0°, hyperextension negative. The code preserves a **continuous signed relative IMU angle** instead of wrapping the fused gyro state to ±180° on every step. The median filter also unwraps around the preceding median, avoiding erroneous averaging at the wrap boundary. Existing calibration, vibration threshold (default −10°), wiring, and serial commands are retained.

## Arduino
Open `JointSense_IMU_signed_hyperextension.ino` in Arduino IDE. Board: Adafruit Feather ESP32 V2. Libraries: Adafruit MPU6050, Adafruit Unified Sensor, Adafruit BusIO, Adafruit DRV2605. Wiring: upper MPU6050 0x68, lower 0x69, DRV2605L 0x5A; both IMUs must have matching axes (X along arm, Y elbow axis, Z away from arm).

## Validation before using the vibration warning
1. Open Serial Monitor at **115200 baud**; type `clear` to remove prior calibration.
2. Hold the arm straight, type `0` and wait for calibration.
3. Bend the elbow to a known positive angle (e.g. 90°), type `90`.
4. Type `l` to enable CSV diagnostics; compare the **calibrated angle**, **raw fused angle**, and **accelerometer angle** while slowly crossing straight into a *small comfortable* hyperextension. Do not force the joint.
5. If the **raw fused angle** does not change sign across straight, verify sensor axes, sign of gyroscope Y, mounting, and that the movement is within the measured plane. Software cannot fix a misaligned sensor orientation. If raw is correct but calibrated is not, clear and redo calibration; optionally add a verified negative reference point.
6. Set limit with `g -10` (or another clinically appropriate value). Do not use as a safety device until validated against an independent reference (e.g. goniometer).

This revision does **not** establish that a physical elbow's 0° crossing is detectable with the current mounting; it must be tested. The existing complementary filter uses a gravity-based Y-axis tilt and cannot independently resolve all 3D elbow orientations.