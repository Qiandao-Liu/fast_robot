/*
 * IMU Test Sketch for SparkFun ICM-20948
 *
 * Features:
 *   - Pitch/Roll from accelerometer (atan2)
 *   - Low-pass filter on accelerometer pitch/roll
 *   - Gyroscope integration for pitch/roll/yaw
 *   - Complementary filter (accel + gyro fusion)
 *   - CSV serial output for easy Python/Jupyter parsing
 *   - LED blinks 3x on startup
 *
 * Hardware: Artemis board + ICM-20948 via QWIIC connector
 * Library:  SparkFun 9DOF IMU Breakout - ICM 20948 - Arduino Library
 *
 * Serial output format (CSV):
 *   time_ms, ax, ay, az, pitch_a, roll_a, pitch_a_lpf, roll_a_lpf,
 *   gx, gy, gz, pitch_g, roll_g, yaw_g, pitch_comp, roll_comp
 */

#include "ICM_20948.h"
#include <math.h>

// AD0_VAL: last bit of ICM-20948 I2C address.
// ADR jumper open (default) → AD0=1 → addr 0x69
// ADR jumper closed         → AD0=0 → addr 0x68
#define AD0_VAL 1

#define LED_PIN LED_BUILTIN

ICM_20948_I2C myICM;

// ---- Timing ----
unsigned long lastTime = 0;
float dt = 0.0;

// ---- Accelerometer pitch/roll ----
float pitch_a = 0.0;
float roll_a  = 0.0;

// ---- Low-pass filtered accelerometer pitch/roll ----
float pitch_a_lpf = 0.0;
float roll_a_lpf  = 0.0;
// LPF alpha: adjust based on your cutoff frequency analysis
// alpha = dt / (dt + 1/(2*pi*fc))
// For fc ~5 Hz and dt ~0.01s: alpha ≈ 0.24
// Tune after FFT analysis of your data.
float alpha_lpf = 0.2;

// ---- Gyroscope integrated angles ----
float pitch_g = 0.0;
float roll_g  = 0.0;
float yaw_g   = 0.0;

// ---- Complementary filter ----
// alpha_comp closer to 0 → trust gyro more (less drift-susceptible? no, less noise but more drift)
// alpha_comp closer to 1 → trust accel more (noisy but no drift)
// Typical: 0.02 ~ 0.1
float alpha_comp  = 0.05;
float pitch_comp  = 0.0;
float roll_comp   = 0.0;

// ---- Two-point calibration offsets (fill in after calibration) ----
// Measure pitch/roll at known -90 and +90, compute scale/offset.
// pitch_corrected = pitch_scale * pitch_raw + pitch_offset
float pitch_scale  = 1.0;
float pitch_offset = 0.0;
float roll_scale   = 1.0;
float roll_offset  = 0.0;

void blinkLED(int count, int delayMs) {
    for (int i = 0; i < count; i++) {
        digitalWrite(LED_PIN, HIGH);
        delay(delayMs);
        digitalWrite(LED_PIN, LOW);
        delay(delayMs);
    }
}

void setup() {
    pinMode(LED_PIN, OUTPUT);
    blinkLED(3, 500); // Visual indicator: 3 slow blinks

    Serial.begin(115200);
    while (!Serial);

    Serial.println("ICM-20948 IMU Test — Full Pipeline");
    Serial.println("==================================");

    Wire.begin();
    Wire.setClock(400000);

    bool initialized = false;
    while (!initialized) {
        myICM.begin(Wire, AD0_VAL);
        Serial.print("Init: ");
        Serial.println(myICM.statusString());
        if (myICM.status != ICM_20948_Stat_Ok) {
            Serial.println("ICM-20948 not detected. Retrying...");
            delay(500);
        } else {
            initialized = true;
        }
    }

    Serial.println("ICM-20948 connected!");
    Serial.println();

    // CSV header
    Serial.println("time_ms,ax,ay,az,pitch_a,roll_a,pitch_a_lpf,roll_a_lpf,"
                   "gx,gy,gz,pitch_g,roll_g,yaw_g,pitch_comp,roll_comp");

    lastTime = millis();
}

void loop() {
    if (!myICM.dataReady()) return;

    myICM.getAGMT();

    // ---- Timing ----
    unsigned long now = millis();
    dt = (now - lastTime) / 1000.0; // seconds
    lastTime = now;
    if (dt <= 0) dt = 0.001; // guard

    // ---- Raw sensor values ----
    float ax = myICM.accX(); // mg
    float ay = myICM.accY();
    float az = myICM.accZ();
    float gx = myICM.gyrX(); // dps
    float gy = myICM.gyrY();
    float gz = myICM.gyrZ();

    // ============================================================
    // 1. Accelerometer pitch & roll (degrees)
    //    pitch = atan2(ax, sqrt(ay^2 + az^2))
    //    roll  = atan2(ay, sqrt(ax^2 + az^2))
    // ============================================================
    pitch_a = atan2(ax, sqrt(ay * ay + az * az)) * 180.0 / M_PI;
    roll_a  = atan2(ay, sqrt(ax * ax + az * az)) * 180.0 / M_PI;

    // Apply two-point calibration (identity by default)
    pitch_a = pitch_scale * pitch_a + pitch_offset;
    roll_a  = roll_scale  * roll_a  + roll_offset;

    // ============================================================
    // 2. Low-pass filter on accelerometer pitch & roll
    //    y[n] = alpha * x[n] + (1 - alpha) * y[n-1]
    //    alpha = dt / (dt + 1/(2*pi*fc))
    //    You can dynamically compute alpha or use a fixed value.
    // ============================================================
    pitch_a_lpf = alpha_lpf * pitch_a + (1.0 - alpha_lpf) * pitch_a_lpf;
    roll_a_lpf  = alpha_lpf * roll_a  + (1.0 - alpha_lpf) * roll_a_lpf;

    // ============================================================
    // 3. Gyroscope integration (degrees)
    //    angle += angular_rate * dt
    //    Subject to drift over time.
    // ============================================================
    pitch_g += gx * dt;
    roll_g  += gy * dt;
    yaw_g   += gz * dt;

    // ============================================================
    // 4. Complementary filter
    //    Fuses gyro (fast, no noise, drifts) with accel (noisy, no drift)
    //    angle = (1 - alpha) * (angle_prev + gyro * dt) + alpha * accel_angle
    // ============================================================
    pitch_comp = (1.0 - alpha_comp) * (pitch_comp + gx * dt) + alpha_comp * pitch_a;
    roll_comp  = (1.0 - alpha_comp) * (roll_comp  + gy * dt) + alpha_comp * roll_a;

    // ============================================================
    // CSV output
    // ============================================================
    Serial.print(now);            Serial.print(",");
    Serial.print(ax, 2);         Serial.print(",");
    Serial.print(ay, 2);         Serial.print(",");
    Serial.print(az, 2);         Serial.print(",");
    Serial.print(pitch_a, 2);    Serial.print(",");
    Serial.print(roll_a, 2);     Serial.print(",");
    Serial.print(pitch_a_lpf, 2); Serial.print(",");
    Serial.print(roll_a_lpf, 2); Serial.print(",");
    Serial.print(gx, 2);        Serial.print(",");
    Serial.print(gy, 2);        Serial.print(",");
    Serial.print(gz, 2);        Serial.print(",");
    Serial.print(pitch_g, 2);   Serial.print(",");
    Serial.print(roll_g, 2);    Serial.print(",");
    Serial.print(yaw_g, 2);     Serial.print(",");
    Serial.print(pitch_comp, 2); Serial.print(",");
    Serial.println(roll_comp, 2);

    delay(5); // ~200 Hz sampling; adjust as needed
}
