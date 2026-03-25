
#include "BLECStringCharacteristic.h"
#include "EString.h"
#include "RobotCommand.h"
#include <ArduinoBLE.h>
#include "ICM_20948.h"
#include <math.h>
#include <SparkFun_VL53L1X.h>

//////////// BLE UUIDs ////////////
#define BLE_UUID_TEST_SERVICE "d1e59283-ea64-46d2-9619-feda9179e362"

#define BLE_UUID_RX_STRING "9750f60b-9c9c-4158-b620-02ec9521cd99"

#define BLE_UUID_TX_FLOAT "27616294-3063-4ecc-b60b-3470ddef2938"
#define BLE_UUID_TX_STRING "f235a225-6735-4d73-94cb-ee5dfce9ba83"
//////////// BLE UUIDs ////////////

//////////// Motor Setup ////////////
#define L_FWD  3    // Left  motor forward  (AIN1)
#define L_BWD  14   // Left  motor backward (AIN2)
#define R_FWD  16   // Right motor forward  (BIN1)
#define R_BWD  15   // Right motor backward (BIN2)

float motorCalFactor = 1.0;  // Multiply right-motor speed to compensate drift

// Non-blocking motor state machine
enum MotorMode { MOT_IDLE, MOT_FWD, MOT_BWD, MOT_LEFT, MOT_RIGHT, MOT_TURN_ANGLE };
MotorMode motorMode  = MOT_IDLE;
unsigned long motorEndTime = 0;
bool motorTimed = false;

// Angle-controlled turn state
float turnTargetDeg      = 90.0f;
float turnAccumDeg       = 0.0f;
unsigned long lastTurnGyroTime = 0;
//////////// Motor Setup ////////////

//////////// ToF Setup ////////////
// Both sensors now have XSHUT wires for reliable hot-restart via hardware reset.
// Sensor 1 XSHUT → A1 (GPIO 1); sensor 2 XSHUT → A0 (GPIO 0).
#define SHUTDOWN_PIN_1  1  // A1 → sensor 1 XSHUT
#define SHUTDOWN_PIN    0  // A0 → sensor 2 XSHUT
#define TOF1_ADDR     0x30 // Sensor 1 remapped here; sensor 2 stays at 0x29

SFEVL53L1X tofSensor1;
SFEVL53L1X tofSensor2;
bool tof1Ready = false;
bool tof2Ready = false;

// ToF data arrays
#define MAX_TOF_SIZE 1000
unsigned long tofTimeStamps[MAX_TOF_SIZE];
int16_t       tofDist1[MAX_TOF_SIZE];   // mm, sensor 1
int16_t       tofDist2[MAX_TOF_SIZE];   // mm, sensor 2
int  tofIndex     = 0;
bool tofArrayFull = false;
bool collectingTOF = false;
//////////// ToF Setup ////////////

//////////// IMU Setup ////////////
// AD0_VAL: 1 if ADR jumper is open (default), 0 if closed
#define AD0_VAL 1
ICM_20948_I2C myICM;
bool imuInitialized = false;

// Filter parameters
float alpha_lpf = 0.2;      // Low-pass filter alpha
float alpha_comp = 0.05;    // Complementary filter alpha

// Filtered values (persistent across readings)
float pitch_a_lpf = 0.0, roll_a_lpf = 0.0;
float pitch_g = 0.0, roll_g = 0.0, yaw_g = 0.0;
float pitch_comp = 0.0, roll_comp = 0.0;
unsigned long lastIMUTime = 0;
//////////// IMU Setup ////////////

//////////// Global Variables ////////////
BLEService testService(BLE_UUID_TEST_SERVICE);

BLECStringCharacteristic rx_characteristic_string(BLE_UUID_RX_STRING, BLEWrite, MAX_MSG_SIZE);

BLEFloatCharacteristic tx_characteristic_float(BLE_UUID_TX_FLOAT, BLERead | BLENotify);
BLECStringCharacteristic tx_characteristic_string(BLE_UUID_TX_STRING, BLERead | BLENotify, MAX_MSG_SIZE);

// RX
RobotCommand robot_cmd(":|");

// TX
EString tx_estring_value;
float tx_float_value = 0.0;

long interval = 500;
static long previousMillis = 0;
unsigned long currentMillis = 0;

// Arrays for storing timestamps and temperature data
#define MAX_DATA_SIZE 1000
unsigned long timeStamps[MAX_DATA_SIZE];
float tempReadings[MAX_DATA_SIZE];
int dataIndex = 0;
bool arrayFull = false;
bool collectingData = false;  // Flag to control data collection
unsigned long lastSampleTime = 0;
int sampleInterval = 10;  // Sample every 10ms for fast collection

// IMU data arrays - for 5+ seconds at ~100Hz = ~500 samples, use 1000 for safety
#define MAX_IMU_SIZE 2000
unsigned long imuTimeStamps[MAX_IMU_SIZE];
float imuAx[MAX_IMU_SIZE], imuAy[MAX_IMU_SIZE], imuAz[MAX_IMU_SIZE];
float imuGx[MAX_IMU_SIZE], imuGy[MAX_IMU_SIZE], imuGz[MAX_IMU_SIZE];
float imuPitchA[MAX_IMU_SIZE], imuRollA[MAX_IMU_SIZE];
float imuPitchALpf[MAX_IMU_SIZE], imuRollALpf[MAX_IMU_SIZE];
float imuPitchG[MAX_IMU_SIZE], imuRollG[MAX_IMU_SIZE], imuYawG[MAX_IMU_SIZE];
float imuPitchComp[MAX_IMU_SIZE], imuRollComp[MAX_IMU_SIZE];
int imuIndex = 0;
bool imuArrayFull = false;
bool collectingIMU = false;
//////////// Global Variables ////////////

enum CommandTypes
{
    PING,
    SEND_TWO_INTS,
    SEND_THREE_FLOATS,
    ECHO,
    DANCE,
    SET_VEL,
    GET_TIME_MILLIS,
    SEND_TIME_DATA,
    GET_TEMP_READINGS,
    START_RECORDING,
    STOP_RECORDING,
    // IMU Commands
    GET_IMU_DATA,           // 11: Get single IMU reading
    START_IMU_RECORDING,    // 12: Start recording IMU data
    STOP_IMU_RECORDING,     // 13: Stop recording IMU data
    SEND_IMU_DATA,          // 14: Send all recorded IMU data
    SET_IMU_PARAMS,         // 15: Set filter parameters (alpha_lpf, alpha_comp)
    RESET_IMU_ANGLES,       // 16: Reset integrated gyro angles
    // ToF Commands
    GET_TOF_DATA,           // 17: Single reading from both sensors → "dist1|dist2"
    START_TOF_RECORDING,    // 18: Start combined ToF+IMU recording
    STOP_TOF_RECORDING,     // 19: Stop combined recording
    SEND_TOF_DATA,          // 20: Send stored ToF data (T|ts|d1|d2 per sample)
    SET_TOF_MODE,           // 21: Set distance mode: 0=Short, 1=Long
    // Motor Commands
    MOTOR_FORWARD,          // 22: Forward  - args: speed [duration_ms]
    MOTOR_BACKWARD,         // 23: Backward - args: speed [duration_ms]
    MOTOR_STOP,             // 24: Stop all motors immediately
    MOTOR_TURN_LEFT,        // 25: On-axis left  - args: speed [duration_ms]
    MOTOR_TURN_RIGHT,       // 26: On-axis right - args: speed [duration_ms]
    SET_MOTOR_CAL,          // 27: Set right-motor calibration factor (float)
    MOTOR_TURN_ANGLE,       // 28: Gyro-controlled turn - args: dir(0=L,1=R) [angle_deg]
};

//////////// Motor Helpers ////////////
void motorsForward(int speed) {
    int rSpeed = constrain((int)(speed * motorCalFactor), 0, 255);
    analogWrite(L_FWD, speed);  analogWrite(L_BWD, 0);
    analogWrite(R_FWD, rSpeed); analogWrite(R_BWD, 0);
}
void motorsBackward(int speed) {
    int rSpeed = constrain((int)(speed * motorCalFactor), 0, 255);
    analogWrite(L_BWD, speed);  analogWrite(L_FWD, 0);
    analogWrite(R_BWD, rSpeed); analogWrite(R_FWD, 0);
}
void motorsStop() {
    analogWrite(L_FWD, 0); analogWrite(L_BWD, 0);
    analogWrite(R_FWD, 0); analogWrite(R_BWD, 0);
}
void motorsTurnLeft(int speed) {
    // Left backward, right forward → spin left in place
    analogWrite(L_BWD, speed); analogWrite(L_FWD, 0);
    analogWrite(R_FWD, speed); analogWrite(R_BWD, 0);
}
void motorsTurnRight(int speed) {
    // Left forward, right backward → spin right in place
    analogWrite(L_FWD, speed); analogWrite(L_BWD, 0);
    analogWrite(R_BWD, speed); analogWrite(R_FWD, 0);
}
void applyMotorMode(MotorMode mode, int speed) {
    switch (mode) {
        case MOT_FWD:   motorsForward(speed);   break;
        case MOT_BWD:   motorsBackward(speed);  break;
        case MOT_LEFT:  motorsTurnLeft(speed);  break;
        case MOT_RIGHT: motorsTurnRight(speed); break;
        default:        motorsStop();           break;
    }
}

/*
 * Non-blocking motor state machine — call from loop() every iteration.
 * Handles timed moves.
 */
void
handle_motors()
{
    if (motorMode == MOT_IDLE) return;

    // Gyro-controlled angle turn
    if (motorMode == MOT_TURN_ANGLE) {
        if (!imuInitialized || !myICM.dataReady()) return;
        myICM.getAGMT();
        unsigned long now = millis();
        float dt = (now - lastTurnGyroTime) / 1000.0f;
        if (dt < 0.0002f) return;
        lastTurnGyroTime = now;
        turnAccumDeg += myICM.gyrZ() * dt;
        if (fabs(turnAccumDeg) >= turnTargetDeg) {
            motorsStop();
            motorMode = MOT_IDLE;
            tx_estring_value.clear();
            tx_estring_value.append("TURN_DONE|");
            tx_estring_value.append(turnAccumDeg);
            tx_characteristic_string.writeValue(tx_estring_value.c_str());
            Serial.print("Turn done, angle="); Serial.println(turnAccumDeg);
        }
        return;
    }

    // Timed single move
    if (motorTimed && millis() >= motorEndTime) {
        motorsStop();
        motorMode = MOT_IDLE;
    }
}
//////////// Motor Helpers ////////////

void
handle_command()
{
    // Set the command string from the characteristic value
    robot_cmd.set_cmd_string(rx_characteristic_string.value(),
                             rx_characteristic_string.valueLength());

    bool success;
    int cmd_type = -1;

    // Get robot command type (an integer)
    /* NOTE: THIS SHOULD ALWAYS BE CALLED BEFORE get_next_value()
     * since it uses strtok internally (refer RobotCommand.h and 
     * https://www.cplusplus.com/reference/cstring/strtok/)
     */
    success = robot_cmd.get_command_type(cmd_type);

    // Check if the last tokenization was successful and return if failed
    if (!success) {
        return;
    }

    // Handle the command type accordingly
    switch (cmd_type) {
        /*
         * Write "PONG" on the GATT characteristic BLE_UUID_TX_STRING
         */
        case PING:
            tx_estring_value.clear();
            tx_estring_value.append("PONG");
            tx_characteristic_string.writeValue(tx_estring_value.c_str());

            Serial.print("Sent back: ");
            Serial.println(tx_estring_value.c_str());

            break;
        /*
         * Extract two integers from the command string
         */
        case SEND_TWO_INTS:
        {
            int int_a, int_b;

            // Extract the next value from the command string as an integer
            success = robot_cmd.get_next_value(int_a);
            if (!success)
                return;

            // Extract the next value from the command string as an integer
            success = robot_cmd.get_next_value(int_b);
            if (!success)
                return;

            Serial.print("Two Integers: ");
            Serial.print(int_a);
            Serial.print(", ");
            Serial.println(int_b);

            break;
        }
        /*
         * Extract three floats from the command string
         */
        case SEND_THREE_FLOATS:
        {
            float float_a, float_b, float_c;

            // Extract the next value from the command string as a float
            success = robot_cmd.get_next_value(float_a);
            if (!success)
                return;

            // Extract the next value from the command string as a float
            success = robot_cmd.get_next_value(float_b);
            if (!success)
                return;

            // Extract the next value from the command string as a float
            success = robot_cmd.get_next_value(float_c);
            if (!success)
                return;

            Serial.print("Three Floats: ");
            Serial.print(float_a);
            Serial.print(", ");
            Serial.print(float_b);
            Serial.print(", ");
            Serial.println(float_c);

            break;
        }
        /*
         * Add a prefix and postfix to the string value extracted from the command string
         */
        case ECHO:
        {
            char char_arr[MAX_MSG_SIZE];

            // Extract the next value from the command string as a character array
            success = robot_cmd.get_next_value(char_arr);
            if (!success)
                return;

            // Create augmented string with prefix and postfix
            tx_estring_value.clear();
            tx_estring_value.append("Robot says -> ");
            tx_estring_value.append(char_arr);
            tx_estring_value.append(" :)");
            tx_characteristic_string.writeValue(tx_estring_value.c_str());

            Serial.print("Sent back: ");
            Serial.println(tx_estring_value.c_str());

            break;
        }
        /*
         * DANCE
         */
        case DANCE:
            Serial.println("Look Ma, I'm Dancin'!");

            break;
        
        /*
         * SET_VEL
         */
        case SET_VEL:

            break;

        /*
         * GET_TIME_MILLIS - Task 3
         * Reply with current time in milliseconds
         */
        case GET_TIME_MILLIS:
            tx_estring_value.clear();
            tx_estring_value.append("T:");
            tx_estring_value.append((int)millis());
            tx_characteristic_string.writeValue(tx_estring_value.c_str());

            Serial.print("Sent time: ");
            Serial.println(tx_estring_value.c_str());

            break;

        /*
         * SEND_TIME_DATA - Task 6
         * Send all stored timestamps
         */
        case SEND_TIME_DATA:
        {
            int limit = arrayFull ? MAX_DATA_SIZE : dataIndex;
            for (int i = 0; i < limit; i++) {
                tx_estring_value.clear();
                tx_estring_value.append("T:");
                tx_estring_value.append((int)timeStamps[i]);
                tx_characteristic_string.writeValue(tx_estring_value.c_str());
                delay(10); // Small delay to ensure data is sent
            }

            Serial.print("Sent ");
            Serial.print(limit);
            Serial.println(" timestamps");

            // Reset arrays
            dataIndex = 0;
            arrayFull = false;

            break;
        }

        /*
         * GET_TEMP_READINGS - Task 7
         * Send all stored temperature readings with timestamps
         */
        case GET_TEMP_READINGS:
        {
            int tempLimit = arrayFull ? MAX_DATA_SIZE : dataIndex;
            for (int i = 0; i < tempLimit; i++) {
                tx_estring_value.clear();
                tx_estring_value.append("T:");
                tx_estring_value.append((int)timeStamps[i]);
                tx_estring_value.append("|C:");
                tx_estring_value.append(tempReadings[i]);
                tx_characteristic_string.writeValue(tx_estring_value.c_str());
                delay(10); // Small delay to ensure data is sent
            }

            Serial.print("Sent ");
            Serial.print(tempLimit);
            Serial.println(" temperature readings");

            // Reset arrays
            dataIndex = 0;
            arrayFull = false;

            break;
        }

        /*
         * START_RECORDING
         * Start collecting timestamp and temperature data
         */
        case START_RECORDING:
            collectingData = true;
            dataIndex = 0;
            arrayFull = false;
            Serial.println("Started data recording");

            tx_estring_value.clear();
            tx_estring_value.append("Recording started");
            tx_characteristic_string.writeValue(tx_estring_value.c_str());

            break;

        /*
         * STOP_RECORDING
         * Stop collecting data
         */
        case STOP_RECORDING:
            collectingData = false;
            Serial.print("Stopped data recording. Collected ");
            Serial.print(dataIndex);
            Serial.println(" samples");

            tx_estring_value.clear();
            tx_estring_value.append("Recording stopped. Samples: ");
            tx_estring_value.append(dataIndex);
            tx_characteristic_string.writeValue(tx_estring_value.c_str());

            break;

        /*
         * GET_IMU_DATA - Get single IMU reading
         * Returns: time,ax,ay,az,pitch_a,roll_a,pitch_lpf,roll_lpf,gx,gy,gz,pitch_g,roll_g,yaw_g,pitch_comp,roll_comp
         */
        case GET_IMU_DATA:
        {
            if (!imuInitialized || !myICM.dataReady()) {
                tx_estring_value.clear();
                tx_estring_value.append("IMU_ERROR");
                tx_characteristic_string.writeValue(tx_estring_value.c_str());
                break;
            }

            myICM.getAGMT();
            unsigned long now = millis();
            float dt = (now - lastIMUTime) / 1000.0;
            if (dt <= 0) dt = 0.001;
            lastIMUTime = now;

            float ax = myICM.accX();
            float ay = myICM.accY();
            float az = myICM.accZ();
            float gx = myICM.gyrX();
            float gy = myICM.gyrY();
            float gz = myICM.gyrZ();

            float pitch_a = atan2(ax, sqrt(ay*ay + az*az)) * 180.0 / M_PI;
            float roll_a = atan2(ay, sqrt(ax*ax + az*az)) * 180.0 / M_PI;

            pitch_a_lpf = alpha_lpf * pitch_a + (1.0 - alpha_lpf) * pitch_a_lpf;
            roll_a_lpf = alpha_lpf * roll_a + (1.0 - alpha_lpf) * roll_a_lpf;

            pitch_g += gx * dt;
            roll_g += gy * dt;
            yaw_g += gz * dt;

            pitch_comp = (1.0 - alpha_comp) * (pitch_comp + gx * dt) + alpha_comp * pitch_a;
            roll_comp = (1.0 - alpha_comp) * (roll_comp + gy * dt) + alpha_comp * roll_a;

            // Send compact format: T|ax|ay|az|pa|ra|plpf|rlpf|gx|gy|gz|pg|rg|yg|pc|rc
            tx_estring_value.clear();
            tx_estring_value.append((int)now);
            tx_estring_value.append("|");
            tx_estring_value.append(pitch_a);
            tx_estring_value.append("|");
            tx_estring_value.append(roll_a);
            tx_estring_value.append("|");
            tx_estring_value.append(pitch_a_lpf);
            tx_estring_value.append("|");
            tx_estring_value.append(roll_a_lpf);
            tx_estring_value.append("|");
            tx_estring_value.append(pitch_comp);
            tx_estring_value.append("|");
            tx_estring_value.append(roll_comp);
            tx_characteristic_string.writeValue(tx_estring_value.c_str());
            break;
        }

        /*
         * START_IMU_RECORDING - Start collecting IMU data at max speed
         */
        case START_IMU_RECORDING:
        {
            collectingIMU = true;
            imuIndex = 0;
            imuArrayFull = false;
            // Reset filter states
            pitch_a_lpf = 0; roll_a_lpf = 0;
            pitch_g = 0; roll_g = 0; yaw_g = 0;
            pitch_comp = 0; roll_comp = 0;
            lastIMUTime = millis();

            Serial.println("Started IMU recording");
            tx_estring_value.clear();
            tx_estring_value.append("IMU_REC_START");
            tx_characteristic_string.writeValue(tx_estring_value.c_str());
            break;
        }

        /*
         * STOP_IMU_RECORDING - Stop collecting IMU data
         */
        case STOP_IMU_RECORDING:
        {
            collectingIMU = false;
            Serial.print("Stopped IMU recording. Samples: ");
            Serial.println(imuIndex);

            tx_estring_value.clear();
            tx_estring_value.append("IMU_REC_STOP|");
            tx_estring_value.append(imuIndex);
            tx_characteristic_string.writeValue(tx_estring_value.c_str());
            break;
        }

        /*
         * SEND_IMU_DATA - Send all recorded IMU data
         * Compact format: T|pa|ra|pc|rc (5 fields only for reliable transfer)
         */
        case SEND_IMU_DATA:
        {
            int limit = imuArrayFull ? MAX_IMU_SIZE : imuIndex;
            int step = 3;  // Send every 3rd sample (~100Hz effective)
            int sent = 0;

            Serial.print("Sending ");
            Serial.print(limit / step);
            Serial.println(" IMU samples (downsampled)...");

            for (int i = 0; i < limit; i += step) {
                tx_estring_value.clear();
                tx_estring_value.append((int)imuTimeStamps[i]);
                tx_estring_value.append("|");
                tx_estring_value.append(imuPitchA[i]);
                tx_estring_value.append("|");
                tx_estring_value.append(imuRollA[i]);
                tx_estring_value.append("|");
                tx_estring_value.append(imuPitchComp[i]);
                tx_estring_value.append("|");
                tx_estring_value.append(imuRollComp[i]);
                tx_characteristic_string.writeValue(tx_estring_value.c_str());
                delay(30);  // Longer delay for reliable BLE
                sent++;
            }

            // Send end marker
            tx_estring_value.clear();
            tx_estring_value.append("IMU_DATA_END|");
            tx_estring_value.append(sent);
            tx_characteristic_string.writeValue(tx_estring_value.c_str());

            Serial.println("IMU data sent");
            break;
        }

        /*
         * SET_IMU_PARAMS - Set filter parameters
         * Format: alpha_lpf|alpha_comp
         */
        case SET_IMU_PARAMS:
        {
            float new_alpha_lpf, new_alpha_comp;
            if (robot_cmd.get_next_value(new_alpha_lpf) &&
                robot_cmd.get_next_value(new_alpha_comp)) {
                alpha_lpf = new_alpha_lpf;
                alpha_comp = new_alpha_comp;
                Serial.print("Set alpha_lpf=");
                Serial.print(alpha_lpf);
                Serial.print(", alpha_comp=");
                Serial.println(alpha_comp);

                tx_estring_value.clear();
                tx_estring_value.append("PARAMS_SET|");
                tx_estring_value.append(alpha_lpf);
                tx_estring_value.append("|");
                tx_estring_value.append(alpha_comp);
                tx_characteristic_string.writeValue(tx_estring_value.c_str());
            }
            break;
        }

        /*
         * RESET_IMU_ANGLES - Reset integrated gyro angles to zero
         */
        case RESET_IMU_ANGLES:
        {
            pitch_g = 0; roll_g = 0; yaw_g = 0;
            pitch_comp = 0; roll_comp = 0;
            pitch_a_lpf = 0; roll_a_lpf = 0;
            lastIMUTime = millis();

            tx_estring_value.clear();
            tx_estring_value.append("ANGLES_RESET");
            tx_characteristic_string.writeValue(tx_estring_value.c_str());
            Serial.println("IMU angles reset");
            break;
        }

        /*
         * GET_TOF_DATA - Return single reading from both ToF sensors
         * Reply format: "dist1|dist2"  (mm)  or "TOF_ERROR"
         */
        case GET_TOF_DATA:
        {
            if (!tof1Ready && !tof2Ready) {
                tx_estring_value.clear();
                tx_estring_value.append("TOF_ERROR");
                tx_characteristic_string.writeValue(tx_estring_value.c_str());
                break;
            }

            // Wait up to 100 ms for a fresh reading
            unsigned long t0 = millis();
            while (!tofSensor1.checkForDataReady() && (millis() - t0 < 100));
            int d1 = tof1Ready ? (int)tofSensor1.getDistance() : -1;
            if (tof1Ready) tofSensor1.clearInterrupt();

            t0 = millis();
            while (!tofSensor2.checkForDataReady() && (millis() - t0 < 100));
            int d2 = tof2Ready ? (int)tofSensor2.getDistance() : -1;
            if (tof2Ready) tofSensor2.clearInterrupt();

            tx_estring_value.clear();
            tx_estring_value.append(d1);
            tx_estring_value.append("|");
            tx_estring_value.append(d2);
            tx_characteristic_string.writeValue(tx_estring_value.c_str());
            break;
        }

        /*
         * START_TOF_RECORDING - Begin combined ToF + IMU recording
         */
        case START_TOF_RECORDING:
        {
            collectingTOF = true;
            collectingIMU = true;
            tofIndex = 0;
            tofArrayFull = false;
            imuIndex = 0;
            imuArrayFull = false;
            // Reset filter states
            pitch_a_lpf = 0; roll_a_lpf = 0;
            pitch_g = 0; roll_g = 0; yaw_g = 0;
            pitch_comp = 0; roll_comp = 0;
            lastIMUTime = millis();

            Serial.println("Started ToF+IMU recording");
            tx_estring_value.clear();
            tx_estring_value.append("TOF_REC_START");
            tx_characteristic_string.writeValue(tx_estring_value.c_str());
            break;
        }

        /*
         * STOP_TOF_RECORDING - Stop combined recording
         */
        case STOP_TOF_RECORDING:
        {
            collectingTOF = false;
            collectingIMU = false;
            Serial.print("Stopped. ToF samples: ");
            Serial.print(tofIndex);
            Serial.print("  IMU samples: ");
            Serial.println(imuIndex);

            tx_estring_value.clear();
            tx_estring_value.append("TOF_REC_STOP|");
            tx_estring_value.append(tofIndex);
            tx_estring_value.append("|");
            tx_estring_value.append(imuIndex);
            tx_characteristic_string.writeValue(tx_estring_value.c_str());
            break;
        }

        /*
         * SEND_TOF_DATA - Send stored ToF data
         * Format per sample: "T|timestamp|dist1|dist2"
         */
        case SEND_TOF_DATA:
        {
            int limit = tofArrayFull ? MAX_TOF_SIZE : tofIndex;
            Serial.print("Sending "); Serial.print(limit); Serial.println(" ToF samples");

            for (int i = 0; i < limit; i++) {
                tx_estring_value.clear();
                tx_estring_value.append("T|");
                tx_estring_value.append((int)tofTimeStamps[i]);
                tx_estring_value.append("|");
                tx_estring_value.append((int)tofDist1[i]);
                tx_estring_value.append("|");
                tx_estring_value.append((int)tofDist2[i]);
                tx_characteristic_string.writeValue(tx_estring_value.c_str());
                delay(10);
            }

            tx_estring_value.clear();
            tx_estring_value.append("TOF_END|");
            tx_estring_value.append(limit);
            tx_characteristic_string.writeValue(tx_estring_value.c_str());
            Serial.println("ToF data sent");
            break;
        }

        /*
         * SET_TOF_MODE - Set distance mode on both sensors
         * Argument: 0 = Short (~1.3 m), 1 = Long (~4 m, default)
         */
        case SET_TOF_MODE:
        {
            int mode;
            if (!robot_cmd.get_next_value(mode)) break;

            // MUST stop ranging before changing mode.
            // Calling setDistanceMode() during active ranging leaves internal
            // timing and algorithm registers in an inconsistent state, which
            // causes a systematic ~130 mm offset in all subsequent readings.
            if (tof1Ready) {
                tofSensor1.stopRanging();
                if (mode == 0) tofSensor1.setDistanceModeShort();
                else           tofSensor1.setDistanceModeLong();
                tofSensor1.clearInterrupt();
                tofSensor1.startRanging();
            }
            if (tof2Ready) {
                tofSensor2.stopRanging();
                if (mode == 0) tofSensor2.setDistanceModeShort();
                else           tofSensor2.setDistanceModeLong();
                tofSensor2.clearInterrupt();
                tofSensor2.startRanging();
            }

            tx_estring_value.clear();
            tx_estring_value.append("TOF_MODE|");
            tx_estring_value.append(mode == 0 ? "SHORT" : "LONG");
            tx_characteristic_string.writeValue(tx_estring_value.c_str());
            Serial.print("ToF mode set to: "); Serial.println(mode == 0 ? "SHORT" : "LONG");
            break;
        }

        /*
         * MOTOR_FORWARD - Drive forward
         * Args: speed [duration_ms]  (duration 0 = run until MOTOR_STOP)
         */
        case MOTOR_FORWARD:
        {
            int speed, duration;
            if (!robot_cmd.get_next_value(speed)) break;
            if (!robot_cmd.get_next_value(duration)) duration = 0;
            motorsForward(speed);
            motorMode = MOT_FWD;
            if (duration > 0) { motorEndTime = millis() + duration; motorTimed = true; }
            else               { motorTimed = false; }
            tx_estring_value.clear();
            tx_estring_value.append("MOTOR_FWD|"); tx_estring_value.append(speed);
            tx_characteristic_string.writeValue(tx_estring_value.c_str());
            Serial.print("Motor FWD speed="); Serial.print(speed);
            Serial.print(" dur="); Serial.println(duration);
            break;
        }

        /*
         * MOTOR_BACKWARD - Drive backward
         * Args: speed [duration_ms]
         */
        case MOTOR_BACKWARD:
        {
            int speed, duration;
            if (!robot_cmd.get_next_value(speed)) break;
            if (!robot_cmd.get_next_value(duration)) duration = 0;
            motorsBackward(speed);
            motorMode = MOT_BWD;
            if (duration > 0) { motorEndTime = millis() + duration; motorTimed = true; }
            else               { motorTimed = false; }
            tx_estring_value.clear();
            tx_estring_value.append("MOTOR_BWD|"); tx_estring_value.append(speed);
            tx_characteristic_string.writeValue(tx_estring_value.c_str());
            break;
        }

        /*
         * MOTOR_STOP - Stop all motors immediately
         */
        case MOTOR_STOP:
            motorsStop();
            motorMode  = MOT_IDLE;
            motorTimed = false;
            tx_estring_value.clear();
            tx_estring_value.append("MOTOR_STOP");
            tx_characteristic_string.writeValue(tx_estring_value.c_str());
            Serial.println("Motor STOP");
            break;

        /*
         * MOTOR_TURN_LEFT - On-axis left spin
         * Args: speed [duration_ms]
         */
        case MOTOR_TURN_LEFT:
        {
            int speed, duration;
            if (!robot_cmd.get_next_value(speed)) break;
            if (!robot_cmd.get_next_value(duration)) duration = 0;
            motorsTurnLeft(speed);
            motorMode = MOT_LEFT;
            if (duration > 0) { motorEndTime = millis() + duration; motorTimed = true; }
            else               { motorTimed = false; }
            tx_estring_value.clear();
            tx_estring_value.append("MOTOR_LEFT|"); tx_estring_value.append(speed);
            tx_characteristic_string.writeValue(tx_estring_value.c_str());
            break;
        }

        /*
         * MOTOR_TURN_RIGHT - On-axis right spin
         * Args: speed [duration_ms]
         */
        case MOTOR_TURN_RIGHT:
        {
            int speed, duration;
            if (!robot_cmd.get_next_value(speed)) break;
            if (!robot_cmd.get_next_value(duration)) duration = 0;
            motorsTurnRight(speed);
            motorMode = MOT_RIGHT;
            if (duration > 0) { motorEndTime = millis() + duration; motorTimed = true; }
            else               { motorTimed = false; }
            tx_estring_value.clear();
            tx_estring_value.append("MOTOR_RIGHT|"); tx_estring_value.append(speed);
            tx_characteristic_string.writeValue(tx_estring_value.c_str());
            break;
        }

        /*
         * SET_MOTOR_CAL - Set right-motor calibration factor
         * Args: cal_factor (float, e.g. 1.05 makes right motor 5% faster)
         */
        case SET_MOTOR_CAL:
        {
            float cal;
            if (!robot_cmd.get_next_value(cal)) break;
            motorCalFactor = cal;
            tx_estring_value.clear();
            tx_estring_value.append("MOTOR_CAL|"); tx_estring_value.append(motorCalFactor);
            tx_characteristic_string.writeValue(tx_estring_value.c_str());
            Serial.print("motorCalFactor="); Serial.println(motorCalFactor);
            break;
        }

        /*
         * MOTOR_TURN_ANGLE - Gyro-controlled turn until target angle reached
         * Args: dir (0=left, 1=right)  [angle_deg (default 90)]
         */
        case MOTOR_TURN_ANGLE:
        {
            int dir, angle;
            if (!robot_cmd.get_next_value(dir)) break;
            if (!robot_cmd.get_next_value(angle)) angle = 90;
            turnTargetDeg    = (float)abs(angle);
            turnAccumDeg     = 0.0f;
            lastTurnGyroTime = millis();
            if (dir == 0) motorsTurnLeft(140);
            else          motorsTurnRight(140);
            motorMode  = MOT_TURN_ANGLE;
            motorTimed = false;
            tx_estring_value.clear();
            tx_estring_value.append("TURN_START|");
            tx_estring_value.append(dir == 0 ? "L" : "R");
            tx_estring_value.append("|");
            tx_estring_value.append(angle);
            tx_characteristic_string.writeValue(tx_estring_value.c_str());
            Serial.print("Turn start dir="); Serial.print(dir);
            Serial.print(" target="); Serial.println(angle);
            break;
        }

        /*
         * The default case may not capture all types of invalid commands.
         * It is safer to validate the command string on the central device (in python)
         * before writing to the characteristic.
         */
        default:
            Serial.print("Invalid Command Type: ");
            Serial.println(cmd_type);
            break;
    }
}

void
setup()
{
    Serial.begin(115200);

    // MUST be the very first I/O operation:
    // Pull both XSHUT pins LOW to force both sensors into hardware reset.
    // This guarantees a clean boot from any restart mode (cold or hot).
    pinMode(SHUTDOWN_PIN_1, OUTPUT);
    digitalWrite(SHUTDOWN_PIN_1, LOW);
    pinMode(SHUTDOWN_PIN, OUTPUT);
    digitalWrite(SHUTDOWN_PIN, LOW);
    delay(10);  // ensure both sensors are fully off before Wire.begin()

    // LED blink to indicate startup
    pinMode(LED_BUILTIN, OUTPUT);
    for (int i = 0; i < 3; i++) {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(300);
        digitalWrite(LED_BUILTIN, LOW);
        delay(300);
    }

    // Initialize IMU
    Wire.begin();
    Wire.setClock(400000);

    myICM.begin(Wire, AD0_VAL);
    if (myICM.status == ICM_20948_Stat_Ok) {
        imuInitialized = true;
        Serial.println("IMU initialized successfully");
    } else {
        Serial.print("IMU init failed: ");
        Serial.println(myICM.statusString());
    }
    lastIMUTime = millis();

    // Initialize motor pins
    pinMode(L_FWD, OUTPUT); pinMode(L_BWD, OUTPUT);
    pinMode(R_FWD, OUTPUT); pinMode(R_BWD, OUTPUT);
    motorsStop();
    Serial.println("Motors initialized");

    // Initialize ToF sensors
    init_tof_sensors();

    BLE.begin();

    // Set advertised local name and service
    BLE.setDeviceName("Artemis BLE");
    BLE.setLocalName("Artemis BLE");
    BLE.setAdvertisedService(testService);

    // Add BLE characteristics
    testService.addCharacteristic(tx_characteristic_float);
    testService.addCharacteristic(tx_characteristic_string);
    testService.addCharacteristic(rx_characteristic_string);

    // Add BLE service
    BLE.addService(testService);

    // Initial values for characteristics
    // Set initial values to prevent errors when reading for the first time on central devices
    tx_characteristic_float.writeValue(0.0);

    /*
     * An example using the EString
     */
    // Clear the contents of the EString before using it
    tx_estring_value.clear();

    // Append the string literal "[->"
    tx_estring_value.append("[->");

    // Append the float value
    tx_estring_value.append(9.0);

    // Append the string literal "<-]"
    tx_estring_value.append("<-]");

    // Write the value to the characteristic
    tx_characteristic_string.writeValue(tx_estring_value.c_str());

    // Output MAC Address
    Serial.print("Advertising BLE with MAC: ");
    Serial.println(BLE.address());

    BLE.advertise();
}

void
write_data()
{
    currentMillis = millis();
    if (currentMillis - previousMillis > interval) {

        tx_float_value = tx_float_value + 0.5;
        tx_characteristic_float.writeValue(tx_float_value);

        if (tx_float_value > 10000) {
            tx_float_value = 0;
            
        }

        previousMillis = currentMillis;
    }
}

void
read_data()
{
    // Query if the characteristic value has been written by another BLE device
    if (rx_characteristic_string.written()) {
        handle_command();
    }
}

/*
 * Record timestamp and temperature data
 * This function is used for Task 6 and 7
 */
void
record_data()
{
    if (!arrayFull) {
        timeStamps[dataIndex] = millis();
        // Simulate temperature reading (can be replaced with actual sensor reading)
        // Using internal temperature sensor on Artemis
        tempReadings[dataIndex] = getTempDegC();

        dataIndex++;

        if (dataIndex >= MAX_DATA_SIZE) {
            arrayFull = true;
            dataIndex = 0; // Reset for circular buffer if needed
            Serial.println("Array is full!");
        }
    }
}

/*
 * Record IMU data at max speed (no delays)
 * Called from main loop when collectingIMU is true
 */
void
record_imu_data()
{
    if (!imuInitialized || imuArrayFull) return;
    if (!myICM.dataReady()) return;

    myICM.getAGMT();
    unsigned long now = millis();
    float dt = (now - lastIMUTime) / 1000.0;
    if (dt <= 0) dt = 0.001;
    lastIMUTime = now;

    float ax = myICM.accX();
    float ay = myICM.accY();
    float az = myICM.accZ();
    float gx = myICM.gyrX();
    float gy = myICM.gyrY();
    float gz = myICM.gyrZ();

    float pitch_a = atan2(ax, sqrt(ay*ay + az*az)) * 180.0 / M_PI;
    float roll_a = atan2(ay, sqrt(ax*ax + az*az)) * 180.0 / M_PI;

    pitch_a_lpf = alpha_lpf * pitch_a + (1.0 - alpha_lpf) * pitch_a_lpf;
    roll_a_lpf = alpha_lpf * roll_a + (1.0 - alpha_lpf) * roll_a_lpf;

    pitch_g += gx * dt;
    roll_g += gy * dt;
    yaw_g += gz * dt;

    pitch_comp = (1.0 - alpha_comp) * (pitch_comp + gx * dt) + alpha_comp * pitch_a;
    roll_comp = (1.0 - alpha_comp) * (roll_comp + gy * dt) + alpha_comp * roll_a;

    // Store in arrays
    imuTimeStamps[imuIndex] = now;
    imuAx[imuIndex] = ax;
    imuAy[imuIndex] = ay;
    imuAz[imuIndex] = az;
    imuGx[imuIndex] = gx;
    imuGy[imuIndex] = gy;
    imuGz[imuIndex] = gz;
    imuPitchA[imuIndex] = pitch_a;
    imuRollA[imuIndex] = roll_a;
    imuPitchALpf[imuIndex] = pitch_a_lpf;
    imuRollALpf[imuIndex] = roll_a_lpf;
    imuPitchG[imuIndex] = pitch_g;
    imuRollG[imuIndex] = roll_g;
    imuYawG[imuIndex] = yaw_g;
    imuPitchComp[imuIndex] = pitch_comp;
    imuRollComp[imuIndex] = roll_comp;

    imuIndex++;
    if (imuIndex >= MAX_IMU_SIZE) {
        imuArrayFull = true;
        collectingIMU = false;  // Auto-stop when full
        Serial.println("IMU array full, auto-stopped");
    }
}

/*
 * Initialize both ToF sensors.
 *
 * Wiring:
 *   Sensor 1 — always powered (no XSHUT wire needed).
 *   Sensor 2 — XSHUT connected to A0 (SHUTDOWN_PIN = 0).
 *
 * Sequence:
 *   1. Pull SHUTDOWN_PIN LOW  → disable sensor 2.
 *   2. Change sensor 1 address to TOF1_ADDR (0x30).
 *   3. Pull SHUTDOWN_PIN HIGH → sensor 2 boots at default 0x29.
 */
void
init_tof_sensors()
{
    // Both XSHUT pins are already LOW from setup() — sensors are in hardware reset.
    // Release sensor 1 first so it can be addressed exclusively at 0x29.
    digitalWrite(SHUTDOWN_PIN_1, HIGH);
    delay(10);  // VL53L1X boot time

    if (tofSensor1.begin() == 0) {
        tofSensor1.setI2CAddress(TOF1_ADDR);  // remap sensor 1 → 0x30
        tofSensor1.setDistanceModeLong();
        tofSensor1.startRanging();
        tof1Ready = true;
        Serial.println("ToF sensor 1 OK");
    } else {
        Serial.println("ToF sensor 1 FAILED");
    }

    // Now release sensor 2; sensor 1 is already at 0x30, no collision.
    digitalWrite(SHUTDOWN_PIN, HIGH);
    delay(10);  // VL53L1X boot time

    if (tofSensor2.begin() == 0) {
        tofSensor2.setDistanceModeLong();
        tofSensor2.startRanging();
        tof2Ready = true;
        Serial.println("ToF sensor 2 OK");
    } else {
        Serial.println("ToF sensor 2 FAILED");
    }
}

/*
 * Always-on ToF diagnostic: polls both sensors non-blocking and prints
 * distance (mm), measurement latency (ms), and sample rate (Hz) once per second.
 * Call from loop() when !collectingTOF to avoid double-consuming data-ready interrupts.
 */
void
poll_tof_diag()
{
    static unsigned long t1_cleared = 0;
    static unsigned long t2_cleared = 0;
    static unsigned long t1_last    = 0;
    static unsigned long t2_last    = 0;
    static float         hz1        = 0.0f;
    static float         hz2        = 0.0f;
    static unsigned long lat1       = 0;
    static unsigned long lat2       = 0;
    static int           d1         = -1;
    static int           d2         = -1;
    static unsigned long nextPrint  = 0;

    unsigned long now = millis();

    if (tof1Ready && tofSensor1.checkForDataReady()) {
        d1   = tofSensor1.getDistance();
        lat1 = (t1_cleared > 0) ? (now - t1_cleared) : 0;
        if (t1_last > 0) hz1 = 1000.0f / (float)max(1UL, now - t1_last);
        t1_last = now;
        tofSensor1.clearInterrupt();
        t1_cleared = millis();
    }

    if (tof2Ready && tofSensor2.checkForDataReady()) {
        d2   = tofSensor2.getDistance();
        lat2 = (t2_cleared > 0) ? (now - t2_cleared) : 0;
        if (t2_last > 0) hz2 = 1000.0f / (float)max(1UL, now - t2_last);
        t2_last = now;
        tofSensor2.clearInterrupt();
        t2_cleared = millis();
    }

    if (now >= nextPrint) {
        nextPrint = now + 1000;
        Serial.print("[TOF] S1: ");
        if (tof1Ready) {
            Serial.print(d1);     Serial.print(" mm | lat=");
            Serial.print(lat1);   Serial.print(" ms | ");
            Serial.print(hz1, 1); Serial.print(" Hz");
        } else {
            Serial.print("FAILED");
        }
        Serial.print("   S2: ");
        if (tof2Ready) {
            Serial.print(d2);     Serial.print(" mm | lat=");
            Serial.print(lat2);   Serial.print(" ms | ");
            Serial.print(hz2, 1); Serial.print(" Hz");
        } else {
            Serial.print("FAILED");
        }
        Serial.println();
    }
}

/*
 * Non-blocking ToF sample: called from main loop.
 * Records one entry when sensor 1 has fresh data; reads sensor 2 simultaneously.
 */
void
record_tof_data()
{
    if (!tof1Ready || tofArrayFull) return;
    if (!tofSensor1.checkForDataReady()) return;

    tofTimeStamps[tofIndex] = millis();
    tofDist1[tofIndex] = (int16_t)tofSensor1.getDistance();
    tofSensor1.clearInterrupt();

    if (tof2Ready && tofSensor2.checkForDataReady()) {
        tofDist2[tofIndex] = (int16_t)tofSensor2.getDistance();
        tofSensor2.clearInterrupt();
    } else {
        tofDist2[tofIndex] = -1;  // sensor 2 not ready this cycle
    }

    tofIndex++;
    if (tofIndex >= MAX_TOF_SIZE) {
        tofArrayFull = true;
        collectingTOF = false;
        Serial.println("ToF array full, auto-stopped");
    }
}

void
loop()
{
    // Listen for connections
    BLEDevice central = BLE.central();

    // If a central is connected to the peripheral
    if (central) {
        Serial.print("Connected to: ");
        Serial.println(central.address());

        // While central is connected
        while (central.connected()) {
            // Send data
            write_data();

            // Read data
            read_data();

            // Motor state machine (non-blocking)
            handle_motors();

            // Collect data if recording is enabled
            if (collectingData && (millis() - lastSampleTime >= sampleInterval)) {
                record_data();
                lastSampleTime = millis();
            }

            // Collect IMU data if recording (no delay - max speed)
            if (collectingIMU) {
                record_imu_data();
            }

            // Non-blocking ToF data collection
            if (collectingTOF) {
                record_tof_data();
            } else {
                poll_tof_diag();
            }
        }

        Serial.println("Disconnected");
    } else {
        // Not BLE-connected: still run diagnostics so you can see sensor state
        poll_tof_diag();
    }
}
