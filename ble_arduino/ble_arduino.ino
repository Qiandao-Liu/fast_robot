#include "BLECStringCharacteristic.h"
#include "EString.h"
#include "RobotCommand.h"
#include <ArduinoBLE.h>
#include "ICM_20948.h"
#include <BasicLinearAlgebra.h>
#include <SparkFun_VL53L1X.h>
#include <math.h>

using namespace BLA;

//////////// BLE UUIDs ////////////
#define BLE_UUID_TEST_SERVICE "d1e59283-ea64-46d2-9619-feda9179e362"
#define BLE_UUID_RX_STRING    "9750f60b-9c9c-4158-b620-02ec9521cd99"
#define BLE_UUID_TX_STRING    "f235a225-6735-4d73-94cb-ee5dfce9ba83"
//////////// BLE UUIDs ////////////

//////////// Motor Setup ////////////
#define L_FWD 3
#define L_BWD 14
#define R_FWD 16
#define R_BWD 15

enum MotorMode { MOT_IDLE, MOT_FWD, MOT_BWD, MOT_LEFT, MOT_RIGHT, MOT_TURN_ANGLE };
enum RunMode { RUN_IDLE, RUN_PID_LINEAR, RUN_PID_KF, RUN_ORIENT, RUN_KF_STEP, RUN_DRIFT, RUN_MAP };
enum RunStopReason { STOP_CMD, STOP_TIMEOUT, STOP_SAFETY, STOP_MODE_SWITCH };

float motorCalFactor = 1.0f;
MotorMode motorMode = MOT_IDLE;
RunMode runMode = RUN_IDLE;
unsigned long motorEndTime = 0;
bool motorTimed = false;

float turnTargetDeg = 90.0f;
float turnAccumDeg = 0.0f;
unsigned long lastTurnGyroTime = 0;
//////////// Motor Setup ////////////

//////////// ToF Setup ////////////
#define SHUTDOWN_PIN_1 1
#define SHUTDOWN_PIN   0
#define TOF1_ADDR      0x30

#define MAX_TOF_SIZE 500

SFEVL53L1X tofSensor1;
SFEVL53L1X tofSensor2;
bool tof1Ready = false;
bool tof2Ready = false;

unsigned long tofTimeStamps[MAX_TOF_SIZE];
int16_t tofDist1[MAX_TOF_SIZE];
int16_t tofDist2[MAX_TOF_SIZE];
int tofIndex = 0;
bool tofArrayFull = false;
bool collectingTOF = false;
//////////// ToF Setup ////////////

//////////// IMU Setup ////////////
#define AD0_VAL 1
#define MAX_IMU_SIZE 1200

ICM_20948_I2C myICM;
bool imuInitialized = false;
bool imuDmpReady = false;

float alpha_lpf = 0.2f;
float alpha_comp = 0.05f;

float pitch_a_lpf = 0.0f;
float roll_a_lpf = 0.0f;
float pitch_g = 0.0f;
float roll_g = 0.0f;
float yaw_g = 0.0f;
float pitch_comp = 0.0f;
float roll_comp = 0.0f;
float imu_pitch_a_raw = 0.0f;
float imu_roll_a_raw = 0.0f;
float imu_last_gyr_z = 0.0f;
float driftDmpHeadingDeg = 0.0f;
float driftDmpHeadingZeroDeg = 0.0f;
unsigned long imuSampleTimeMs = 0;
unsigned long lastIMUTime = 0;
unsigned long yawLastUpdateMs = 0;
unsigned long driftDmpHeadingTsMs = 0;
bool driftDmpHeadingValid = false;
bool driftDmpZeroSet = false;
int16_t driftDmpHeadingAccuracy = -1;

unsigned long imuTimeStamps[MAX_IMU_SIZE];
float imuPitchA[MAX_IMU_SIZE];
float imuRollA[MAX_IMU_SIZE];
float imuPitchComp[MAX_IMU_SIZE];
float imuRollComp[MAX_IMU_SIZE];
int imuIndex = 0;
bool imuArrayFull = false;
bool collectingIMU = false;
//////////// IMU Setup ////////////

//////////// PID / Range Trace Setup ////////////
#define PID_LENGTH 3000
#define PID_SAFETY_MARGIN_MM 60
#define DEADBAND_MIN 40
#define PWM_MAX 160

enum RangeTraceKind { RANGE_REAL = 0, RANGE_EXTRAP = 1, RANGE_KF_PRED = 2 };

float pid_kp = 0.055f;
float pid_ki = 0.003f;
float pid_kd = 0.016f;
int pid_setpoint = 304;
unsigned long pid_start_ms = 0;
unsigned long pid_timeout_ms = 10000;

int16_t pid_e_hist[PID_LENGTH];
int16_t pid_motor_hist[PID_LENGTH];
uint32_t pid_t_hist[PID_LENGTH];
int pid_e_pos = 0;

int16_t range_trace_dist[PID_LENGTH];
uint8_t range_trace_kind[PID_LENGTH];
uint32_t range_trace_t[PID_LENGTH];
int range_trace_pos = 0;

float pid_I = 0.0f;
float pid_dF = 0.0f;
float pid_last_e = 0.0f;
unsigned long pid_last_t = 0;

float tof_slope = 0.0f;
float tof_last_val = 0.0f;
unsigned long tof_last_t_ms = 0;
bool tof_extrap_valid = false;
float tof_current = -1.0f;

#define KF_DEBUG_LENGTH 800
#define KF_DEBUG_INTERVAL_MS 20
#define KF_GATE_MIN_MM 75.0f
#define KF_FLAG_REAL_TOF 0x01
#define KF_FLAG_KF_UPDATE 0x02
#define KF_FLAG_KF_REJECT 0x04
#define KF_FLAG_SAFETY_STOP 0x08

int16_t kf_dbg_raw_hist[KF_DEBUG_LENGTH];
int16_t kf_dbg_est_hist[KF_DEBUG_LENGTH];
int16_t kf_dbg_vel_hist[KF_DEBUG_LENGTH];
int16_t kf_dbg_err_hist[KF_DEBUG_LENGTH];
int16_t kf_dbg_pwm_hist[KF_DEBUG_LENGTH];
uint8_t kf_dbg_flags_hist[KF_DEBUG_LENGTH];
uint32_t kf_dbg_t_hist[KF_DEBUG_LENGTH];
int kf_dbg_pos = 0;
unsigned long kf_dbg_last_log_ms = 0;
bool pid_safety_stop_latched = false;
//////////// PID / Range Trace Setup ////////////

//////////// Drift Stunt Setup (Lab 8) ////////////
#define DRIFT_LOG_LEN  800
#define DRIFT_ROTATE_DONE_BAND_DEG 8.0f
#define DRIFT_ROTATE_DONE_COUNT 3
#define DRIFT_TURN_PROGRESS_MIN 150.0f
#define DRIFT_RETURN_STEER_MAX 60

int   drift_approach_pwm  = 160;
int   drift_return_pwm    = 160;
float drift_trigger_dist  = 914.0f;
float drift_stop_dist     = 300.0f;
float drift_return_yaw_kp = 1.0f;
unsigned long drift_return_ms   = 2000;
unsigned long drift_timeout_ms  = 8000;

int   drift_phase              = 0;   // 0=approach 1=rotate 2=return 3=done
float drift_approach_heading_ref = 0.0f;
int   drift_rotate_done_count  = 0;
unsigned long drift_start_ms        = 0;
unsigned long drift_return_start_ms = 0;

int16_t  drift_raw_hist[DRIFT_LOG_LEN];
int16_t  drift_est_hist[DRIFT_LOG_LEN];
int16_t  drift_yaw_hist[DRIFT_LOG_LEN];
int16_t  drift_mot_hist[DRIFT_LOG_LEN];
int16_t  drift_heading_err_hist[DRIFT_LOG_LEN];
int16_t  drift_gyro_hist[DRIFT_LOG_LEN];
uint8_t  drift_phase_hist[DRIFT_LOG_LEN];
uint32_t drift_t_hist[DRIFT_LOG_LEN];
int      drift_log_pos = 0;
//////////// Drift Stunt Setup (Lab 8) ////////////

//////////// Mapping Setup (Lab 9) ////////////
#define MAP_LOG_LEN          256
#define MAP_DONE_BAND_DEG    10.0f
#define MAP_DONE_GYRO_DPS    40.0f
#define MAP_DONE_COUNT       1
#define MAP_SAMPLE_WAIT_MS   120

int map_step_deg = 3;
int map_samples_goal = 127;
unsigned long map_timeout_ms = 120000;
int map_turn_dir = 1;
int map_sweep_deg = 380;

int map_phase = 0;  // 0=rotate/settle 1=sample 2=done
int map_sample_idx = 0;
int map_done_count = 0;
unsigned long map_start_ms = 0;
unsigned long map_sample_wait_start_ms = 0;

int16_t  map_target_hist[MAP_LOG_LEN];
int16_t  map_heading_hist[MAP_LOG_LEN];
int16_t  map_front_hist[MAP_LOG_LEN];
int16_t  map_right_hist[MAP_LOG_LEN];
uint32_t map_t_hist[MAP_LOG_LEN];
int      map_log_pos = 0;
//////////// Mapping Setup (Lab 9) ////////////

//////////// Kalman Filter Setup ////////////
float kf_d = 0.0003163f;
float kf_m = 0.0002093f;
float kf_sig1 = 50.0f;
float kf_sig2 = 50.0f;
float kf_sig3 = 20.0f;

Matrix<2,1> kf_mu;
Matrix<2,2> kf_sigma;
Matrix<2,2> kf_A_cont;
Matrix<2,1> kf_B_cont;
Matrix<1,2> kf_C;

bool kf_initialized = false;
unsigned long kf_prev_t_ms = 0;
float kf_last_u = 0.0f;
bool kf_last_meas_used = false;
bool kf_last_meas_rejected = false;
float kf_last_innovation_mm = 0.0f;
float kf_last_gate_mm = KF_GATE_MIN_MM;

int kf_step_pwm_val = 150;
int kf_step_min_dist = 400;
unsigned long kf_step_start_ms = 0;
unsigned long kf_step_timeout = 8000;
unsigned long kf_step_kick_until_ms = 0;

#define KF_STEP_KICK_PWM 160
#define KF_STEP_KICK_MS 80
#define KF_STEP_MIN_ALLOWED_MM 520
//////////// Kalman Filter Setup ////////////

//////////// Orientation PID Setup ////////////
#define ORIENT_LENGTH 3000
#define TURN_DEADBAND 120
#define TURN_PWM_MAX 160
#define TURN_KICK_PWM 170
#define TURN_KICK_MS 70

float orient_kp = 2.0f;
float orient_ki = 0.0f;
float orient_kd = 0.0f;
float orient_target_deg = 90.0f;
unsigned long orient_start_ms = 0;
unsigned long orient_timeout_ms = 12000;

int16_t orient_yaw_hist[ORIENT_LENGTH];
int16_t orient_e_hist[ORIENT_LENGTH];
int16_t orient_motor_hist[ORIENT_LENGTH];
uint32_t orient_t_hist[ORIENT_LENGTH];
int orient_pos = 0;

float orient_I = 0.0f;
float orient_dF = 0.0f;
float orient_last_e = 0.0f;
unsigned long orient_last_t = 0;
int orient_last_pwm_sign = 0;
unsigned long orient_kick_until_ms = 0;
//////////// Orientation PID Setup ////////////

//////////// BLE Globals ////////////
BLEService testService(BLE_UUID_TEST_SERVICE);
BLECStringCharacteristic rx_characteristic_string(BLE_UUID_RX_STRING, BLEWrite, MAX_MSG_SIZE);
BLECStringCharacteristic tx_characteristic_string(BLE_UUID_TX_STRING, BLERead | BLENotify, MAX_MSG_SIZE);

RobotCommand robot_cmd(":|");
EString tx_estring_value;
//////////// BLE Globals ////////////

enum CommandTypes {
    PING = 0,
    CMD_RETIRED_1 = 1,
    CMD_RETIRED_2 = 2,
    CMD_RETIRED_3 = 3,
    CMD_RETIRED_4 = 4,
    CMD_RETIRED_5 = 5,
    CMD_RETIRED_6 = 6,
    CMD_RETIRED_7 = 7,
    CMD_RETIRED_8 = 8,
    CMD_RETIRED_9 = 9,
    CMD_RETIRED_10 = 10,
    GET_IMU_DATA = 11,
    START_IMU_RECORDING = 12,
    STOP_IMU_RECORDING = 13,
    SEND_IMU_DATA = 14,
    SET_IMU_PARAMS = 15,
    RESET_IMU_ANGLES = 16,
    GET_TOF_DATA = 17,
    START_TOF_RECORDING = 18,
    STOP_TOF_RECORDING = 19,
    SEND_TOF_DATA = 20,
    SET_TOF_MODE = 21,
    MOTOR_FORWARD = 22,
    MOTOR_BACKWARD = 23,
    MOTOR_STOP = 24,
    MOTOR_TURN_LEFT = 25,
    MOTOR_TURN_RIGHT = 26,
    SET_MOTOR_CAL = 27,
    MOTOR_TURN_ANGLE = 28,
    PID_START = 29,
    PID_STOP = 30,
    GET_PID_DATA = 31,
    SET_PID_GAINS = 32,
    ORIENT_START = 33,
    ORIENT_STOP = 34,
    GET_ORIENT_DATA = 35,
    SET_ORIENT_GAINS = 36,
    SET_ORIENT_TARGET = 37,
    KF_STEP_START = 38,
    KF_STEP_STOP = 39,
    SET_KF_PARAMS = 40,
    KF_PID_START = 41,
    GET_KF_DEBUG_DATA = 42,
    DRIFT_START = 43,
    DRIFT_STOP = 44,
    GET_DRIFT_DATA = 45,
    SET_DRIFT_PARAMS = 46,
    MAP_START = 47,
    MAP_STOP = 48,
    GET_MAP_DATA = 49,
    SET_MAP_PARAMS = 50,
    GET_MAP_STATUS = 51,
};

void setup() {
    Serial.begin(115200);

    pinMode(SHUTDOWN_PIN_1, OUTPUT);
    digitalWrite(SHUTDOWN_PIN_1, LOW);
    pinMode(SHUTDOWN_PIN, OUTPUT);
    digitalWrite(SHUTDOWN_PIN, LOW);
    delay(10);

    pinMode(LED_BUILTIN, OUTPUT);
    for (int i = 0; i < 3; i++) {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(300);
        digitalWrite(LED_BUILTIN, LOW);
        delay(300);
    }

    Wire.begin();
    Wire.setClock(400000);

    myICM.begin(Wire, AD0_VAL);
    if (myICM.status == ICM_20948_Stat_Ok) {
        imuInitialized = true;
        Serial.println("IMU initialized successfully");
        imuDmpReady = init_imu_dmp();
    } else {
        Serial.print("IMU init failed: ");
        Serial.println(myICM.statusString());
    }
    lastIMUTime = millis();

    pinMode(L_FWD, OUTPUT);
    pinMode(L_BWD, OUTPUT);
    pinMode(R_FWD, OUTPUT);
    pinMode(R_BWD, OUTPUT);
    motorsStop();
    Serial.println("Motors initialized");

    init_tof_sensors();

    BLE.begin();
    BLE.setDeviceName("Artemis BLE");
    BLE.setLocalName("Artemis BLE");
    BLE.setAdvertisedService(testService);

    testService.addCharacteristic(tx_characteristic_string);
    testService.addCharacteristic(rx_characteristic_string);
    BLE.addService(testService);

    tx_characteristic_string.writeValue("READY");

    Serial.print("Advertising BLE with MAC: ");
    Serial.println(BLE.address());
    BLE.advertise();
}

void read_data() {
    if (rx_characteristic_string.written()) {
        handle_command();
    }
}

void loop() {
    BLEDevice central = BLE.central();
    if (!central) {
        return;
    }

    Serial.print("Connected to: ");
    Serial.println(central.address());

    while (central.connected()) {
        read_data();
        handle_motors();

        if (collectingIMU && runMode != RUN_ORIENT) {
            record_imu_data();
        }

        switch (runMode) {
            case RUN_PID_LINEAR:
            case RUN_PID_KF:
                handle_pid();
                break;
            case RUN_ORIENT:
                handle_orient_pid();
                break;
            case RUN_KF_STEP:
                handle_kf_step();
                break;
            case RUN_DRIFT:
                handle_drift();
                break;
            case RUN_MAP:
                handle_map();
                break;
            case RUN_IDLE:
            default:
                if (collectingTOF) {
                    record_tof_data();
                }
                break;
        }
    }

    motorsStop();
    motorMode = MOT_IDLE;
    motorTimed = false;
    runMode = RUN_IDLE;
    collectingTOF = false;
    collectingIMU = false;
    Serial.println("Disconnected");
}
