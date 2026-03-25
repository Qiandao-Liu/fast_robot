from enum import Enum


class CMD(Enum):
    PING = 0

    # Command IDs 1-10 were retired with the demo-only firmware cleanup.

    # IMU Commands
    GET_IMU_DATA = 11
    START_IMU_RECORDING = 12
    STOP_IMU_RECORDING = 13
    SEND_IMU_DATA = 14
    SET_IMU_PARAMS = 15
    RESET_IMU_ANGLES = 16

    # ToF Commands
    GET_TOF_DATA = 17
    START_TOF_RECORDING = 18
    STOP_TOF_RECORDING = 19
    SEND_TOF_DATA = 20
    SET_TOF_MODE = 21

    # Motor Commands
    MOTOR_FORWARD = 22
    MOTOR_BACKWARD = 23
    MOTOR_STOP = 24
    MOTOR_TURN_LEFT = 25
    MOTOR_TURN_RIGHT = 26
    SET_MOTOR_CAL = 27
    MOTOR_TURN_ANGLE = 28

    # PID Commands
    PID_START = 29
    PID_STOP = 30
    GET_PID_DATA = 31
    SET_PID_GAINS = 32

    # Orientation PID Commands
    ORIENT_START = 33
    ORIENT_STOP = 34
    GET_ORIENT_DATA = 35
    SET_ORIENT_GAINS = 36
    SET_ORIENT_TARGET = 37

    # Kalman Filter Commands
    KF_STEP_START = 38
    KF_STEP_STOP = 39
    SET_KF_PARAMS = 40
    KF_PID_START = 41
    GET_KF_DEBUG_DATA = 42

    # Drift Stunt Commands (Lab 8)
    DRIFT_START = 43      # params: approach_pwm|return_pwm|trigger_dist_mm|return_ms|timeout_ms
    DRIFT_STOP = 44
    GET_DRIFT_DATA = 45
    SET_DRIFT_PARAMS = 46  # same params as DRIFT_START, sets without starting
