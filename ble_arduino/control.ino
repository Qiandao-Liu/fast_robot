void motorsForward(int speed) {
    int rSpeed = constrain((int)(speed * motorCalFactor), 0, 255);
    analogWrite(L_FWD, speed);
    analogWrite(L_BWD, 0);
    analogWrite(R_FWD, rSpeed);
    analogWrite(R_BWD, 0);
}

void motorsBackward(int speed) {
    int rSpeed = constrain((int)(speed * motorCalFactor), 0, 255);
    analogWrite(L_BWD, speed);
    analogWrite(L_FWD, 0);
    analogWrite(R_BWD, rSpeed);
    analogWrite(R_FWD, 0);
}

void motorsStop() {
    analogWrite(L_FWD, 0);
    analogWrite(L_BWD, 0);
    analogWrite(R_FWD, 0);
    analogWrite(R_BWD, 0);
}

void motorsTurnLeft(int speed) {
    analogWrite(L_BWD, speed);
    analogWrite(L_FWD, 0);
    analogWrite(R_FWD, speed);
    analogWrite(R_BWD, 0);
}

void motorsTurnRight(int speed) {
    analogWrite(L_FWD, speed);
    analogWrite(L_BWD, 0);
    analogWrite(R_BWD, speed);
    analogWrite(R_FWD, 0);
}

void motorsForwardSteered(int baseSpeed, int steerBias) {
    int leftSpeed = constrain(baseSpeed + steerBias, 0, 255);
    int rightBase = constrain(baseSpeed - steerBias, 0, 255);
    int rightSpeed = constrain((int)(rightBase * motorCalFactor), 0, 255);

    analogWrite(L_FWD, leftSpeed);
    analogWrite(L_BWD, 0);
    analogWrite(R_FWD, rightSpeed);
    analogWrite(R_BWD, 0);
}

int16_t clamp_i16(float value) {
    if (value > 32767.0f) return 32767;
    if (value < -32768.0f) return -32768;
    return (int16_t)lroundf(value);
}

float wrap_angle_deg(float deg) {
    while (deg > 180.0f) deg -= 360.0f;
    while (deg < -180.0f) deg += 360.0f;
    return deg;
}

bool pidUsesKalman() {
    return runMode == RUN_PID_KF;
}

void reset_imu_filters() {
    pitch_a_lpf = 0.0f;
    roll_a_lpf = 0.0f;
    pitch_g = 0.0f;
    roll_g = 0.0f;
    yaw_g = 0.0f;
    pitch_comp = 0.0f;
    roll_comp = 0.0f;
    imu_pitch_a_raw = 0.0f;
    imu_roll_a_raw = 0.0f;
    imu_last_gyr_z = 0.0f;
    imuSampleTimeMs = 0;
    yawLastUpdateMs = 0;
    lastIMUTime = millis();
}

void reset_distance_pid_state(unsigned long now_ms) {
    pid_I = 0.0f;
    pid_dF = 0.0f;
    pid_last_e = 0.0f;
    pid_last_t = now_ms;
    pid_safety_stop_latched = false;
}

void reset_orient_pid_state(unsigned long now_ms, bool clear_history) {
    if (clear_history) {
        orient_pos = 0;
    }
    orient_I = 0.0f;
    orient_dF = 0.0f;
    orient_last_e = 0.0f;
    orient_last_pwm_sign = 0;
    orient_kick_until_ms = 0;
    orient_last_t = now_ms;
}

void apply_drive_pwm_signed(int pwm) {
    if (pwm > 0) {
        motorsForward(pwm);
    } else if (pwm < 0) {
        motorsBackward(-pwm);
    } else {
        motorsStop();
    }
}

void update_kf_control_input(int pwm) {
    float norm_base = (float)max(1, abs(kf_step_pwm_val));
    kf_last_u = (pwm != 0) ? -(float)pwm / norm_base : 0.0f;
}

bool update_yaw_estimate() {
    if (!update_imu_state()) return false;

    if (yawLastUpdateMs != 0 && imuSampleTimeMs > yawLastUpdateMs) {
        float dt = (float)(imuSampleTimeMs - yawLastUpdateMs) / 1000.0f;
        if (dt > 0.0002f) {
            yaw_g = wrap_angle_deg(yaw_g - imu_last_gyr_z * dt);
        }
    }

    yawLastUpdateMs = imuSampleTimeMs;
    return true;
}

bool step_distance_pid(float dist_mm, unsigned long now_ms, float &error_mm, int &pwm) {
    int dt = (int)(now_ms - pid_last_t);
    if (dt < 1) return false;
    pid_last_t = now_ms;

    error_mm = dist_mm - (float)pid_setpoint;
    pid_I += error_mm * (float)dt / 1000.0f;
    if (pid_I > 1000.0f) pid_I = 1000.0f;
    if (pid_I < -1000.0f) pid_I = -1000.0f;

    float d_raw = (error_mm - pid_last_e) / ((float)dt / 1000.0f);
    pid_dF = 0.9f * pid_dF + 0.1f * d_raw;
    pid_last_e = error_mm;

    float output = pid_kp * error_mm + pid_ki * pid_I + pid_kd * pid_dF;
    output = constrain(output, -(float)PWM_MAX, (float)PWM_MAX);

    pwm = 0;
    if (fabsf(output) <= 2.0f) {
        return true;
    }

    float abs_out = fabsf(output);
    float mapped = DEADBAND_MIN + (abs_out / (float)PWM_MAX) * (float)(PWM_MAX - DEADBAND_MIN);
    mapped = constrain(mapped, (float)DEADBAND_MIN, (float)PWM_MAX);
    pwm = (int)mapped;

    if (output > 0.0f && !pid_safety_stop_latched) {
        return true;
    }

    if (output <= 0.0f) {
        pwm = -pwm;
        return true;
    }

    pwm = 0;
    return true;
}

bool step_orient_pid_with_heading(float heading_deg, float &error_deg, int &pwm,
                                  unsigned long now_ms, bool log_history) {
    float dt = (float)(now_ms - orient_last_t) / 1000.0f;
    if (dt <= 0.0005f) return false;
    orient_last_t = now_ms;

    error_deg = wrap_angle_deg(orient_target_deg - heading_deg);

    orient_I += error_deg * dt;
    if (orient_I > 100.0f) orient_I = 100.0f;
    if (orient_I < -100.0f) orient_I = -100.0f;

    float d_raw = (error_deg - orient_last_e) / dt;
    orient_dF = 0.9f * orient_dF + 0.1f * d_raw;
    orient_last_e = error_deg;

    float output = orient_kp * error_deg + orient_ki * orient_I + orient_kd * orient_dF;
    output = constrain(output, -(float)TURN_PWM_MAX, (float)TURN_PWM_MAX);

    pwm = 0;
    int desired_sign = 0;
    float abs_out = fabsf(output);

    if (abs_out < 2.0f) {
        motorsStop();
        orient_last_pwm_sign = 0;
    } else {
        desired_sign = (output > 0.0f) ? 1 : -1;
        float mapped = TURN_DEADBAND + (abs_out - 2.0f) * (TURN_PWM_MAX - TURN_DEADBAND)
                       / (TURN_PWM_MAX - 2.0f);
        pwm = (int)constrain(mapped, (float)TURN_DEADBAND, (float)TURN_PWM_MAX);

        if ((orient_last_pwm_sign == 0 || desired_sign != orient_last_pwm_sign) &&
            now_ms >= orient_kick_until_ms) {
            orient_kick_until_ms = now_ms + TURN_KICK_MS;
        }

        if (now_ms < orient_kick_until_ms) {
            pwm = TURN_KICK_PWM;
        }

        if (desired_sign > 0) {
            motorsTurnRight(pwm);
        } else {
            motorsTurnLeft(pwm);
            pwm = -pwm;
        }
        orient_last_pwm_sign = desired_sign;
    }

    if (log_history && orient_pos < ORIENT_LENGTH) {
        orient_yaw_hist[orient_pos] = (int16_t)lroundf(heading_deg * 10.0f);
        orient_e_hist[orient_pos] = (int16_t)lroundf(error_deg * 10.0f);
        orient_motor_hist[orient_pos] = (int16_t)pwm;
        orient_t_hist[orient_pos] = now_ms;
        orient_pos++;
    }

    return true;
}

bool step_orient_pid(float &error_deg, int &pwm, unsigned long &now_ms, bool log_history) {
    if (!update_yaw_estimate()) return false;

    now_ms = imuSampleTimeMs;
    return step_orient_pid_with_heading(yaw_g, error_deg, pwm, now_ms, log_history);
}

void append_drift_log(int raw_mm, float est_mm, float heading_deg, int motor_cmd, int phase,
                      float heading_err_deg, float gyro_dps, unsigned long ts_ms) {
    if (drift_log_pos >= DRIFT_LOG_LEN) return;
    drift_raw_hist[drift_log_pos] = clamp_i16((float)raw_mm);
    drift_est_hist[drift_log_pos] = clamp_i16(est_mm);
    drift_yaw_hist[drift_log_pos] = clamp_i16(heading_deg * 10.0f);
    drift_mot_hist[drift_log_pos] = clamp_i16((float)motor_cmd);
    drift_heading_err_hist[drift_log_pos] = clamp_i16(heading_err_deg * 10.0f);
    drift_gyro_hist[drift_log_pos] = clamp_i16(gyro_dps * 10.0f);
    drift_phase_hist[drift_log_pos] = (uint8_t)phase;
    drift_t_hist[drift_log_pos] = ts_ms;
    drift_log_pos++;
}

void reset_kf_debug_log() {
    kf_dbg_pos = 0;
    kf_dbg_last_log_ms = 0;
    pid_safety_stop_latched = false;
    kf_last_meas_used = false;
    kf_last_meas_rejected = false;
    kf_last_innovation_mm = 0.0f;
    kf_last_gate_mm = KF_GATE_MIN_MM;
}

void append_kf_debug_row(int raw_mm, float est_mm, float vel_mmps,
                         float err_mm, int pwm, uint8_t flags,
                         unsigned long ts_ms) {
    if (kf_dbg_pos >= KF_DEBUG_LENGTH) return;
    kf_dbg_raw_hist[kf_dbg_pos] = clamp_i16((float)raw_mm);
    kf_dbg_est_hist[kf_dbg_pos] = clamp_i16(est_mm);
    kf_dbg_vel_hist[kf_dbg_pos] = clamp_i16(vel_mmps);
    kf_dbg_err_hist[kf_dbg_pos] = clamp_i16(err_mm);
    kf_dbg_pwm_hist[kf_dbg_pos] = clamp_i16((float)pwm);
    kf_dbg_flags_hist[kf_dbg_pos] = flags;
    kf_dbg_t_hist[kf_dbg_pos] = ts_ms;
    kf_dbg_pos++;
}

void emit_pid_safety_stop(int dist_mm, unsigned long ts_ms) {
    tx_estring_value.clear();
    tx_estring_value.append("PID_SAFETY_STOP|");
    tx_estring_value.append(dist_mm);
    tx_estring_value.append("|");
    tx_estring_value.append((int)ts_ms);
    ble_write_reliable(tx_estring_value.c_str());
}

void log_range_trace(int dist_mm, RangeTraceKind kind, unsigned long ts_ms) {
    if (range_trace_pos >= PID_LENGTH) return;
    range_trace_dist[range_trace_pos] = clamp_i16((float)dist_mm);
    range_trace_kind[range_trace_pos] = (uint8_t)kind;
    range_trace_t[range_trace_pos] = ts_ms;
    range_trace_pos++;
}

void stop_active_drive_run(RunStopReason reason, bool stop_motors) {
    if (stop_motors) {
        motorsStop();
    }
    motorMode = MOT_IDLE;
    motorTimed = false;
    if (reason == STOP_SAFETY) {
        pid_safety_stop_latched = true;
    }
    if (runMode == RUN_ORIENT) {
        orient_last_pwm_sign = 0;
    }
    runMode = RUN_IDLE;
}

void start_pid_run(bool use_kf) {
    stop_active_drive_run(STOP_MODE_SWITCH, true);
    collectingTOF = false;
    collectingIMU = false;
    pid_e_pos = 0;
    range_trace_pos = 0;
    reset_kf_debug_log();
    tof_extrap_valid = false;
    tof_current = -1.0f;
    tof_slope = 0.0f;
    tof_last_val = 0.0f;
    tof_last_t_ms = 0;
    kf_initialized = false;
    kf_last_u = 0.0f;
    pid_start_ms = millis();
    reset_distance_pid_state(pid_start_ms);
    runMode = use_kf ? RUN_PID_KF : RUN_PID_LINEAR;
}

void start_orient_run() {
    stop_active_drive_run(STOP_MODE_SWITCH, true);
    collectingTOF = false;
    collectingIMU = false;
    reset_imu_filters();
    orient_start_ms = millis();
    reset_orient_pid_state(orient_start_ms, true);
    runMode = RUN_ORIENT;
}

void start_kf_step_run(int pwm_val, int min_dist) {
    stop_active_drive_run(STOP_MODE_SWITCH, true);
    collectingTOF = false;
    collectingIMU = false;
    kf_step_pwm_val = pwm_val;
    kf_step_min_dist = max(min_dist, KF_STEP_MIN_ALLOWED_MM);
    range_trace_pos = 0;
    pid_e_pos = 0;
    reset_kf_debug_log();
    kf_step_start_ms = millis();
    kf_step_kick_until_ms = kf_step_start_ms + KF_STEP_KICK_MS;
    runMode = RUN_KF_STEP;
    motorsForward(max(kf_step_pwm_val, KF_STEP_KICK_PWM));
}

void set_manual_motor(MotorMode mode, int speed, int duration_ms, const char *ack_prefix) {
    stop_active_drive_run(STOP_MODE_SWITCH, true);

    switch (mode) {
        case MOT_FWD:
            motorsForward(speed);
            break;
        case MOT_BWD:
            motorsBackward(speed);
            break;
        case MOT_LEFT:
            motorsTurnLeft(speed);
            break;
        case MOT_RIGHT:
            motorsTurnRight(speed);
            break;
        default:
            motorsStop();
            break;
    }

    motorMode = mode;
    if (duration_ms > 0) {
        motorEndTime = millis() + duration_ms;
        motorTimed = true;
    } else {
        motorTimed = false;
    }

    tx_estring_value.clear();
    tx_estring_value.append(ack_prefix);
    tx_estring_value.append("|");
    tx_estring_value.append(speed);
    tx_characteristic_string.writeValue(tx_estring_value.c_str());
}

void handle_motors() {
    if (motorMode == MOT_IDLE) return;

    if (motorMode == MOT_TURN_ANGLE) {
        if (!update_imu_state()) return;

        unsigned long now = imuSampleTimeMs;
        float dt = (float)(now - lastTurnGyroTime) / 1000.0f;
        if (dt < 0.0002f) return;
        lastTurnGyroTime = now;
        turnAccumDeg += imu_last_gyr_z * dt;

        if (fabsf(turnAccumDeg) >= turnTargetDeg) {
            motorsStop();
            motorMode = MOT_IDLE;
            tx_estring_value.clear();
            tx_estring_value.append("TURN_DONE|");
            tx_estring_value.append(turnAccumDeg);
            tx_characteristic_string.writeValue(tx_estring_value.c_str());
            Serial.print("Turn done, angle=");
            Serial.println(turnAccumDeg);
        }
        return;
    }

    if (motorTimed && millis() >= motorEndTime) {
        motorsStop();
        motorMode = MOT_IDLE;
        motorTimed = false;
    }
}

void kf_init(float first_dist_mm) {
    kf_A_cont(0, 0) = 0.0f;
    kf_A_cont(0, 1) = 1.0f;
    kf_A_cont(1, 0) = 0.0f;
    kf_A_cont(1, 1) = -(kf_d / kf_m);
    kf_B_cont(0, 0) = 0.0f;
    kf_B_cont(1, 0) = 1.0f / kf_m;
    kf_C(0, 0) = 1.0f;
    kf_C(0, 1) = 0.0f;

    kf_mu(0, 0) = first_dist_mm;
    kf_mu(1, 0) = 0.0f;

    kf_sigma(0, 0) = kf_sig1 * kf_sig1;
    kf_sigma(0, 1) = 0.0f;
    kf_sigma(1, 0) = 0.0f;
    kf_sigma(1, 1) = kf_sig2 * kf_sig2;

    kf_initialized = true;
}

float kf_step_fn(float dt_s, float u, bool has_meas, float z) {
    kf_last_meas_used = false;
    kf_last_meas_rejected = false;
    kf_last_innovation_mm = 0.0f;
    kf_last_gate_mm = KF_GATE_MIN_MM;

    Matrix<2, 2> eye2;
    eye2(0, 0) = 1.0f;
    eye2(0, 1) = 0.0f;
    eye2(1, 0) = 0.0f;
    eye2(1, 1) = 1.0f;

    Matrix<2, 2> Ad = eye2 + kf_A_cont * dt_s;

    Matrix<2, 1> Bd;
    Bd(0, 0) = kf_B_cont(0, 0) * dt_s;
    Bd(1, 0) = kf_B_cont(1, 0) * dt_s;

    Matrix<2, 2> Qu;
    Qu(0, 0) = kf_sig1 * kf_sig1 * dt_s;
    Qu(0, 1) = 0.0f;
    Qu(1, 0) = 0.0f;
    Qu(1, 1) = kf_sig2 * kf_sig2 * dt_s;

    Matrix<2, 1> mu_p = Ad * kf_mu + Bd * u;
    Matrix<2, 2> sig_p = Ad * kf_sigma * ~Ad + Qu;

    if (has_meas) {
        float sm = (kf_C * sig_p * ~kf_C)(0, 0) + kf_sig3 * kf_sig3;
        if (fabsf(sm) < 1e-6f) sm = 1e-6f;

        float y_m = z - (kf_C * mu_p)(0, 0);
        float gate_mm = fmaxf(KF_GATE_MIN_MM, 3.0f * sqrtf(sm));
        kf_last_innovation_mm = y_m;
        kf_last_gate_mm = gate_mm;

        if (fabsf(y_m) > gate_mm) {
            kf_last_meas_rejected = true;
            kf_mu = mu_p;
            kf_sigma = sig_p;
            return kf_mu(0, 0);
        }

        Matrix<2, 1> tmp = sig_p * ~kf_C;
        float inv_sm = 1.0f / sm;
        Matrix<2, 1> K;
        K(0, 0) = tmp(0, 0) * inv_sm;
        K(1, 0) = tmp(1, 0) * inv_sm;

        kf_mu(0, 0) = mu_p(0, 0) + K(0, 0) * y_m;
        kf_mu(1, 0) = mu_p(1, 0) + K(1, 0) * y_m;
        kf_sigma = (eye2 - K * kf_C) * sig_p;
        kf_last_meas_used = true;
    } else {
        kf_mu = mu_p;
        kf_sigma = sig_p;
    }

    return kf_mu(0, 0);
}

void handle_pid() {
    if (runMode != RUN_PID_LINEAR && runMode != RUN_PID_KF) return;

    unsigned long now = millis();
    if (now - pid_start_ms >= pid_timeout_ms) {
        stop_active_drive_run(STOP_TIMEOUT, true);
        Serial.println("PID timeout — hard stop");
        return;
    }

    bool got_new_tof = false;
    float new_dist = 0.0f;
    unsigned long t_new = 0;
    int raw_dbg_mm = -1;
    uint8_t dbg_flags = 0;

    if (read_front_tof_sample(new_dist, t_new)) {
        got_new_tof = true;
        raw_dbg_mm = (int)new_dist;
        dbg_flags |= KF_FLAG_REAL_TOF;
        log_range_trace((int)new_dist, RANGE_REAL, t_new);

        if (new_dist <= (float)(pid_setpoint + PID_SAFETY_MARGIN_MM)) {
            tof_current = new_dist;
            dbg_flags |= KF_FLAG_SAFETY_STOP;

            float est_vel = pidUsesKalman() && kf_initialized
                ? kf_mu(1, 0)
                : tof_slope * 1000.0f;
            append_kf_debug_row(raw_dbg_mm, tof_current, est_vel,
                                tof_current - (float)pid_setpoint,
                                0, dbg_flags, t_new);
            kf_dbg_last_log_ms = t_new;

            stop_active_drive_run(STOP_SAFETY, true);
            emit_pid_safety_stop((int)new_dist, t_new);
            Serial.print("PID safety stop at ");
            Serial.print(new_dist);
            Serial.println(" mm");
            return;
        }

        if (!pidUsesKalman()) {
            if (tof_extrap_valid) {
                float dt_tof = (float)(t_new - tof_last_t_ms);
                if (dt_tof > 0.5f) {
                    tof_slope = (new_dist - tof_last_val) / dt_tof;
                }
            }
            tof_last_val = new_dist;
            tof_last_t_ms = t_new;
            tof_extrap_valid = true;
            tof_current = new_dist;
        }
    }

    if (!pidUsesKalman()) {
        if (!got_new_tof) {
            if (!tof_extrap_valid) return;
            unsigned long t_now = millis();
            float dt_since = (float)(t_now - tof_last_t_ms);
            tof_current = tof_last_val + tof_slope * dt_since;
            log_range_trace((int)tof_current, RANGE_EXTRAP, t_now);
        }
    } else {
        if (!kf_initialized) {
            if (!got_new_tof) return;
            kf_init(new_dist);
            kf_prev_t_ms = millis();
            tof_current = new_dist;
        } else {
            unsigned long t_now = millis();
            float dt_s = (float)(t_now - kf_prev_t_ms) / 1000.0f;
            if (dt_s < 0.0001f) dt_s = 0.001f;
            kf_prev_t_ms = t_now;

            float kf_pos = kf_step_fn(dt_s, kf_last_u, got_new_tof, new_dist);
            tof_current = kf_pos;
            if (kf_last_meas_used) dbg_flags |= KF_FLAG_KF_UPDATE;
            if (kf_last_meas_rejected) dbg_flags |= KF_FLAG_KF_REJECT;

            if (!got_new_tof) {
                log_range_trace((int)kf_pos, RANGE_KF_PRED, t_now);
            }
        }
    }

    now = millis();
    float e = 0.0f;
    int pwm = 0;
    if (!step_distance_pid(tof_current, now, e, pwm)) return;

    apply_drive_pwm_signed(pwm);

    if (pidUsesKalman()) {
        update_kf_control_input(pwm);
    }

    if (pid_e_pos < PID_LENGTH) {
        pid_e_hist[pid_e_pos] = (int16_t)e;
        pid_motor_hist[pid_e_pos] = (int16_t)pwm;
        pid_t_hist[pid_e_pos] = now;
        pid_e_pos++;
    }

    if (kf_dbg_last_log_ms == 0 ||
        now - kf_dbg_last_log_ms >= KF_DEBUG_INTERVAL_MS ||
        dbg_flags != 0) {
        float est_vel = pidUsesKalman() && kf_initialized
            ? kf_mu(1, 0)
            : tof_slope * 1000.0f;
        append_kf_debug_row(raw_dbg_mm, tof_current, est_vel, e, pwm, dbg_flags, now);
        kf_dbg_last_log_ms = now;
    }
}

void handle_orient_pid() {
    if (runMode != RUN_ORIENT) return;
    unsigned long now = millis();
    if (now - orient_start_ms >= orient_timeout_ms) {
        stop_active_drive_run(STOP_TIMEOUT, true);
        Serial.println("Orientation PID timeout");
        return;
    }

    float e = 0.0f;
    int pwm = 0;
    step_orient_pid(e, pwm, now, true);
}

bool drift_update_distance_estimate(unsigned long now_ms, float &est_dist, int &raw_mm) {
    float raw_dist = 0.0f;
    unsigned long t_new = 0;
    bool got_tof = read_front_tof_sample(raw_dist, t_new);
    raw_mm = got_tof ? (int)raw_dist : -1;

    if (!kf_initialized) {
        if (!got_tof) return false;
        kf_init(raw_dist);
        kf_prev_t_ms = now_ms;
        est_dist = raw_dist;
        return true;
    }

    float dt_s = (float)(now_ms - kf_prev_t_ms) / 1000.0f;
    if (dt_s < 0.0001f) dt_s = 0.001f;
    kf_prev_t_ms = now_ms;
    est_dist = kf_step_fn(dt_s, kf_last_u, got_tof, raw_dist);
    return true;
}

float drift_control_distance_mm(float est_dist, int raw_mm) {
    return (raw_mm > 0) ? (float)raw_mm : est_dist;
}

void start_drift_run() {
    if (!imuDmpReady) {
        motorsStop();
        runMode = RUN_IDLE;
        tx_characteristic_string.writeValue("DRIFT_DMP_ERR");
        Serial.println("Drift start rejected: DMP not ready");
        return;
    }

    stop_active_drive_run(STOP_MODE_SWITCH, true);
    collectingTOF = false;
    collectingIMU = false;
    drift_log_pos = 0;
    drift_phase = 0;
    drift_approach_heading_ref = 0.0f;
    kf_initialized = false;
    kf_last_u = 0.0f;
    kf_step_pwm_val = drift_approach_pwm;
    pid_setpoint = (int)lroundf(drift_stop_dist);
    reset_imu_filters();
    driftDmpHeadingDeg = 0.0f;
    driftDmpHeadingZeroDeg = 0.0f;
    driftDmpHeadingValid = false;
    driftDmpZeroSet = false;
    driftDmpHeadingAccuracy = -1;
    driftDmpHeadingTsMs = 0;
    if (imuDmpReady) {
        myICM.resetFIFO();
    }
    unsigned long now_ms = millis();
    reset_distance_pid_state(now_ms);
    reset_orient_pid_state(now_ms, false);
    drift_return_start_ms = 0;
    drift_rotate_done_count = 0;
    drift_start_ms = now_ms;
    runMode = RUN_DRIFT;
}

void stream_drift_history() {
    Serial.print("Sending drift rows=");
    Serial.println(drift_log_pos);

    for (int i = 0; i < drift_log_pos; i++) {
        tx_estring_value.clear();
        tx_estring_value.append("DRF|");
        tx_estring_value.append((int)drift_raw_hist[i]);
        tx_estring_value.append("|");
        tx_estring_value.append((int)drift_est_hist[i]);
        tx_estring_value.append("|");
        tx_estring_value.append((int)drift_yaw_hist[i]);
        tx_estring_value.append("|");
        tx_estring_value.append((int)drift_mot_hist[i]);
        tx_estring_value.append("|");
        tx_estring_value.append((int)drift_phase_hist[i]);
        tx_estring_value.append("|");
        tx_estring_value.append((int)drift_heading_err_hist[i]);
        tx_estring_value.append("|");
        tx_estring_value.append((int)drift_gyro_hist[i]);
        tx_estring_value.append("|");
        tx_estring_value.append((int)drift_t_hist[i]);
        ble_write_reliable(tx_estring_value.c_str());
    }

    tx_estring_value.clear();
    tx_estring_value.append("DRF_END|");
    tx_estring_value.append(drift_log_pos);
    ble_write_reliable(tx_estring_value.c_str());
}

void append_map_log(float target_deg, float heading_deg, int front_mm, int right_mm,
                    unsigned long ts_ms) {
    if (map_log_pos >= MAP_LOG_LEN) return;
    map_target_hist[map_log_pos] = clamp_i16(target_deg * 10.0f);
    map_heading_hist[map_log_pos] = clamp_i16(heading_deg * 10.0f);
    map_front_hist[map_log_pos] = clamp_i16((float)front_mm);
    map_right_hist[map_log_pos] = clamp_i16((float)right_mm);
    map_t_hist[map_log_pos] = ts_ms;
    map_log_pos++;
}

void stream_map_history() {
    Serial.print("Sending map rows=");
    Serial.println(map_log_pos);

    for (int i = 0; i < map_log_pos; i++) {
        tx_estring_value.clear();
        tx_estring_value.append("MAP|");
        tx_estring_value.append((int)map_target_hist[i]);
        tx_estring_value.append("|");
        tx_estring_value.append((int)map_heading_hist[i]);
        tx_estring_value.append("|");
        tx_estring_value.append((int)map_front_hist[i]);
        tx_estring_value.append("|");
        tx_estring_value.append((int)map_right_hist[i]);
        tx_estring_value.append("|");
        tx_estring_value.append((int)map_t_hist[i]);
        ble_write_reliable(tx_estring_value.c_str());
    }

    tx_estring_value.clear();
    tx_estring_value.append("MAP_END|");
    tx_estring_value.append(map_log_pos);
    ble_write_reliable(tx_estring_value.c_str());
}

void start_map_run() {
    if (!imuDmpReady) {
        motorsStop();
        runMode = RUN_IDLE;
        tx_characteristic_string.writeValue("MAP_DMP_ERR");
        Serial.println("Map start rejected: DMP not ready");
        return;
    }

    stop_active_drive_run(STOP_MODE_SWITCH, true);
    collectingTOF = false;
    collectingIMU = false;
    map_log_pos = 0;
    map_phase = 0;
    map_sample_idx = 0;
    map_done_count = 0;
    map_start_ms = millis();
    map_sample_wait_start_ms = 0;
    orient_target_deg = 0.0f;
    reset_imu_filters();
    driftDmpHeadingDeg = 0.0f;
    driftDmpHeadingZeroDeg = 0.0f;
    driftDmpHeadingValid = false;
    driftDmpZeroSet = false;
    driftDmpHeadingAccuracy = -1;
    driftDmpHeadingTsMs = 0;
    if (imuDmpReady) {
        myICM.resetFIFO();
    }
    reset_orient_pid_state(map_start_ms, false);
    runMode = RUN_MAP;
}

bool read_map_tof_sample(int &front_mm, int &right_mm, unsigned long &ts_ms) {
    float right_dist = 0.0f;
    if (!read_right_tof_sample(right_dist, ts_ms)) return false;

    front_mm = -1;
    right_mm = (int)right_dist;
    return true;
}

void finish_map_sample(float heading_deg, int front_mm, int right_mm, unsigned long ts_ms) {
    float target_deg = (float)(map_turn_dir * map_sample_idx * map_step_deg);
    append_map_log(target_deg, heading_deg, front_mm, right_mm, ts_ms);

    map_sample_idx++;
    if (map_sample_idx >= map_samples_goal || map_log_pos >= MAP_LOG_LEN) {
        motorsStop();
        map_phase = 2;
        runMode = RUN_IDLE;
        tx_estring_value.clear();
        tx_estring_value.append("MAP_DONE|");
        tx_estring_value.append(map_log_pos);
        ble_write_reliable(tx_estring_value.c_str());
        return;
    }

    orient_target_deg = wrap_angle_deg((float)(map_turn_dir * map_sample_idx * map_step_deg));
    reset_orient_pid_state(ts_ms, false);
    map_done_count = 0;
    map_phase = 0;
}

void handle_map() {
    if (runMode != RUN_MAP) return;

    unsigned long now = millis();
    if (now - map_start_ms >= map_timeout_ms) {
        motorsStop();
        runMode = RUN_IDLE;
        tx_characteristic_string.writeValue("MAP_TIMEOUT");
        return;
    }

    float heading_deg = 0.0f;
    float gyro_dps = imu_last_gyr_z;
    unsigned long heading_ts = now;
    if (!update_drift_heading_state(heading_deg, gyro_dps, heading_ts)) return;

    if (map_phase == 0) {
        float e = 0.0f;
        int pwm = 0;
        if (!step_orient_pid_with_heading(heading_deg, e, pwm, heading_ts, false)) return;

        bool settled = fabsf(e) <= MAP_DONE_BAND_DEG && fabsf(gyro_dps) <= MAP_DONE_GYRO_DPS;
        map_done_count = settled ? (map_done_count + 1) : 0;

        if (map_done_count >= MAP_DONE_COUNT) {
            motorsStop();
            map_sample_wait_start_ms = heading_ts;
            map_phase = 1;
        }
        return;
    }

    if (map_phase == 1) {
        motorsStop();

        int front_mm = -1;
        int right_mm = -1;
        unsigned long tof_ts = heading_ts;
        if (read_map_tof_sample(front_mm, right_mm, tof_ts)) {
            finish_map_sample(heading_deg, front_mm, right_mm, tof_ts);
            return;
        }

        if (now - map_sample_wait_start_ms >= MAP_SAMPLE_WAIT_MS) {
            finish_map_sample(heading_deg, -1, -1, heading_ts);
        }
    }
}

void handle_drift() {
    if (runMode != RUN_DRIFT) return;

    unsigned long now = millis();

    // Global timeout
    if (now - drift_start_ms > drift_timeout_ms) {
        motorsStop();
        runMode = RUN_IDLE;
        tx_characteristic_string.writeValue("DRIFT_TIMEOUT");
        return;
    }

    if (drift_phase == 0) {
        motorsForward(drift_approach_pwm);
        update_imu_state();
        float heading_deg = driftDmpHeadingValid ? driftDmpHeadingDeg : 0.0f;
        float gyro_dps = imu_last_gyr_z;
        unsigned long heading_ts = imuSampleTimeMs > 0 ? imuSampleTimeMs : now;

        float est_dist = 0.0f;
        int raw_mm = -1;
        if (!drift_update_distance_estimate(now, est_dist, raw_mm)) return;
        float control_dist = drift_control_distance_mm(est_dist, raw_mm);

        update_kf_control_input(drift_approach_pwm);
        append_drift_log(raw_mm, est_dist, heading_deg, drift_approach_pwm, 0, 0.0f, gyro_dps, heading_ts);

        if (control_dist <= drift_trigger_dist) {
            if (!update_drift_heading_state(heading_deg, gyro_dps, heading_ts)) return;
            drift_approach_heading_ref = heading_deg;
            orient_target_deg = wrap_angle_deg(drift_approach_heading_ref + 180.0f);
            kf_last_u = 0.0f;
            reset_orient_pid_state(heading_ts, false);
            drift_phase = 1;
        }
        return;
    }

    if (drift_phase == 1) {
        float heading_deg = 0.0f;
        float gyro_dps = imu_last_gyr_z;
        unsigned long heading_ts = now;
        if (!update_drift_heading_state(heading_deg, gyro_dps, heading_ts)) return;

        int pwm = 0;
        float e = 0.0f;
        if (!step_orient_pid_with_heading(heading_deg, e, pwm, heading_ts, false)) return;

        append_drift_log(-1, kf_initialized ? kf_mu(0, 0) : -1.0f,
                         heading_deg, pwm, 1, e, gyro_dps, heading_ts);

        float turn_progress = fabsf(wrap_angle_deg(heading_deg - drift_approach_heading_ref));
        bool rotate_done = turn_progress >= DRIFT_TURN_PROGRESS_MIN &&
                           fabsf(e) <= DRIFT_ROTATE_DONE_BAND_DEG;

        if (rotate_done) {
            drift_rotate_done_count++;
        } else {
            drift_rotate_done_count = 0;
        }

        if (drift_rotate_done_count >= DRIFT_ROTATE_DONE_COUNT) {
            motorsStop();
            drift_return_start_ms = heading_ts;
            drift_phase = 2;
        }
        return;
    }

    if (drift_phase == 2) {
        float heading_deg = 0.0f;
        float gyro_dps = imu_last_gyr_z;
        unsigned long heading_ts = now;
        if (!update_drift_heading_state(heading_deg, gyro_dps, heading_ts)) return;

        float heading_err = wrap_angle_deg(orient_target_deg - heading_deg);
        int steer_bias = (int)lroundf(drift_return_yaw_kp * heading_err);
        steer_bias = constrain(steer_bias, -DRIFT_RETURN_STEER_MAX, DRIFT_RETURN_STEER_MAX);
        motorsForwardSteered(drift_return_pwm, steer_bias);

        append_drift_log(-1, -1.0f, heading_deg, steer_bias, 2, heading_err, gyro_dps, heading_ts);

        if (now - drift_return_start_ms >= drift_return_ms) {
            motorsStop();
            drift_phase = 3;
            runMode = RUN_IDLE;
            tx_characteristic_string.writeValue("DRIFT_DONE");
        }
        return;
    }
}

void handle_kf_step() {
    if (runMode != RUN_KF_STEP) return;

    unsigned long now = millis();
    if (now < kf_step_kick_until_ms) {
        motorsForward(max(kf_step_pwm_val, KF_STEP_KICK_PWM));
    } else {
        motorsForward(kf_step_pwm_val);
    }

    if (now - kf_step_start_ms >= kf_step_timeout) {
        stop_active_drive_run(STOP_TIMEOUT, true);
        stream_pid_history();
        tx_estring_value.clear();
        tx_estring_value.append("KF_STEP_END|");
        tx_estring_value.append(range_trace_pos);
        ble_write_reliable(tx_estring_value.c_str());
        return;
    }

    float dist = 0.0f;
    unsigned long t_new = 0;
    if (!read_front_tof_sample(dist, t_new)) return;

    log_range_trace((int)dist, RANGE_REAL, t_new);
    if (dist <= (float)kf_step_min_dist) {
        stop_active_drive_run(STOP_SAFETY, true);
        stream_pid_history();
        tx_estring_value.clear();
        tx_estring_value.append("KF_STEP_END|");
        tx_estring_value.append(range_trace_pos);
        ble_write_reliable(tx_estring_value.c_str());
    }
}
