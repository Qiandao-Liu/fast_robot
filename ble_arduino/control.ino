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
    lastIMUTime = millis();
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
    pid_I = 0.0f;
    pid_dF = 0.0f;
    pid_last_e = 0.0f;
    tof_extrap_valid = false;
    tof_current = -1.0f;
    tof_slope = 0.0f;
    tof_last_val = 0.0f;
    tof_last_t_ms = 0;
    kf_initialized = false;
    kf_last_u = 0.0f;
    pid_start_ms = millis();
    pid_last_t = pid_start_ms;
    runMode = use_kf ? RUN_PID_KF : RUN_PID_LINEAR;
}

void start_orient_run() {
    stop_active_drive_run(STOP_MODE_SWITCH, true);
    collectingTOF = false;
    collectingIMU = false;
    orient_pos = 0;
    orient_I = 0.0f;
    orient_dF = 0.0f;
    orient_last_e = 0.0f;
    orient_last_pwm_sign = 0;
    orient_kick_until_ms = 0;
    reset_imu_filters();
    orient_start_ms = millis();
    orient_last_t = orient_start_ms;
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
    int dt = (int)(now - pid_last_t);
    if (dt < 1) return;
    pid_last_t = now;

    float e = tof_current - (float)pid_setpoint;
    pid_I += e * (float)dt / 1000.0f;
    if (pid_I > 1000.0f) pid_I = 1000.0f;
    if (pid_I < -1000.0f) pid_I = -1000.0f;

    float d_raw = (e - pid_last_e) / ((float)dt / 1000.0f);
    pid_dF = 0.9f * pid_dF + 0.1f * d_raw;
    pid_last_e = e;

    float output = pid_kp * e + pid_ki * pid_I + pid_kd * pid_dF;
    output = constrain(output, -(float)PWM_MAX, (float)PWM_MAX);

    int pwm = 0;
    if (fabsf(output) > 2.0f) {
        float abs_out = fabsf(output);
        float mapped = DEADBAND_MIN + (abs_out / (float)PWM_MAX) * (float)(PWM_MAX - DEADBAND_MIN);
        mapped = constrain(mapped, (float)DEADBAND_MIN, (float)PWM_MAX);
        pwm = (int)mapped;

        if (output > 0.0f && !pid_safety_stop_latched) {
            motorsForward(pwm);
        } else if (output <= 0.0f) {
            motorsBackward(pwm);
            pwm = -pwm;
        } else {
            motorsStop();
            pwm = 0;
        }
    } else {
        motorsStop();
        pwm = 0;
    }

    if (pidUsesKalman()) {
        float norm_base = (float)max(1, abs(kf_step_pwm_val));
        kf_last_u = (pwm != 0) ? -(float)pwm / norm_base : 0.0f;
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
    if (!update_imu_state()) return;

    unsigned long now = imuSampleTimeMs;
    if (now - orient_start_ms >= orient_timeout_ms) {
        stop_active_drive_run(STOP_TIMEOUT, true);
        Serial.println("Orientation PID timeout");
        return;
    }

    float dt = (float)(now - orient_last_t) / 1000.0f;
    if (dt <= 0.0005f) return;
    orient_last_t = now;

    yaw_g = wrap_angle_deg(yaw_g - imu_last_gyr_z * dt);
    float e = wrap_angle_deg(orient_target_deg - yaw_g);

    orient_I += e * dt;
    if (orient_I > 100.0f) orient_I = 100.0f;
    if (orient_I < -100.0f) orient_I = -100.0f;

    float d_raw = (e - orient_last_e) / dt;
    orient_dF = 0.9f * orient_dF + 0.1f * d_raw;
    orient_last_e = e;

    float output = orient_kp * e + orient_ki * orient_I + orient_kd * orient_dF;
    output = constrain(output, -(float)TURN_PWM_MAX, (float)TURN_PWM_MAX);

    int pwm = 0;
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
            now >= orient_kick_until_ms) {
            orient_kick_until_ms = now + TURN_KICK_MS;
        }

        if (now < orient_kick_until_ms) {
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

    if (orient_pos < ORIENT_LENGTH) {
        orient_yaw_hist[orient_pos] = (int16_t)lroundf(yaw_g * 10.0f);
        orient_e_hist[orient_pos] = (int16_t)lroundf(e * 10.0f);
        orient_motor_hist[orient_pos] = (int16_t)pwm;
        orient_t_hist[orient_pos] = now;
        orient_pos++;
    }
}

void start_drift_run() {
    stop_active_drive_run(STOP_MODE_SWITCH, true);
    collectingTOF = false;
    collectingIMU = false;
    drift_log_pos = 0;
    drift_phase = 0;
    drift_yaw_start = 0.0f;
    // Reset KF
    kf_initialized = false;
    kf_last_u = 0.0f;
    kf_step_pwm_val = drift_approach_pwm;
    // Reset yaw
    reset_imu_filters();
    // Reset orient PID state (reuse globals for rotation phase)
    orient_I = 0.0f;
    orient_dF = 0.0f;
    orient_last_e = 0.0f;
    orient_last_pwm_sign = 0;
    orient_kick_until_ms = 0;
    drift_brake_start_ms  = 0;
    drift_return_start_ms = 0;
    memset(drift_e_window, 0, sizeof(drift_e_window));
    drift_e_win_idx = 0;
    drift_start_ms = millis();
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
        tx_estring_value.append((int)drift_t_hist[i]);
        ble_write_reliable(tx_estring_value.c_str());
    }

    tx_estring_value.clear();
    tx_estring_value.append("DRF_END|");
    tx_estring_value.append(drift_log_pos);
    ble_write_reliable(tx_estring_value.c_str());
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

    // ── Phase 0: Approach ────────────────────────────────────────
    if (drift_phase == 0) {
        update_imu_state();

        float new_dist = 0.0f;
        unsigned long t_new = 0;
        bool got_tof = read_front_tof_sample(new_dist, t_new);

        if (!kf_initialized) {
            motorsForward(drift_approach_pwm);
            if (!got_tof) return;
            kf_init(new_dist);
            kf_prev_t_ms = now;
        }

        float dt_s = (float)(now - kf_prev_t_ms) / 1000.0f;
        if (dt_s < 0.0001f) dt_s = 0.001f;
        kf_prev_t_ms = now;
        float est_dist = kf_step_fn(dt_s, kf_last_u, got_tof, new_dist);

        motorsForward(drift_approach_pwm);
        kf_last_u = -(float)drift_approach_pwm / (float)max(1, kf_step_pwm_val);

        if (drift_log_pos < DRIFT_LOG_LEN) {
            drift_raw_hist[drift_log_pos]   = got_tof ? (int16_t)new_dist : -1;
            drift_est_hist[drift_log_pos]   = (int16_t)est_dist;
            drift_yaw_hist[drift_log_pos]   = (int16_t)lroundf(yaw_g * 10.0f);
            drift_mot_hist[drift_log_pos]   = (int16_t)drift_approach_pwm;
            drift_phase_hist[drift_log_pos] = 0;
            drift_t_hist[drift_log_pos]     = now;
            drift_log_pos++;
        }

        if (est_dist <= drift_brake_dist) {
            // Lock exact 180° target from current heading BEFORE braking starts
            drift_yaw_start   = yaw_g;
            orient_target_deg = wrap_angle_deg(yaw_g + 180.0f);
            drift_brake_start_ms = now;
            drift_phase = 1;
        }
        return;
    }

    // ── Phase 1: Brake ───────────────────────────────────────────
    if (drift_phase == 1) {
        motorsBackward(drift_brake_pwm);
        update_imu_state();

        if (drift_log_pos < DRIFT_LOG_LEN) {
            drift_raw_hist[drift_log_pos]   = -1;
            drift_est_hist[drift_log_pos]   = (int16_t)kf_mu(0, 0);
            drift_yaw_hist[drift_log_pos]   = (int16_t)lroundf(yaw_g * 10.0f);
            drift_mot_hist[drift_log_pos]   = -(int16_t)drift_brake_pwm;
            drift_phase_hist[drift_log_pos] = 1;
            drift_t_hist[drift_log_pos]     = now;
            drift_log_pos++;
        }

        if (now - drift_brake_start_ms >= drift_brake_ms) {
            motorsStop();
            // Reset orient PID state cleanly before rotation begins
            orient_I = 0.0f;
            orient_dF = 0.0f;
            orient_last_e = 0.0f;
            orient_last_pwm_sign = 0;
            orient_kick_until_ms = 0;
            orient_last_t = imuSampleTimeMs;
            kf_last_u = 0.0f;
            drift_phase = 2;
        }
        return;
    }

    // ── Phase 2: Rotate 180° ─────────────────────────────────────
    if (drift_phase == 2) {
        if (!update_imu_state()) return;

        unsigned long imu_now = imuSampleTimeMs;
        float dt = (float)(imu_now - orient_last_t) / 1000.0f;
        if (dt <= 0.0005f) return;
        orient_last_t = imu_now;

        yaw_g = wrap_angle_deg(yaw_g - imu_last_gyr_z * dt);
        float e = wrap_angle_deg(orient_target_deg - yaw_g);

        orient_I += e * dt;
        if (orient_I >  100.0f) orient_I =  100.0f;
        if (orient_I < -100.0f) orient_I = -100.0f;

        float d_raw = (e - orient_last_e) / dt;
        orient_dF = 0.9f * orient_dF + 0.1f * d_raw;
        orient_last_e = e;

        float output = orient_kp * e + orient_ki * orient_I + orient_kd * orient_dF;
        output = constrain(output, -(float)TURN_PWM_MAX, (float)TURN_PWM_MAX);

        int pwm = 0;
        float abs_out = fabsf(output);
        if (abs_out >= 2.0f) {
            int desired_sign = (output > 0.0f) ? 1 : -1;
            float mapped = TURN_DEADBAND + (abs_out - 2.0f) * (float)(TURN_PWM_MAX - TURN_DEADBAND)
                           / (float)(TURN_PWM_MAX - 2);
            pwm = (int)constrain(mapped, (float)TURN_DEADBAND, (float)TURN_PWM_MAX);
            if ((orient_last_pwm_sign == 0 || desired_sign != orient_last_pwm_sign) &&
                imu_now >= orient_kick_until_ms) {
                orient_kick_until_ms = imu_now + TURN_KICK_MS;
            }
            if (imu_now < orient_kick_until_ms) pwm = TURN_KICK_PWM;
            if (desired_sign > 0) {
                motorsTurnRight(pwm);
            } else {
                motorsTurnLeft(pwm);
                pwm = -pwm;
            }
            orient_last_pwm_sign = desired_sign;
        } else {
            motorsStop();
            orient_last_pwm_sign = 0;
        }

        if (drift_log_pos < DRIFT_LOG_LEN) {
            drift_raw_hist[drift_log_pos]   = -1;
            drift_est_hist[drift_log_pos]   = (int16_t)kf_mu(0, 0);
            drift_yaw_hist[drift_log_pos]   = (int16_t)lroundf(yaw_g * 10.0f);
            drift_mot_hist[drift_log_pos]   = (int16_t)pwm;
            drift_phase_hist[drift_log_pos] = 2;
            drift_t_hist[drift_log_pos]     = now;
            drift_log_pos++;
        }

        // Rolling window exit: require last DRIFT_WIN_SIZE samples all < DRIFT_DONE_EACH
        // AND their mean < DRIFT_DONE_AVG — prevents premature exit during oscillation
        drift_e_window[drift_e_win_idx % DRIFT_WIN_SIZE] = e;
        drift_e_win_idx++;

        if (drift_e_win_idx >= DRIFT_WIN_SIZE) {
            float sum = 0.0f;
            bool all_close = true;
            for (int i = 0; i < DRIFT_WIN_SIZE; i++) {
                float ae = fabsf(drift_e_window[i]);
                sum += ae;
                if (ae >= DRIFT_DONE_EACH) all_close = false;
            }
            if (all_close && (sum / DRIFT_WIN_SIZE) < DRIFT_DONE_AVG) {
                motorsStop();
                drift_return_start_ms = now;
                drift_phase = 3;
            }
        }
        return;
    }

    // ── Phase 3: Return ──────────────────────────────────────────
    if (drift_phase == 3) {
        motorsForward(drift_return_pwm);
        update_imu_state();

        if (drift_log_pos < DRIFT_LOG_LEN) {
            drift_raw_hist[drift_log_pos]   = -1;
            drift_est_hist[drift_log_pos]   = -1;
            drift_yaw_hist[drift_log_pos]   = (int16_t)lroundf(yaw_g * 10.0f);
            drift_mot_hist[drift_log_pos]   = (int16_t)drift_return_pwm;
            drift_phase_hist[drift_log_pos] = 3;
            drift_t_hist[drift_log_pos]     = now;
            drift_log_pos++;
        }

        if (now - drift_return_start_ms >= drift_return_ms) {
            motorsStop();
            drift_phase = 4;
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
