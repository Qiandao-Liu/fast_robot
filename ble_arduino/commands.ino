// Reliable BLE write with backpressure: retries until the BLE stack accepts
// the notification, so no messages are silently dropped.
bool ble_write_reliable(const char* str) {
    constexpr int MAX_RETRIES = 150;
    for (int i = 0; i < MAX_RETRIES; i++) {
        if (tx_characteristic_string.writeValue(str)) {
            BLE.poll();
            delay(20);
            return true;
        }
        BLE.poll();
        delay(5);
    }
    Serial.print("BLE write failed: ");
    Serial.println(str);
    return false;
}

void stream_pid_history() {
    Serial.print("Sending TOF=");
    Serial.print(range_trace_pos);
    Serial.print(" PID=");
    Serial.println(pid_e_pos);

    for (int i = 0; i < range_trace_pos; i++) {
        tx_estring_value.clear();
        tx_estring_value.append("TOF|");
        tx_estring_value.append((int)range_trace_dist[i]);
        tx_estring_value.append("|");
        tx_estring_value.append((int)range_trace_kind[i]);
        tx_estring_value.append("|");
        tx_estring_value.append((int)range_trace_t[i]);
        ble_write_reliable(tx_estring_value.c_str());
    }

    for (int i = 0; i < pid_e_pos; i++) {
        tx_estring_value.clear();
        tx_estring_value.append("PID|");
        tx_estring_value.append((int)pid_e_hist[i]);
        tx_estring_value.append("|");
        tx_estring_value.append((int)pid_motor_hist[i]);
        tx_estring_value.append("|");
        tx_estring_value.append((int)pid_t_hist[i]);
        ble_write_reliable(tx_estring_value.c_str());
    }

    tx_estring_value.clear();
    tx_estring_value.append("PID_END|");
    tx_estring_value.append(range_trace_pos);
    tx_estring_value.append("|");
    tx_estring_value.append(pid_e_pos);
    ble_write_reliable(tx_estring_value.c_str());
}

void stream_orient_history() {
    Serial.print("Sending orient samples=");
    Serial.println(orient_pos);

    for (int i = 0; i < orient_pos; i++) {
        tx_estring_value.clear();
        tx_estring_value.append("OPID|");
        tx_estring_value.append((int)orient_yaw_hist[i]);
        tx_estring_value.append("|");
        tx_estring_value.append((int)orient_e_hist[i]);
        tx_estring_value.append("|");
        tx_estring_value.append((int)orient_motor_hist[i]);
        tx_estring_value.append("|");
        tx_estring_value.append((int)orient_t_hist[i]);
        ble_write_reliable(tx_estring_value.c_str());
    }

    tx_estring_value.clear();
    tx_estring_value.append("OPID_END|");
    tx_estring_value.append(orient_pos);
    ble_write_reliable(tx_estring_value.c_str());
}

void stream_kf_debug() {
    Serial.print("Sending KF debug rows=");
    Serial.println(kf_dbg_pos);

    for (int i = 0; i < kf_dbg_pos; i++) {
        tx_estring_value.clear();
        tx_estring_value.append("DBG|");
        tx_estring_value.append((int)kf_dbg_raw_hist[i]);
        tx_estring_value.append("|");
        tx_estring_value.append((int)kf_dbg_est_hist[i]);
        tx_estring_value.append("|");
        tx_estring_value.append((int)kf_dbg_vel_hist[i]);
        tx_estring_value.append("|");
        tx_estring_value.append((int)kf_dbg_err_hist[i]);
        tx_estring_value.append("|");
        tx_estring_value.append((int)kf_dbg_pwm_hist[i]);
        tx_estring_value.append("|");
        tx_estring_value.append((int)kf_dbg_flags_hist[i]);
        tx_estring_value.append("|");
        tx_estring_value.append((int)kf_dbg_t_hist[i]);
        ble_write_reliable(tx_estring_value.c_str());
    }

    tx_estring_value.clear();
    tx_estring_value.append("DBG_END|");
    tx_estring_value.append(kf_dbg_pos);
    ble_write_reliable(tx_estring_value.c_str());
}

void handle_command() {
    robot_cmd.set_cmd_string(rx_characteristic_string.value(),
                             rx_characteristic_string.valueLength());

    int cmd_type = -1;
    if (!robot_cmd.get_command_type(cmd_type)) {
        return;
    }

    switch (cmd_type) {
        case PING:
            tx_estring_value.clear();
            tx_estring_value.append("PONG");
            tx_characteristic_string.writeValue(tx_estring_value.c_str());
            break;

        case GET_IMU_DATA:
        {
            if (!update_imu_state()) {
                tx_characteristic_string.writeValue("IMU_ERROR");
                break;
            }

            tx_estring_value.clear();
            tx_estring_value.append((int)imuSampleTimeMs);
            tx_estring_value.append("|");
            tx_estring_value.append(imu_pitch_a_raw);
            tx_estring_value.append("|");
            tx_estring_value.append(imu_roll_a_raw);
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

        case START_IMU_RECORDING:
            stop_active_drive_run(STOP_MODE_SWITCH, true);
            collectingTOF = false;
            collectingIMU = true;
            imuIndex = 0;
            imuArrayFull = false;
            reset_imu_filters();
            tx_characteristic_string.writeValue("IMU_REC_START");
            break;

        case STOP_IMU_RECORDING:
            collectingIMU = false;
            tx_estring_value.clear();
            tx_estring_value.append("IMU_REC_STOP|");
            tx_estring_value.append(imuIndex);
            tx_characteristic_string.writeValue(tx_estring_value.c_str());
            break;

        case SEND_IMU_DATA:
        {
            int limit = imuArrayFull ? MAX_IMU_SIZE : imuIndex;
            int step = 3;
            int sent = 0;

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
                delay(30);
                sent++;
            }

            tx_estring_value.clear();
            tx_estring_value.append("IMU_DATA_END|");
            tx_estring_value.append(sent);
            tx_characteristic_string.writeValue(tx_estring_value.c_str());
            break;
        }

        case SET_IMU_PARAMS:
        {
            float new_alpha_lpf, new_alpha_comp;
            if (!robot_cmd.get_next_value(new_alpha_lpf)) break;
            if (!robot_cmd.get_next_value(new_alpha_comp)) break;
            alpha_lpf = new_alpha_lpf;
            alpha_comp = new_alpha_comp;

            tx_estring_value.clear();
            tx_estring_value.append("PARAMS_SET|");
            tx_estring_value.append(alpha_lpf);
            tx_estring_value.append("|");
            tx_estring_value.append(alpha_comp);
            tx_characteristic_string.writeValue(tx_estring_value.c_str());
            break;
        }

        case RESET_IMU_ANGLES:
            reset_imu_filters();
            tx_characteristic_string.writeValue("ANGLES_RESET");
            break;

        case GET_TOF_DATA:
        {
            if (!tof1Ready && !tof2Ready) {
                tx_characteristic_string.writeValue("TOF_ERROR");
                break;
            }

            unsigned long t0 = millis();
            while (tof1Ready && !tofSensor1.checkForDataReady() && (millis() - t0 < 100)) {}
            int d1 = tof1Ready ? (int)tofSensor1.getDistance() : -1;
            if (tof1Ready) tofSensor1.clearInterrupt();

            t0 = millis();
            while (tof2Ready && !tofSensor2.checkForDataReady() && (millis() - t0 < 100)) {}
            int d2 = tof2Ready ? (int)tofSensor2.getDistance() : -1;
            if (tof2Ready) tofSensor2.clearInterrupt();

            tx_estring_value.clear();
            tx_estring_value.append(d1);
            tx_estring_value.append("|");
            tx_estring_value.append(d2);
            tx_characteristic_string.writeValue(tx_estring_value.c_str());
            break;
        }

        case START_TOF_RECORDING:
            stop_active_drive_run(STOP_MODE_SWITCH, true);
            collectingTOF = true;
            collectingIMU = true;
            tofIndex = 0;
            tofArrayFull = false;
            imuIndex = 0;
            imuArrayFull = false;
            reset_imu_filters();
            tx_characteristic_string.writeValue("TOF_REC_START");
            break;

        case STOP_TOF_RECORDING:
            collectingTOF = false;
            collectingIMU = false;
            tx_estring_value.clear();
            tx_estring_value.append("TOF_REC_STOP|");
            tx_estring_value.append(tofIndex);
            tx_estring_value.append("|");
            tx_estring_value.append(imuIndex);
            tx_characteristic_string.writeValue(tx_estring_value.c_str());
            break;

        case SEND_TOF_DATA:
        {
            int limit = tofArrayFull ? MAX_TOF_SIZE : tofIndex;
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
            break;
        }

        case SET_TOF_MODE:
        {
            int mode;
            if (!robot_cmd.get_next_value(mode)) break;

            if (tof1Ready) {
                tofSensor1.stopRanging();
                if (mode == 0) tofSensor1.setDistanceModeShort();
                else tofSensor1.setDistanceModeLong();
                tofSensor1.clearInterrupt();
                tofSensor1.startRanging();
            }
            if (tof2Ready) {
                tofSensor2.stopRanging();
                if (mode == 0) tofSensor2.setDistanceModeShort();
                else tofSensor2.setDistanceModeLong();
                tofSensor2.clearInterrupt();
                tofSensor2.startRanging();
            }

            tx_estring_value.clear();
            tx_estring_value.append("TOF_MODE|");
            tx_estring_value.append(mode == 0 ? "SHORT" : "LONG");
            tx_characteristic_string.writeValue(tx_estring_value.c_str());
            break;
        }

        case MOTOR_FORWARD:
        {
            int speed, duration;
            if (!robot_cmd.get_next_value(speed)) break;
            if (!robot_cmd.get_next_value(duration)) duration = 0;
            set_manual_motor(MOT_FWD, speed, duration, "MOTOR_FWD");
            break;
        }

        case MOTOR_BACKWARD:
        {
            int speed, duration;
            if (!robot_cmd.get_next_value(speed)) break;
            if (!robot_cmd.get_next_value(duration)) duration = 0;
            set_manual_motor(MOT_BWD, speed, duration, "MOTOR_BWD");
            break;
        }

        case MOTOR_STOP:
            stop_active_drive_run(STOP_CMD, true);
            tx_characteristic_string.writeValue("MOTOR_STOP");
            break;

        case MOTOR_TURN_LEFT:
        {
            int speed, duration;
            if (!robot_cmd.get_next_value(speed)) break;
            if (!robot_cmd.get_next_value(duration)) duration = 0;
            set_manual_motor(MOT_LEFT, speed, duration, "MOTOR_LEFT");
            break;
        }

        case MOTOR_TURN_RIGHT:
        {
            int speed, duration;
            if (!robot_cmd.get_next_value(speed)) break;
            if (!robot_cmd.get_next_value(duration)) duration = 0;
            set_manual_motor(MOT_RIGHT, speed, duration, "MOTOR_RIGHT");
            break;
        }

        case SET_MOTOR_CAL:
        {
            float cal;
            if (!robot_cmd.get_next_value(cal)) break;
            motorCalFactor = cal;
            tx_estring_value.clear();
            tx_estring_value.append("MOTOR_CAL|");
            tx_estring_value.append(motorCalFactor);
            tx_characteristic_string.writeValue(tx_estring_value.c_str());
            break;
        }

        case MOTOR_TURN_ANGLE:
        {
            int dir, angle;
            if (!robot_cmd.get_next_value(dir)) break;
            if (!robot_cmd.get_next_value(angle)) angle = 90;
            stop_active_drive_run(STOP_MODE_SWITCH, true);
            turnTargetDeg = (float)abs(angle);
            turnAccumDeg = 0.0f;
            lastTurnGyroTime = millis();
            if (dir == 0) motorsTurnLeft(140);
            else motorsTurnRight(140);
            motorMode = MOT_TURN_ANGLE;
            motorTimed = false;

            tx_estring_value.clear();
            tx_estring_value.append("TURN_START|");
            tx_estring_value.append(dir == 0 ? "L" : "R");
            tx_estring_value.append("|");
            tx_estring_value.append(angle);
            tx_characteristic_string.writeValue(tx_estring_value.c_str());
            break;
        }

        case PID_START:
            start_pid_run(false);
            tx_characteristic_string.writeValue("PID_START");
            break;

        case PID_STOP:
            stop_active_drive_run(STOP_CMD, true);
            tx_characteristic_string.writeValue("PID_STOP");
            break;

        case GET_PID_DATA:
            stream_pid_history();
            break;

        case SET_PID_GAINS:
        {
            float kp, ki, kd, sp;
            if (!robot_cmd.get_next_value(kp)) break;
            if (!robot_cmd.get_next_value(ki)) break;
            if (!robot_cmd.get_next_value(kd)) break;
            if (!robot_cmd.get_next_value(sp)) break;
            pid_kp = kp;
            pid_ki = ki;
            pid_kd = kd;
            pid_setpoint = (int)sp;

            tx_estring_value.clear();
            tx_estring_value.append("PID_GAINS|");
            tx_estring_value.append(pid_kp);
            tx_estring_value.append("|");
            tx_estring_value.append(pid_ki);
            tx_estring_value.append("|");
            tx_estring_value.append(pid_kd);
            tx_estring_value.append("|");
            tx_estring_value.append(pid_setpoint);
            tx_characteristic_string.writeValue(tx_estring_value.c_str());
            break;
        }

        case ORIENT_START:
            start_orient_run();
            tx_characteristic_string.writeValue("ORIENT_START");
            break;

        case ORIENT_STOP:
            stop_active_drive_run(STOP_CMD, true);
            tx_characteristic_string.writeValue("ORIENT_STOP");
            break;

        case GET_ORIENT_DATA:
            stream_orient_history();
            break;

        case SET_ORIENT_GAINS:
        {
            float kp, ki, kd;
            if (!robot_cmd.get_next_value(kp)) break;
            if (!robot_cmd.get_next_value(ki)) break;
            if (!robot_cmd.get_next_value(kd)) break;
            orient_kp = kp;
            orient_ki = ki;
            orient_kd = kd;

            tx_estring_value.clear();
            tx_estring_value.append("ORIENT_GAINS|");
            tx_estring_value.append(orient_kp);
            tx_estring_value.append("|");
            tx_estring_value.append(orient_ki);
            tx_estring_value.append("|");
            tx_estring_value.append(orient_kd);
            tx_characteristic_string.writeValue(tx_estring_value.c_str());
            break;
        }

        case SET_ORIENT_TARGET:
        {
            float target;
            if (!robot_cmd.get_next_value(target)) break;
            orient_target_deg = wrap_angle_deg(target);

            tx_estring_value.clear();
            tx_estring_value.append("ORIENT_TARGET|");
            tx_estring_value.append(orient_target_deg);
            tx_characteristic_string.writeValue(tx_estring_value.c_str());
            break;
        }

        case KF_STEP_START:
        {
            int pwm_val = 150;
            int min_dist = 400;
            robot_cmd.get_next_value(pwm_val);
            robot_cmd.get_next_value(min_dist);
            start_kf_step_run(pwm_val, min_dist);

            tx_estring_value.clear();
            tx_estring_value.append("KF_STEP_START|");
            tx_estring_value.append(kf_step_pwm_val);
            tx_estring_value.append("|kick=");
            tx_estring_value.append(KF_STEP_KICK_PWM);
            tx_estring_value.append("|");
            tx_estring_value.append(KF_STEP_KICK_MS);
            tx_estring_value.append("|stop=");
            tx_estring_value.append(kf_step_min_dist);
            tx_characteristic_string.writeValue(tx_estring_value.c_str());
            break;
        }

        case KF_STEP_STOP:
            if (runMode == RUN_KF_STEP) {
                stop_active_drive_run(STOP_CMD, true);
                stream_pid_history();
            }
            tx_estring_value.clear();
            tx_estring_value.append("KF_STEP_STOP|");
            tx_estring_value.append(range_trace_pos);
            tx_characteristic_string.writeValue(tx_estring_value.c_str());
            break;

        case SET_KF_PARAMS:
        {
            float d, m, s1, s2, s3;
            int step_pwm = 0;
            if (!robot_cmd.get_next_value(d)) break;
            if (!robot_cmd.get_next_value(m)) break;
            if (!robot_cmd.get_next_value(s1)) break;
            if (!robot_cmd.get_next_value(s2)) break;
            if (!robot_cmd.get_next_value(s3)) break;
            if (!robot_cmd.get_next_value(step_pwm)) break;

            kf_d = d;
            kf_m = m;
            kf_sig1 = s1;
            kf_sig2 = s2;
            kf_sig3 = s3;
            kf_step_pwm_val = max(1, step_pwm);
            kf_initialized = false;

            char ack_buf[MAX_MSG_SIZE];
            snprintf(ack_buf, sizeof(ack_buf),
                     "KF_PARAMS|d=%.7f|m=%.7f|s1=%.1f|s2=%.1f|s3=%.1f|step_pwm=%d",
                     kf_d, kf_m, kf_sig1, kf_sig2, kf_sig3, kf_step_pwm_val);
            tx_characteristic_string.writeValue(ack_buf);
            break;
        }

        case KF_PID_START:
            start_pid_run(true);
            tx_characteristic_string.writeValue("KF_PID_START");
            break;

        case GET_KF_DEBUG_DATA:
            stream_kf_debug();
            break;

        case DRIFT_START:
        {
            // params: approach_pwm|return_pwm|brake_dist|brake_pwm|brake_ms|return_ms|timeout_ms
            int ap = 200, rp = 200, bpwm = 200, bms = 250, ret_ms = 2000, tout = 8000;
            float bdist = 500.0f;
            robot_cmd.get_next_value(ap);
            robot_cmd.get_next_value(rp);
            robot_cmd.get_next_value(bdist);
            robot_cmd.get_next_value(bpwm);
            robot_cmd.get_next_value(bms);
            robot_cmd.get_next_value(ret_ms);
            robot_cmd.get_next_value(tout);
            drift_approach_pwm = constrain(ap, 0, 255);
            drift_return_pwm   = constrain(rp, 0, 255);
            drift_brake_dist   = bdist;
            drift_brake_pwm    = constrain(bpwm, 0, 255);
            drift_brake_ms     = (unsigned long)max(0, bms);
            drift_return_ms    = (unsigned long)max(0, ret_ms);
            drift_timeout_ms   = (unsigned long)max(1000, tout);
            start_drift_run();

            char ack_buf[MAX_MSG_SIZE];
            snprintf(ack_buf, sizeof(ack_buf),
                     "DRIFT_START|ap=%d|rp=%d|bdist=%.0f|bpwm=%d|bms=%lu|ret=%lu|tout=%lu",
                     drift_approach_pwm, drift_return_pwm,
                     drift_brake_dist, drift_brake_pwm, drift_brake_ms,
                     drift_return_ms, drift_timeout_ms);
            tx_characteristic_string.writeValue(ack_buf);
            break;
        }

        case DRIFT_STOP:
            stop_active_drive_run(STOP_CMD, true);
            tx_characteristic_string.writeValue("DRIFT_STOP");
            break;

        case GET_DRIFT_DATA:
            stream_drift_history();
            break;

        case SET_DRIFT_PARAMS:
        {
            // params: approach_pwm|return_pwm|brake_dist|brake_pwm|brake_ms|return_ms|timeout_ms
            int ap = 200, rp = 200, bpwm = 200, bms = 250, ret_ms = 2000, tout = 8000;
            float bdist = 500.0f;
            robot_cmd.get_next_value(ap);
            robot_cmd.get_next_value(rp);
            robot_cmd.get_next_value(bdist);
            robot_cmd.get_next_value(bpwm);
            robot_cmd.get_next_value(bms);
            robot_cmd.get_next_value(ret_ms);
            robot_cmd.get_next_value(tout);
            drift_approach_pwm = constrain(ap, 0, 255);
            drift_return_pwm   = constrain(rp, 0, 255);
            drift_brake_dist   = bdist;
            drift_brake_pwm    = constrain(bpwm, 0, 255);
            drift_brake_ms     = (unsigned long)max(0, bms);
            drift_return_ms    = (unsigned long)max(0, ret_ms);
            drift_timeout_ms   = (unsigned long)max(1000, tout);

            char ack_buf[MAX_MSG_SIZE];
            snprintf(ack_buf, sizeof(ack_buf),
                     "DRIFT_PARAMS|ap=%d|rp=%d|bdist=%.0f|bpwm=%d|bms=%lu|ret=%lu|tout=%lu",
                     drift_approach_pwm, drift_return_pwm,
                     drift_brake_dist, drift_brake_pwm, drift_brake_ms,
                     drift_return_ms, drift_timeout_ms);
            tx_characteristic_string.writeValue(ack_buf);
            break;
        }

        default:
            if (cmd_type >= CMD_RETIRED_1 && cmd_type <= CMD_RETIRED_10) {
                Serial.print("Retired command ID: ");
                Serial.println(cmd_type);
            } else {
                Serial.print("Invalid Command Type: ");
                Serial.println(cmd_type);
            }
            break;
    }
}
