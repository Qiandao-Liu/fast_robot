float normalize_dmp_heading_deg(float deg) {
    while (deg > 180.0f) deg -= 360.0f;
    while (deg < -180.0f) deg += 360.0f;
    return deg;
}

float quat9_to_yaw_deg(const icm_20948_DMP_data_t &data) {
    double q1 = ((double)data.Quat9.Data.Q1) / 1073741824.0;
    double q2 = ((double)data.Quat9.Data.Q2) / 1073741824.0;
    double q3 = ((double)data.Quat9.Data.Q3) / 1073741824.0;
    double q0_sq = 1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3));
    if (q0_sq < 0.0) q0_sq = 0.0;
    double q0 = sqrt(q0_sq);

    // Map the chip frame to aircraft frame before extracting yaw.
    double qw = q0;
    double qx = q2;
    double qy = q1;
    double qz = -q3;

    double t3 = +2.0 * (qw * qz + qx * qy);
    double t4 = +1.0 - 2.0 * (qy * qy + qz * qz);
    return (float)(atan2(t3, t4) * 180.0 / PI);
}

bool init_imu_dmp() {
    if (!imuInitialized) return false;

    bool success = true;
    success &= (myICM.initializeDMP() == ICM_20948_Stat_Ok);
    success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_ORIENTATION) == ICM_20948_Stat_Ok);
    success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Quat9, 0) == ICM_20948_Stat_Ok);
    success &= (myICM.enableFIFO() == ICM_20948_Stat_Ok);
    success &= (myICM.enableDMP() == ICM_20948_Stat_Ok);
    success &= (myICM.resetDMP() == ICM_20948_Stat_Ok);
    success &= (myICM.resetFIFO() == ICM_20948_Stat_Ok);

    if (!success) {
        Serial.print("IMU DMP init failed: ");
        Serial.println(myICM.statusString());
        return false;
    }

    Serial.println("IMU DMP enabled");
    return true;
}

bool poll_drift_dmp_heading() {
    if (!imuDmpReady) return false;

    icm_20948_DMP_data_t data;
    icm_20948_DMP_data_t latestQuat;
    bool gotQuat = false;

    while (true) {
        myICM.readDMPdataFromFIFO(&data);
        ICM_20948_Status_e status = myICM.status;

        if (status == ICM_20948_Stat_Ok || status == ICM_20948_Stat_FIFOMoreDataAvail) {
            if ((data.header & DMP_header_bitmap_Quat9) > 0) {
                latestQuat = data;
                gotQuat = true;
            }

            if (status != ICM_20948_Stat_FIFOMoreDataAvail) {
                break;
            }
            continue;
        }

        if (status == ICM_20948_Stat_FIFONoDataAvail) {
            break;
        }

        if (status == ICM_20948_Stat_FIFOIncompleteData) {
            continue;
        }

        Serial.print("DMP FIFO read failed: ");
        Serial.println(myICM.statusString());
        break;
    }

    if (!gotQuat) {
        return driftDmpHeadingValid;
    }

    float yaw_deg = quat9_to_yaw_deg(latestQuat);
    if (!driftDmpZeroSet) {
        driftDmpHeadingZeroDeg = yaw_deg;
        driftDmpZeroSet = true;
    }

    driftDmpHeadingDeg = normalize_dmp_heading_deg(yaw_deg - driftDmpHeadingZeroDeg);
    driftDmpHeadingTsMs = millis();
    driftDmpHeadingAccuracy = latestQuat.Quat9.Data.Accuracy;
    driftDmpHeadingValid = true;
    return true;
}

bool update_drift_heading_state(float &heading_deg, float &gyro_dps, unsigned long &ts_ms) {
    update_imu_state();
    gyro_dps = imu_last_gyr_z;

    if (!poll_drift_dmp_heading()) {
        return false;
    }

    heading_deg = driftDmpHeadingDeg;
    ts_ms = driftDmpHeadingTsMs;
    return true;
}

bool update_imu_state() {
    if (!imuInitialized || !myICM.dataReady()) return false;

    myICM.getAGMT();
    unsigned long now = millis();
    float dt = (float)(now - lastIMUTime) / 1000.0f;
    if (dt <= 0.0f) dt = 0.001f;
    lastIMUTime = now;

    float ax = myICM.accX();
    float ay = myICM.accY();
    float az = myICM.accZ();
    float gx = myICM.gyrX();
    float gy = myICM.gyrY();
    float gz = myICM.gyrZ();

    imu_pitch_a_raw = atan2(ax, sqrt(ay * ay + az * az)) * 180.0f / M_PI;
    imu_roll_a_raw = atan2(ay, sqrt(ax * ax + az * az)) * 180.0f / M_PI;

    pitch_a_lpf = alpha_lpf * imu_pitch_a_raw + (1.0f - alpha_lpf) * pitch_a_lpf;
    roll_a_lpf = alpha_lpf * imu_roll_a_raw + (1.0f - alpha_lpf) * roll_a_lpf;

    pitch_g += gx * dt;
    roll_g += gy * dt;
    imu_last_gyr_z = gz;
    imuSampleTimeMs = now;

    pitch_comp = (1.0f - alpha_comp) * (pitch_comp + gx * dt) + alpha_comp * imu_pitch_a_raw;
    roll_comp = (1.0f - alpha_comp) * (roll_comp + gy * dt) + alpha_comp * imu_roll_a_raw;

    return true;
}

void record_imu_data() {
    if (imuArrayFull) return;
    if (!update_imu_state()) return;

    imuTimeStamps[imuIndex] = imuSampleTimeMs;
    imuPitchA[imuIndex] = imu_pitch_a_raw;
    imuRollA[imuIndex] = imu_roll_a_raw;
    imuPitchComp[imuIndex] = pitch_comp;
    imuRollComp[imuIndex] = roll_comp;
    imuIndex++;

    if (imuIndex >= MAX_IMU_SIZE) {
        imuArrayFull = true;
        collectingIMU = false;
        Serial.println("IMU array full, auto-stopped");
    }
}

bool read_front_tof_sample(float &dist_mm, unsigned long &ts_ms) {
    if (!tof1Ready || !tofSensor1.checkForDataReady()) return false;
    dist_mm = (float)tofSensor1.getDistance();
    tofSensor1.clearInterrupt();
    ts_ms = millis();
    return true;
}

bool read_right_tof_sample(float &dist_mm, unsigned long &ts_ms) {
    if (!tof2Ready || !tofSensor2.checkForDataReady()) return false;
    dist_mm = (float)tofSensor2.getDistance();
    tofSensor2.clearInterrupt();
    ts_ms = millis();
    return true;
}

void record_tof_data() {
    float dist1 = 0.0f;
    unsigned long ts_ms = 0;
    if (tofArrayFull) return;
    if (!read_front_tof_sample(dist1, ts_ms)) return;

    tofTimeStamps[tofIndex] = ts_ms;
    tofDist1[tofIndex] = clamp_i16(dist1);

    if (tof2Ready && tofSensor2.checkForDataReady()) {
        tofDist2[tofIndex] = (int16_t)tofSensor2.getDistance();
        tofSensor2.clearInterrupt();
    } else {
        tofDist2[tofIndex] = -1;
    }

    tofIndex++;
    if (tofIndex >= MAX_TOF_SIZE) {
        tofArrayFull = true;
        collectingTOF = false;
        Serial.println("ToF array full, auto-stopped");
    }
}

void init_tof_sensors() {
    digitalWrite(SHUTDOWN_PIN_1, HIGH);
    delay(10);

    if (tofSensor1.begin() == 0) {
        tofSensor1.setI2CAddress(TOF1_ADDR);
        tofSensor1.setDistanceModeLong();
        tofSensor1.startRanging();
        tof1Ready = true;
        Serial.println("ToF sensor 1 OK");
    } else {
        Serial.println("ToF sensor 1 FAILED");
    }

    digitalWrite(SHUTDOWN_PIN, HIGH);
    delay(10);

    if (tofSensor2.begin() == 0) {
        tofSensor2.setDistanceModeLong();
        tofSensor2.startRanging();
        tof2Ready = true;
        Serial.println("ToF sensor 2 OK");
    } else {
        Serial.println("ToF sensor 2 FAILED");
    }
}
