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
