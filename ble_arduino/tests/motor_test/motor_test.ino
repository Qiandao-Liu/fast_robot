// Motor test: forward 5s → stop 1s → backward 5s → stop 1s → repeat
//
// Wiring (DRV8833, parallel-coupled channels):
//   Left  motor: AIN1 = pin 3  (forward), AIN2 = pin 14 (backward)
//   Right motor: BIN1 = pin 16 (forward), BIN2 = pin 15 (backward)

#define L_FWD  3
#define L_BWD  14
#define R_FWD  16
#define R_BWD  15

#define SPEED  150   // PWM 0-255; tune as needed

void motorsForward(int speed) {
    analogWrite(L_FWD, speed);
    analogWrite(R_FWD, speed);
    analogWrite(L_BWD, 0);
    analogWrite(R_BWD, 0);
}

void motorsBackward(int speed) {
    analogWrite(L_BWD, speed);
    analogWrite(R_BWD, speed);
    analogWrite(L_FWD, 0);
    analogWrite(R_FWD, 0);
}

void motorsStop() {
    analogWrite(L_FWD, 0);
    analogWrite(L_BWD, 0);
    analogWrite(R_FWD, 0);
    analogWrite(R_BWD, 0);
}

void setup() {
    Serial.begin(115200);
    pinMode(L_FWD, OUTPUT);
    pinMode(L_BWD, OUTPUT);
    pinMode(R_FWD, OUTPUT);
    pinMode(R_BWD, OUTPUT);
    motorsStop();
    Serial.println("Motor test ready");
}

void loop() {
    Serial.println("Forward");
    motorsForward(SPEED);
    delay(5000);

    motorsStop();
    delay(1000);

    Serial.println("Backward");
    motorsBackward(SPEED);
    delay(5000);

    motorsStop();
    delay(1000);
}


// void loop() {
//     // 30% duty cycle
//     analogWrite(L_FWD, 75);
//     analogWrite(R_FWD, 75);

// }

