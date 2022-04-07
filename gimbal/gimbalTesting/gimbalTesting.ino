// Gimbal calibration and testing

#include <Servo.h>

Servo rollServo, yawServo, pitchServo;

// Speed params
int step = 1, t = 15;

void setup() {
//    rollServo.attach(9);
    yawServo.attach(10);
    pitchServo.attach(11);
}

void loop() {
//    for (pos = 0; pos <= 180; pos += step) {
//        rollServo.write(pos);
//        delay(t);
//    }
//    for (pos = 180; pos >= 0; pos -= step) {
//        rollServo.write(pos);
//        delay(t);
//    }
    for (int i = 0; i <= 180; i += step) {
        yawServo.write(i);
        delay(t);
    }
    for (int i = 180; i >= 0; i -= step) {
        yawServo.write(i);
        delay(t);
    }
    for (int i = 0; i <= 180; i += step) {
        pitchServo.write(i);
        delay(t);
    }
    for (int i = 180; i >= 0; i -= step) {
        pitchServo.write(i);
        delay(t);
    }
}
