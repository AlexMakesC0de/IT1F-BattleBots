#ifndef TURN_RIGHT_H
#define TURN_RIGHT_H

#include <Arduino.h>

// Function Prototype
void turnRight90();

// Turn Right Function
void turnRight90() {
    Serial.println("Turning Right 90°...");
    pulseCountR1 = 0;
    pulseCountR2 = 0;

    int baseSpeed = 180;
    int slowSpeed = 120;
    int correctionThreshold = 5;

    while (pulseCountR1 < P_TURN && pulseCountR2 < P_TURN) {
        int remainingPulses = P_TURN - max(pulseCountR1, pulseCountR2);
        int speed = (remainingPulses < 10) ? slowSpeed : baseSpeed;

        // Apply correction if one wheel moves faster
        if (pulseCountR1 > pulseCountR2 + correctionThreshold) {
            analogWrite(MOTOR_A1, speed - 30);
            analogWrite(MOTOR_A2, 0);
            analogWrite(MOTOR_B1, speed);
            analogWrite(MOTOR_B2, 0);
        } else if (pulseCountR2 > pulseCountR1 + correctionThreshold) {
            analogWrite(MOTOR_A1, speed);
            analogWrite(MOTOR_A2, 0);
            analogWrite(MOTOR_B1, speed - 30);
            analogWrite(MOTOR_B2, 0);
        } else {
            analogWrite(MOTOR_A1, speed);  // Left forward
            analogWrite(MOTOR_A2, 0);
            analogWrite(MOTOR_B1, 0);      // Right backward
            analogWrite(MOTOR_B2, speed);
        }
    }

    stopMotors();
}

#endif // TURN_RIGHT_H
