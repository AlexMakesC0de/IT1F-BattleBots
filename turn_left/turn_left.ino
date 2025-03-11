#ifndef TURN_LEFT_H
#define TURN_LEFT_H

#include <Arduino.h>

// Function Prototype
void turnLeft90();

// Turn Left Function
void turnLeft90() {
    Serial.println("Turning Left 90°...");
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
            analogWrite(MOTOR_A1, 0);
            analogWrite(MOTOR_A2, speed - 30);
            analogWrite(MOTOR_B1, 0);
            analogWrite(MOTOR_B2, speed);
        } else if (pulseCountR2 > pulseCountR1 + correctionThreshold) {
            analogWrite(MOTOR_A1, 0);
            analogWrite(MOTOR_A2, speed);
            analogWrite(MOTOR_B1, 0);
            analogWrite(MOTOR_B2, speed - 30);
        } else {
            analogWrite(MOTOR_A1, 0);
            analogWrite(MOTOR_A2, speed); // Left backward
            analogWrite(MOTOR_B1, 0);
            analogWrite(MOTOR_B2, speed); // Right forward
        }
    }

    stopMotors();
}

#endif // TURN_LEFT_H
