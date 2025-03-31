#include <Arduino.h>
#include "../include/Gripper.h"

static unsigned long timer = 0;
static int lastPulse = GRIPPER_OPEN;

void setupGripper() {
    pinMode(GRIPPER_PIN, OUTPUT);
    digitalWrite(GRIPPER_PIN, LOW);
}

void gripper(int pulse) {
    static unsigned long timer = 0;
    static int lastPulse;
    unsigned long currentTime = millis();

    if (pulse > 0) {
        lastPulse = pulse;
    } else {
        pulse = lastPulse;
    }

    if (currentTime >= timer) {
        digitalWrite(GRIPPER_PIN, HIGH);
        delayMicroseconds(pulse);
        digitalWrite(GRIPPER_PIN, LOW);
        timer = currentTime + SERVO_INTERVAL;
    }
}

void openGripper() {
    // Send multiple pulses to ensure the servo reaches the position
    for(int i = 0; i < 25; i++) {
        gripper(GRIPPER_OPEN);
        delay(20);
    }
}

void closeGripper() {
    // Send multiple pulses to ensure the servo reaches the position
    for(int i = 0; i < 25; i++) {
        gripper(GRIPPER_CLOSED);
        delay(20);
    }
} 