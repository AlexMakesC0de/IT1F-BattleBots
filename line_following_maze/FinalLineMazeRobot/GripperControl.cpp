#include "GripperControl.h"

void setupGripper() {
  pinMode(SERVO, OUTPUT);
  digitalWrite(SERVO, LOW);
}

void gripper(int pulse) {
  static unsigned long timer;
  static int lastPulse;
  if (millis() > timer) {
    if (pulse > 0) {
      lastPulse = pulse;
    } else {
      pulse = lastPulse;
    }

    digitalWrite(SERVO, HIGH);
    delayMicroseconds(pulse);
    digitalWrite(SERVO, LOW);
    timer = millis() + 20;
  }
} 