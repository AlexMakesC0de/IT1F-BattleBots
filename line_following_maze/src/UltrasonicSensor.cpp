#include <Arduino.h>
#include "../include/UltrasonicSensor.h"

unsigned long distanceMillis = 0;
float duration = 0;
float currentDistance = 0;

void setupUltrasonicSensor() {
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
}

void updateDistance() {
    // Clear the trigger pin
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);

    // Set the trigger pin HIGH for 10 microseconds
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    // Read the echo pin, return the sound wave travel time in microseconds
    duration = pulseIn(ECHO_PIN, HIGH);

    // Calculate the distance
    currentDistance = duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)
}

float getDistance() {
    return currentDistance;
} 