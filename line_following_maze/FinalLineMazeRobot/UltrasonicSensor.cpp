#include "UltrasonicSensor.h"

// Set up ultrasonic sensor pins
void setupUltrasonicSensor() {
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
}

// Measure distance with timeout and rate limiting
float getDistance() {
  static unsigned long lastReadTime = 0;   // Time of last measurement
  static float lastMeasurement = -1;       // Last valid distance reading
  unsigned long currentTime = millis();    // Current time
  
  // Only take new readings every 200ms to avoid interference
  if (currentTime - lastReadTime >= 200) {
    lastReadTime = currentTime;            // Update timestamp
    
    // Clear trigger pin
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);                  // Short delay for clean signal
    
    // Send 10μs pulse to trigger measurement
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);                 // Pulse duration
    digitalWrite(TRIG_PIN, LOW);
    
    // Measure time for echo to return (timeout after 30ms)
    long duration = pulseIn(ECHO_PIN, HIGH, 30000);
    
    // Convert time to distance (speed of sound = 0.034 cm/μs)
    // Divide by 2 because sound travels to object and back
    float distance = duration * 0.034 / 2;
    
    // Only accept readings within valid range
    if (distance > 0 && distance < MAX_DISTANCE) {
      lastMeasurement = distance;          // Store valid reading
    } else {
      lastMeasurement = -1;                // Invalid reading
    }
  }
  
  // Return the most recent valid measurement (or -1 if invalid)
  return lastMeasurement;
} 