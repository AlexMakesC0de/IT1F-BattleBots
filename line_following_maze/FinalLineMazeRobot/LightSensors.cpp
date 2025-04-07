#include "LightSensors.h"
#include "MotorControl.h"

// Read all sensor values into the global array
void readSensors() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    sensorValues[i] = analogRead(sensorPins[i]);  // Read raw analog value (0-1023)
  }
}

// Calibrate sensors by finding min/max values as robot moves
void calibrateSensors() {
  static bool firstRun = true;  // Only initialize values on first call
  readSensors();  // Get current readings

  // On first run, set default min/max values
  if (firstRun) {
    for (int i = 0; i < NUM_SENSORS; i++) {
      sensorMin[i] = 1023;  // Start with max possible value (will decrease)
      sensorMax[i] = 0;     // Start with min possible value (will increase)
    }
    firstRun = false;
  }

  // Check if we've moved far enough for calibration
  if (_leftTicks > DISTANCE_FROM_BASE_TO_CONE || _rightTicks > DISTANCE_FROM_BASE_TO_CONE) {
    stopMotors();  // Stop once calibration distance reached
    sensorsCalibrated = true;  // Mark as calibrated

    // Calculate threshold as average of min and max for each sensor
    for (int i = 0; i < NUM_SENSORS; i++) {
      sensorThreshold[i] = (sensorMin[i] + sensorMax[i]) / 2;  // Midpoint as threshold
      // Print debug values for each sensor
      Serial.print("Threshold [");
      Serial.print(i);
      Serial.print("]: ");
      Serial.println(sensorThreshold[i]);
      Serial.println(sensorMin[i]);
      Serial.println(sensorMax[i]);
    }
    return;
  }

  // Update min/max values for each sensor
  for (int i = 0; i < NUM_SENSORS; i++) {
    int sensorValue = analogRead(sensorPins[i]);
    // Keep track of minimum seen
    if (sensorValue < sensorMin[i]) {
      sensorMin[i] = sensorValue;
    }
    // Keep track of maximum seen
    if (sensorValue > sensorMax[i]) {
      sensorMax[i] = sensorValue;
    }
  }

  // Keep moving forward to collect more calibration data
  moveForwardPID(200, 200, true, false);  // Move with encoder feedback, no line tracking
}

// Calculate weighted position of line under sensors
int calculateLinePosition() {
  long weightedSum = 0;  // Sum of (sensor reading × position)
  long sum = 0;          // Sum of all sensor readings

  // Calculate weighted average of sensor readings
  for (int i = 0; i < NUM_SENSORS; i++) {
    int value = sensorValues[i];
    weightedSum += (long)value * i * 1000;  // Multiply by position (×1000 for precision)
    sum += value;                           // Add to total
  }

  return weightedSum / sum;  // Return weighted position (higher = more to the right)
} 