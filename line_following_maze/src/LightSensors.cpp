#include <Arduino.h>
#include "../include/LightSensors.h"
#include "../include/Motors.h"

#define NUM_SENSORS 8  // Number of sensors

// Light sensor pins
const int LIGHT_SENSOR[NUM_SENSORS] = {A0, A1, A2, A3, A4, A5, A6, A7};

int sensorValues[NUM_SENSORS];  // Array to store sensor readings
int sensorMin[NUM_SENSORS];     // Minimum values for calibration
int sensorMax[NUM_SENSORS];     // Maximum values for calibration
int sensorThreshold[NUM_SENSORS]; // Individual thresholds for each sensor
int linePosition = 3500;         // Default position (center)
bool onLine = false;             // Flag to indicate if line is detected
bool sensorsCalibrated = false;  // Track if calibration is complete

// Global threshold value for backward compatibility
int dynamicThreshold = 750; // Start with default value

void setupLightSensors() {
    // Initialize analog pins as inputs
    for (int i = 0; i < NUM_SENSORS; i++) {
        pinMode(LIGHT_SENSOR[i], INPUT);
        // Initialize min/max values
        sensorMin[i] = 1023;  // Set min to max ADC value
        sensorMax[i] = 0;     // Set max to min ADC value
    }
}

void readSensors() {
    // Read all sensors
    for (int i = 0; i < NUM_SENSORS; i++) {
        sensorValues[i] = analogRead(LIGHT_SENSOR[i]);
    }
}

// Call this function repeatedly while the robot is moving
// to calibrate the sensors
void updateCalibration() {
    // Read current sensor values
    readSensors();
    
    // Update min/max values for each sensor
    for (int i = 0; i < NUM_SENSORS; i++) {
        if (sensorValues[i] < sensorMin[i]) {
            sensorMin[i] = sensorValues[i];
        }
        if (sensorValues[i] > sensorMax[i]) {
            sensorMax[i] = sensorValues[i];
        }
    }
}

// Function to finalize calibration - call when robot has traversed
// both line and non-line surfaces
void finalizeCalibration() {
    // Calculate threshold values for each sensor
    for (int i = 0; i < NUM_SENSORS; i++) {
        // Check if we have valid min/max values with sufficient difference
        if (sensorMax[i] - sensorMin[i] > 50) {
            sensorThreshold[i] = (sensorMin[i] + sensorMax[i]) / 2;
        } else {
            // Fall back to a reasonable default
            sensorThreshold[i] = 750;
        }
        
        // Debug output
        Serial.print("Sensor ");
        Serial.print(i);
        Serial.print(" calibration - Min: ");
        Serial.print(sensorMin[i]);
        Serial.print(", Max: ");
        Serial.print(sensorMax[i]);
        Serial.print(", Threshold: ");
        Serial.println(sensorThreshold[i]);
    }
    
    // Calculate an average threshold for backward compatibility
    int total = 0;
    for (int i = 0; i < NUM_SENSORS; i++) {
        total += sensorThreshold[i];
    }
    dynamicThreshold = total / NUM_SENSORS;
    
    sensorsCalibrated = true;
    Serial.println("Sensor calibration complete!");
    Serial.print("Average threshold: ");
    Serial.println(dynamicThreshold);
}

// Main calibration function to be called in executeStartSequence
void calibrateSensors() {
    // Reset calibration variables
    sensorsCalibrated = false;
    for (int i = 0; i < NUM_SENSORS; i++) {
        sensorMin[i] = 1023;
        sensorMax[i] = 0;
    }
    
    Serial.println("Starting sensor calibration...");
    
    // Reset encoder counts
    pulseCountR1 = 0;
    pulseCountR2 = 0;
    
    // Define calibration distance (adjust as needed)
    int calibrationDistance = 25; // About 20cm
    
    // Move forward while collecting calibration data
    while (pulseCountR1 < calibrationDistance && pulseCountR2 < calibrationDistance) {
        // Drive motors
        analogWrite(MOTOR_A_2, MOTOR_SPEED + MOTOR_DEVIATION); // Right Forward
        analogWrite(MOTOR_A_1, 0);
        analogWrite(MOTOR_B_2, MOTOR_SPEED); // Left Forward
        analogWrite(MOTOR_B_1, 0);
        
        // Update calibration with current readings
        updateCalibration();
        
        // Small delay to avoid overwhelming readings
        delay(10);
    }
    
    // Stop motors
    analogWrite(MOTOR_A_1, 0);
    analogWrite(MOTOR_A_2, 0);
    analogWrite(MOTOR_B_1, 0);
    analogWrite(MOTOR_B_2, 0);
    
    // Finalize calibration
    finalizeCalibration();
}

// Function to check if a specific reading is above its sensor's threshold
bool isBlack(int sensorIndex, int reading) {
    return reading > sensorThreshold[sensorIndex];
}

// Backward compatible function for previous isBlack usage
bool isBlack(int reading) {
    return reading > dynamicThreshold;
}

void printLightSensorValues() {
    // Print header
    Serial.println("Light Sensor Values:");
    
    // Read and print values from all 8 sensors
    for (int i = 0; i < NUM_SENSORS; i++) {
        int sensorValue = analogRead(LIGHT_SENSOR[i]);
        Serial.print("Sensor ");
        Serial.print(i);
        Serial.print(": ");
        Serial.print(sensorValue);
        Serial.print(" (");
        Serial.print(isBlack(i, sensorValue) ? "Black" : "White");
        Serial.println(")");
    }
    Serial.println(); // Add blank line between readings
}

void determineLinePosition() {
    readSensors();
    
    // Variables for weighted average calculation
    float weightedSum = 0;
    int sum = 0;
    
    // Print sensor values for debugging
    Serial.print("Sensors: ");
    
    for (int i = 0; i < NUM_SENSORS; i++) {
        Serial.print(sensorValues[i]);
        Serial.print("\t");
        
        // Check if sensor detects line using individual thresholds
        if (isBlack(i, sensorValues[i])) {
            weightedSum += i * 1000;  // Multiply by position (0-7000)
            sum += 1;
        }
    }
    Serial.println();
    
    // Calculate line position if at least one sensor detects the line
    if (sum > 0) {
        linePosition = weightedSum / sum;
        onLine = true;
        
        // Print position
        Serial.print("Line Position: ");
        Serial.println(linePosition);
    } else {
        onLine = false;
        Serial.println("No line detected!");
    }
}

// Function to calculate error for PID control
int getLineError() {
    determineLinePosition();
    
    // If no line detected, return last known position's error
    if (!onLine) {
        // Return a large error if no line, based on last known position
        if (linePosition < 3500) {
            return -2000;  // Line was last seen on the left, hard turn left
        } else {
            return 2000;   // Line was last seen on the right, hard turn right
        }
    }
    
    // Calculate error from center (3500)
    return linePosition - 3500;
}

void getSensorCombinations() {
    readSensors();
    
    Serial.println("Sensor Combinations:");
    
    // Print raw sensor values
    Serial.print("Raw: ");
    for (int i = 0; i < NUM_SENSORS; i++) {
        Serial.print(sensorValues[i]);
        Serial.print(" ");
    }
    Serial.println();
    
    // Print binary representation (black/white)
    Serial.print("B/W: ");
    for (int i = 0; i < NUM_SENSORS; i++) {
        Serial.print(isBlack(i, sensorValues[i]) ? "B" : "W");
        Serial.print(" ");
    }
    Serial.println();
    
    // Calculate and print line position
    determineLinePosition();
    
    // Calculate and print error
    int error = getLineError();
    Serial.print("Error: ");
    Serial.println(error);
    
    Serial.println();  // Extra line for readability
}