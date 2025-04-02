#include <Arduino.h>
#include "../include/Motors.h"
#include "../include/Gripper.h"
#include "../include/LightSensors.h"

// Define NUM_SENSORS to match LightSensors.cpp
#define NUM_SENSORS 8  // Number of sensors

// External declarations for variables in other files
extern int sensorValues[];  // Declared in LightSensors.cpp

volatile int pulseCountR1 = 0;
volatile int pulseCountR2 = 0;

// PID control variables
float Kp = 0.2;  // Reduced from 0.4 to minimize zigzagging
float Ki = 0.00001;  // Removed integral component temporarily 
float Kd = 0.3;  // Increased from 0.1 to dampen oscillations
int error = 0;
int lastError = 0;
int integral = 0;
int derivative = 0;
int correction = 0;

void countPulseR1() { pulseCountR1++; }
void countPulseR2() { pulseCountR2++; }

void setupMotors() {
    pinMode(MOTOR_A_1, OUTPUT);
    pinMode(MOTOR_A_2, OUTPUT);
    pinMode(MOTOR_B_1, OUTPUT);
    pinMode(MOTOR_B_2, OUTPUT);

    pinMode(ENCODER_R1, INPUT);
    pinMode(ENCODER_R2, INPUT);

    attachInterrupt(digitalPinToInterrupt(ENCODER_R1), countPulseR1, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCODER_R2), countPulseR2, RISING);
}

void moveForward() {
    pulseCountR1 = 0;
    pulseCountR2 = 0;

    while (pulseCountR1 < P_MOVE && pulseCountR2 < P_MOVE) {
        analogWrite(MOTOR_A_2, MOTOR_SPEED + MOTOR_DEVIATION); // Right Forward
        analogWrite(MOTOR_A_1, 0);
        analogWrite(MOTOR_B_2, MOTOR_SPEED + MOTOR_DEVIATION); // Left Forward (slightly faster)
        analogWrite(MOTOR_B_1, 0);
    }
    stopMotors();
}

void moveBackward() {
    pulseCountR1 = 0;
    pulseCountR2 = 0;

    while (pulseCountR1 < P_MOVE && pulseCountR2 < P_MOVE) {
        analogWrite(MOTOR_A_1, MOTOR_SPEED + MOTOR_DEVIATION); // Right Backward
        analogWrite(MOTOR_A_2, 0);
        analogWrite(MOTOR_B_1, MOTOR_SPEED); // Left Backward
        analogWrite(MOTOR_B_2, 0);
    }
    stopMotors();
}

void turnLeft90() {
    pulseCountR1 = 0;
    pulseCountR2 = 0;

    

    Serial.println("Turning LEFT 90 degrees");
    
    while (pulseCountR1 < P_TURN_90 && pulseCountR2 < P_TURN_90) {
        analogWrite(MOTOR_A_2, TURN_SPEED + MOTOR_DEVIATION); // Right Forward
        analogWrite(MOTOR_A_1, 0);
        analogWrite(MOTOR_B_1, TURN_SPEED); // Left Backward
        analogWrite(MOTOR_B_2, 0);
    }
    stopMotors();
}

void turnRight90() {
    pulseCountR1 = 0;
    pulseCountR2 = 0;

    Serial.println("Turning RIGHT 90 degrees");
    
    while (pulseCountR1 < P_TURN_90 && pulseCountR2 < P_TURN_90) {
        analogWrite(MOTOR_A_1, TURN_SPEED + MOTOR_DEVIATION); // Right Backward
        analogWrite(MOTOR_A_2, 0);
        analogWrite(MOTOR_B_2, TURN_SPEED); // Left Forward
        analogWrite(MOTOR_B_1, 0);
    }
    stopMotors();
}

void turnAround() {
    pulseCountR1 = 0;
    pulseCountR2 = 0;

    Serial.println("Turning AROUND 180 degrees");
    
    while (pulseCountR1 < P_TURN_180 && pulseCountR2 < P_TURN_180) {
        analogWrite(MOTOR_A_1, MOTOR_SPEED + MOTOR_DEVIATION); // Right Backward
        analogWrite(MOTOR_A_2, 0);
        analogWrite(MOTOR_B_2, MOTOR_SPEED); // Left Forward
        analogWrite(MOTOR_B_1, 0);
    }
    stopMotors();
}

void moveForward30cm() {
    pulseCountR1 = 0;
    pulseCountR2 = 0;

    while (pulseCountR1 < P_MOVE_30CM && pulseCountR2 < P_MOVE_30CM) {
        analogWrite(MOTOR_A_2, MOTOR_SPEED + MOTOR_DEVIATION); // Right Forward
        analogWrite(MOTOR_A_1, 0);
        analogWrite(MOTOR_B_2, MOTOR_SPEED); // Left Forward (slightly faster)
        analogWrite(MOTOR_B_1, 0);
    }
    stopMotors();
}

void moveBackward10cm() {
    pulseCountR1 = 0;
    pulseCountR2 = 0;

    while (pulseCountR1 < P_MOVE_10CM && pulseCountR2 < P_MOVE_10CM) {
        analogWrite(MOTOR_A_1, MOTOR_SPEED + MOTOR_DEVIATION); // Right Backward
        analogWrite(MOTOR_A_2, 0);
        analogWrite(MOTOR_B_1, MOTOR_SPEED); // Left Backward
        analogWrite(MOTOR_B_2, 0);
    }
    stopMotors();
}

void stopMotors() {
    analogWrite(MOTOR_A_1, 0);
    analogWrite(MOTOR_A_2, 0);
    analogWrite(MOTOR_B_1, 0);
    analogWrite(MOTOR_B_2, 0);
}

// Function to calculate line position from sensor readings
int calculateLinePosition() {
  long weightedSum = 0;
  long sum = 0;

  for (int i = 0; i < NUM_SENSORS; i++) {
    int value = sensorValues[i];
    weightedSum += (long)value * i * 1000;
    sum += value;
  }
  
  // Prevent division by zero
  if (sum == 0) return 3500; // Default to center if no line
  
  return weightedSum / sum;
}

// Updated function to drive the robot using PID control
void moveForwardPID(int _leftSpeed, int _rightSpeed, bool withOutLine, bool lineTracking) {
  if (withOutLine) {
    // PID Variables
    Kp = 6.5;  // Proportional Gain
    Ki = 0.1;  // Integral Gain
    Kd = 5;  // Derivative Gain

    // Calculate error
    error = pulseCountR1 - pulseCountR2;

    // Update PID terms
    integral += error;
    derivative = error - lastError;
    lastError = error;

    // Prevent integral windup
    integral = constrain(integral, -10, 10);

  } else if (lineTracking) {
    readSensors();
    int position = calculateLinePosition();

    // PID Variables
    Kp = 0.4;  // Proportional Gain
    Ki = 0.00001;  // Integral Gain
    Kd = 0.15;  // Derivative Gain

    int center = (NUM_SENSORS - 1) * 1000 / 2;  // Midpoint of sensor array
    // Calculate error
    error = position - center;

    // Update PID terms
    integral += error;
    derivative = error - lastError;
    lastError = error;   
  }

  // Apply PID correction
  correction = (Kp * error) + (Ki * integral) + (Kd * derivative);
  _leftSpeed -= correction;
  _rightSpeed += correction;

  // Ensure PWM values are within valid range (0 - 255)
  _leftSpeed = constrain(_leftSpeed, 0, 255);
  _rightSpeed = constrain(_rightSpeed, 0, 255);

  // Move motors directly as in the original implementation
  // Left motor
  analogWrite(MOTOR_B_2, _leftSpeed); // Left Forward
  analogWrite(MOTOR_B_1, 0);
  
  // Right motor
  analogWrite(MOTOR_A_2, _rightSpeed); // Right Forward
  analogWrite(MOTOR_A_1, 0);
}

// Function to get the current line position
int getLinePosition() {
  determineLinePosition();
  return linePosition;
}

// Updated followLine function that uses PID control and handles dead ends
void followLine() {
  // Base speed for motors
  int baseSpeed = 200;
  
  // Check if line is detected
  determineLinePosition();
  
  // Dead end detection - if no line is detected
  if (!onLine) {
    Serial.println("Possible dead end detected - no line");
    
    // Stop motors
    stopMotors();
    delay(200); // Short pause
    
    // Double-check by reading sensors again
    determineLinePosition();
    
    if (!onLine) {
      Serial.println("Dead end confirmed - turning around");
      // Turn around at dead end
      turnAround();
      delay(100);
      return;
    }
  }
  
  // Use PID control for line following
  moveForwardPID(baseSpeed, baseSpeed, false, true);
} 