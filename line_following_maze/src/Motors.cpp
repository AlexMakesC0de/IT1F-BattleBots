#include <Arduino.h>
#include "../include/Motors.h"
#include "../include/Gripper.h"
#include "../include/LightSensors.h"

volatile int pulseCountR1 = 0;
volatile int pulseCountR2 = 0;

// PID control variables
float Kp = 0.4;  // Proportional Gain
float Ki = 0.00001;  // Integral Gain
float Kd = 0.1;  // Derivative Gain
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
        analogWrite(MOTOR_A_2, MOTOR_SPEED); // Right Forward
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
        analogWrite(MOTOR_A_1, MOTOR_SPEED); // Right Backward
        analogWrite(MOTOR_A_2, 0);
        analogWrite(MOTOR_B_1, MOTOR_SPEED); // Left Backward
        analogWrite(MOTOR_B_2, 0);
    }
    stopMotors();
}

void turnLeft90() {
    pulseCountR1 = 0;
    pulseCountR2 = 0;

    while (pulseCountR1 < P_TURN_90 && pulseCountR2 < P_TURN_90) {
        analogWrite(MOTOR_A_2, MOTOR_SPEED); // Right Forward
        analogWrite(MOTOR_A_1, 0);
        analogWrite(MOTOR_B_1, MOTOR_SPEED); // Left Backward
        analogWrite(MOTOR_B_2, 0);
    }
    stopMotors();
}

void turnRight90() {
    pulseCountR1 = 0;
    pulseCountR2 = 0;

    while (pulseCountR1 < P_TURN_90 && pulseCountR2 < P_TURN_90) {
        analogWrite(MOTOR_A_1, MOTOR_SPEED); // Right Backward
        analogWrite(MOTOR_A_2, 0);
        analogWrite(MOTOR_B_2, MOTOR_SPEED); // Left Forward
        analogWrite(MOTOR_B_1, 0);
    }
    stopMotors();
}

void turnAround() {
    pulseCountR1 = 0;
    pulseCountR2 = 0;

    while (pulseCountR1 < P_TURN_180 && pulseCountR2 < P_TURN_180) {
        analogWrite(MOTOR_A_1, MOTOR_SPEED); // Right Backward
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
        analogWrite(MOTOR_A_1, MOTOR_SPEED); // Right Backward
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

// Updated function to drive the robot using PID control
void moveForwardPID(int _leftSpeed, int _rightSpeed, bool withOutLine, bool lineTracking) {
  if (withOutLine) {
    // PID Variables for encoder-based straight-line control
    Kp = 6.5;  // Proportional Gain
    Ki = 0.1;  // Integral Gain
    Kd = 5;    // Derivative Gain

    // Calculate error based on encoder ticks
    error = pulseCountR1 - pulseCountR2;

    // Update PID terms
    integral += error;
    derivative = error - lastError;
    lastError = error;

    // Prevent integral windup
    integral = constrain(integral, -10, 10);

  } else if (lineTracking) {
    // Read sensors and calculate line position
    int position = getLinePosition();

    // PID Variables for line following
    Kp = 0.4;    // Proportional Gain
    Ki = 0.00001;  // Integral Gain
    Kd = 0.1;    // Derivative Gain

    int center = 3500;  // Center position (3500 for our 8-sensor array)
    
    // Calculate error
    error = position - center;

    // Update PID terms
    integral += error;
    derivative = error - lastError;
    lastError = error;   
  }

  // Apply PID correction
  correction = (Kp * error) + (Ki * integral) + (Kd * derivative);
  
  // Adjust motor speeds based on correction
  _leftSpeed -= correction;
  _rightSpeed += correction;

  // Ensure PWM values are within valid range (0 - 255)
  _leftSpeed = constrain(_leftSpeed, 0, MOTOR_SPEED);
  _rightSpeed = constrain(_rightSpeed, 0, MOTOR_SPEED);

  // Drive motors
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

// Updated followLine function that uses PID control
void followLine() {
  // Base speed for motors
  int baseSpeed = 140;
  
  // Use PID control for line following
  moveForwardPID(baseSpeed, baseSpeed, false, true);
} 