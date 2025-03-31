#ifndef MOTORS_H
#define MOTORS_H

// Motor Pins
#define MOTOR_A_1 11  // Right Backward
#define MOTOR_A_2 10  // Right Forward
#define MOTOR_B_1 9   // Left Backward
#define MOTOR_B_2 8   // Left Forward

#define MOTOR_DEVIATION       70        // Correction for motor speed imbalance
#define MOTOR_SPEED           180       // Motor forward and backward speed

#define ENCODER_R1            2         // Left Wheel Sensor
#define ENCODER_R2            3         // Right Wheel Sensor

// Wheel & Encoder Properties
const float WHEEL_DIAMETER = 0.065;     // 65mm = 0.065m
const int PPR = 20;                     // Pulses per revolution
const float WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * 3.14159;

// Movement Variables
const int P_MOVE = (0.50 / WHEEL_CIRCUMFERENCE) * PPR;  // Pulses for 0.5m
const int P_TURN_90 = PPR / 2.4;         // Pulses for 90-degree turn
const int P_TURN_180 = PPR;            // Pulses for 180-degree turn
const int P_MOVE_30CM = 30;            // Pulses for 30cm movement
const int P_MOVE_40CM = 40;            // Pulses for 40cm movement
const int P_MOVE_10CM = 10;            // Pulses for 10cm movement
const int P_MOVE_25CM = 25;            // Pulses for 25cm movement

// External variables
extern volatile int pulseCountR1;
extern volatile int pulseCountR2;
// PID control variables
extern float Kp;
extern float Ki;
extern float Kd;
extern int error;
extern int lastError;
extern int integral;
extern int derivative;
extern int correction;

// Function declarations
void setupMotors();
void moveForward();
void moveBackward();
void turnLeft90();
void turnRight90();
void turnAround();
void stopMotors();
void moveForward30cm();
void moveForward40cm();
void moveBackward10cm();
void moveForward25cm();
void executeStartSequence();
void followLine();
void moveForwardPID(int leftSpeed, int rightSpeed, bool withOutLine, bool lineTracking);
int getLinePosition();

#endif 