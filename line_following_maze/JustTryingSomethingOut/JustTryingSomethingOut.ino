#include <Arduino.h>

enum RobotState { FOLLOW_LINE, TURNING, TURNING_LEFT, TURNING_RIGHT, TURNING_AROUND, CHECKING_FOR_PATH_AHEAD };
enum LinePosition { T_JUNCTION, LEFT_LINE, RIGHT_LINE, NO_LINE, CENTER_LINE };
RobotState robotState = FOLLOW_LINE;
LinePosition linePosition = CENTER_LINE;

//Line Sensors
#define NUM_SENSORS 8  // Number of sensors
int sensorPins[NUM_SENSORS] = {A0, A1, A2, A3, A4, A5, A6, A7};  // Sensor pin mapping
int sensorValues[NUM_SENSORS];  // Array to store sensor readings
int sensorMin[NUM_SENSORS];
int sensorMax[NUM_SENSORS];
int sensorThreshold[NUM_SENSORS]; // Array to store thresholds for each sensor

//Line Positions
bool leftTurn; 
bool rightTurn;
bool tJunctionOrBase;
bool deadEnd;

//Measurements
const float WHEEL_CIRCUMFERENCE = 20.4;
const int PULSE_PER_REVOLUTION = 20;
const float DISTANCE_BETWEEN_WHEELS = 22.75;
static const int DISTANCE_FROM_BASE_TO_CONE = 55; // Distance is in ticks
const int target = DISTANCE_FROM_BASE_TO_CONE;

// Encoder Pulse Counters
volatile signed int _leftTicks = 4;
volatile signed int _rightTicks = 0;

// Motor Pins
#define MOTOR_A_1 11  // Left Forward
#define MOTOR_A_2 10  // Left Reverse
#define MOTOR_B_1 9   // Right Forward
#define MOTOR_B_2 6   // Right Reverse
int baseSpeed = 180;  // Adjusted to match your speed

//Servo Control
#define GRIPPER_OPEN 1750  // Updated to match your project
#define GRIPPER_CLOSE 1220  // Updated to match your project
#define SERVO 4  // Changed to match your GRIPPER_PIN
const int pulse = 2000;
int previousTime = 0;
const int gripperInterval = 20;  // This already matches your SERVO_INTERVAL

//Rotation Sensors
#define MOTOR_R1 2  // Left Wheel Sensor
#define MOTOR_R2 3  // Right Wheel Sensor

#define ISR_INTERVAL 20 // interval of 20 milli seconds to update counter by interupt

// PID control system variables - from examples
float Kp = 0.05;  // Proportional gain - initial value from examples
float Ki = 0.00001;  // Integral gain - initial value from examples
float Kd = 0.8;  // Derivative gain - initial value from examples
int error = 0;
int lastError = 0;
float integral = 0;
float derivative = 0;
int correction;

void leftEncoderISR() {
  static unsigned long timer;
  if (millis() > timer) {
     _leftTicks++;
     timer = millis() + ISR_INTERVAL;
  }
}

void rightEncoderISR() {
  static unsigned long timer;
  if (millis() > timer) {
    _rightTicks++;
    timer = millis() + ISR_INTERVAL;
  }
}

void moveForwardPID(int _leftSpeed, int _rightSpeed, bool withOutLine, bool lineTracking) {
  if (withOutLine) {
    // For straight line movement using encoders
    // Calculate error (difference between left and right encoder counts)
    error = _leftTicks - _rightTicks;
    
    // Update PID terms
    integral += error;
    derivative = error - lastError;
    lastError = error;

    // Prevent integral windup
    integral = constrain(integral, -10, 10);
    
    // Calculate correction
    correction = (Kp * error) + (Ki * integral) + (Kd * derivative);

  } else if (lineTracking) {
    // For line following using sensors
    readSensors();
    
    // Calculate weighted position (similar to examples)
    long weightedSum = 0;
    long sum = 0;
    
    for (int i = 0; i < NUM_SENSORS; i++) {
      int value = sensorValues[i];
      weightedSum += (long)value * i * 1000;
      sum += value;
    }
    
    // Prevent division by zero
    if (sum == 0) sum = 1;
    
    // Calculate position (0-7000 range)
    int position = weightedSum / sum;
    
    // Calculate error relative to center (3500)
    error = position - 3500;  // 3500 is center (as in examples)
    
    // Update PID terms
    integral += error;
    derivative = error - lastError;
    lastError = error;
    
    // Prevent integral windup (more aggressive limiting)
    integral = constrain(integral, -100, 100);
    
    // Calculate correction with PID values from examples
    correction = (Kp * error) + (Ki * integral) + (Kd * derivative);
  }

  // Apply PID correction to motor speeds
  _leftSpeed -= correction;
  _rightSpeed += correction;

  // Ensure PWM values are within valid range (0 - 255)
  _leftSpeed = constrain(_leftSpeed, 0, 255);
  _rightSpeed = constrain(_rightSpeed, 0, 255);

  // Move motors
  moveForward(_leftSpeed, _rightSpeed);
}

// ... rest of the code ... 