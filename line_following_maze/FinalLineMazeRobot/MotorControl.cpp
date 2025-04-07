#include "MotorControl.h"
#include "LightSensors.h"
#include "NeoPixels.h"

// Initialize motor control pins
void setupMotors() {
  pinMode(MOTOR_A_1, OUTPUT);     // Right backward
  pinMode(MOTOR_A_2, OUTPUT);     // Right forward
  pinMode(MOTOR_B_1, OUTPUT);     // Left backward
  pinMode(MOTOR_B_2, OUTPUT);     // Left forward
  digitalWrite(MOTOR_A_1, LOW);   // Start with motors off
  digitalWrite(MOTOR_A_2, LOW);
  digitalWrite(MOTOR_B_1, LOW);
  digitalWrite(MOTOR_B_2, LOW);
}

// Interrupt handler for left wheel encoder
void leftEncoderISR() {
  static unsigned long timer;
  if (millis() > timer) {         // Debounce encoder signals
     _leftTicks++;                // Increment left wheel counter
     timer = millis() + ISR_INTERVAL;  // Set next valid time
  }
}

// Interrupt handler for right wheel encoder
void rightEncoderISR() {
  static unsigned long timer;
  if (millis() > timer) {         // Debounce encoder signals
    _rightTicks++;                // Increment right wheel counter
    timer = millis() + ISR_INTERVAL;  // Set next valid time
  }
}

// Basic forward movement with different speeds for each wheel
void moveForward(int _leftSpeed, int _rightSpeed) {
  analogWrite(MOTOR_A_2, _rightSpeed);  // Right wheel forward
  analogWrite(MOTOR_B_2, _leftSpeed);   // Left wheel forward
}

// Stop all motors
void stopMotors() {
  analogWrite(MOTOR_A_1, 0);      // Turn off all motor pins
  analogWrite(MOTOR_A_2, 0);
  analogWrite(MOTOR_B_1, 0);
  analogWrite(MOTOR_B_2, 0);
  updateNeoPixels();              // Update lights to show we stopped
}

// Turn in place - right backward, left forward
void turn180(int _leftSpeed, int _rightSpeed) {
  analogWrite(MOTOR_A_1, _rightSpeed);  // Right wheel backward
  analogWrite(MOTOR_A_2, 0);
  analogWrite(MOTOR_B_1, 0);
  analogWrite(MOTOR_B_2, _leftSpeed);   // Left wheel forward
}

// Reset both wheel encoder counters
void resetTicks() {
  _leftTicks = 0;
  _rightTicks = 0;
}

// Forward movement with PID control for straight lines or line following
void moveForwardPID(int _leftSpeed, int _rightSpeed, bool withOutLine, bool lineTracking) {
  if (withOutLine) {
    // PID for straight movement using wheel encoders
    Kp = 6.5;                     // Proportional gain (higher = stronger response)
    Ki = 0.1;                     // Integral gain (accumulating error)
    Kd = 5;                       // Derivative gain (change in error)

    error = _leftTicks - _rightTicks;  // Difference between wheel movements
    integral += error;            // Accumulate error over time
    derivative = error - lastError;  // Rate of change in error
    lastError = error;            // Save for next calculation
    integral = constrain(integral, -10, 10);  // Prevent windup

  } else if (lineTracking) {
    // PID for line following using sensors
    readSensors();                // Get fresh readings
    int position = calculateLinePosition();  // Where is the line?

    Kp = 0.4;                     // Lower values for smoother line following
    Ki = 0.00001;                 // Very small integral term
    Kd = 0.15;                    // Dampen oscillations

    int center = (NUM_SENSORS - 1) * 1000 / 2;  // Middle position (3500)
    error = position - center;    // How far off center are we?
    integral += error;            // Accumulate error
    derivative = error - lastError;  // Rate of change
    lastError = error;            // Save for next calculation
  }

  // Calculate final correction value
  correction = (Kp * error) + (Ki * integral) + (Kd * derivative);
  
  // Apply correction by slowing one wheel and speeding up the other
  _leftSpeed -= correction;       // If positive error, slow left wheel
  _rightSpeed += correction;      // If positive error, speed up right wheel

  // Keep speeds within valid PWM range
  _leftSpeed = constrain(_leftSpeed, 0, 255);
  _rightSpeed = constrain(_rightSpeed, 0, 255);

  // Apply corrected speeds to motors
  moveForward(_leftSpeed, _rightSpeed);
}

// Turn left by specific angle with encoder tracking
void turnLeftMillis(int angle) {
  static unsigned long lastCheck = 0;
  const unsigned long checkInterval = 5;  // Check progress every 5ms
  static int targetPulses;                // Target encoder count
  
  // Start turn sequence if not already turning left
  if (robotState != TURNING_LEFT) {
    resetTicks();                         // Start fresh count
    targetPulses = 0;
    
    // Calculate how many ticks for the requested angle
    float turnDistance = (angle / 360.0) * turn_Circumference;  // Arc length
    targetPulses = (turnDistance / WHEEL_CIRCUMFERENCE) * PULSE_PER_REVOLUTION;  // Convert to ticks
    
    // Stop all motors first
    analogWrite(MOTOR_A_1, 0);
    analogWrite(MOTOR_A_2, 0);
    analogWrite(MOTOR_B_1, 0);
    analogWrite(MOTOR_B_2, 0);
    
    // Start turning - right wheel forward, left stopped
    analogWrite(MOTOR_A_2, 140);          // Right wheel forward
    analogWrite(MOTOR_B_2, 0);            // Left wheel stopped
    
    robotState = TURNING_LEFT;            // Update state
    motionComplete = false;               // Mark motion in progress
    updateNeoPixels();                    // Update lights to show turning
  }
  
  // Check if turn is complete
  if (robotState == TURNING_LEFT) {
    lastCheck = millis();
    if (_rightTicks >= targetPulses) {    // Right wheel reached target
      stopMotors();                       // Stop turning
      robotState = FOLLOW_LINE;           // Back to line following
      motionComplete = true;              // Motion complete
      linePosition = CENTER_LINE;         // Assume we're centered
      updateNeoPixels();                  // Update lights
    }
  }
}

// Turn right by specific angle with encoder tracking
void turnRightMillis(int angle) {
  static unsigned long lastCheck = 0;
  const unsigned long checkInterval = 5;  // Check progress every 5ms
  static int targetPulses;                // Target encoder count
  
  // Start turn sequence if not already turning right
  if (robotState != TURNING_RIGHT) {
    resetTicks();                         // Start fresh count
    targetPulses = 0;
    
    // Calculate how many ticks for the requested angle
    float turnDistance = (angle / 360.0) * turn_Circumference;  // Arc length
    targetPulses = (turnDistance / WHEEL_CIRCUMFERENCE) * PULSE_PER_REVOLUTION;  // Convert to ticks
    
    // Stop all motors first
    analogWrite(MOTOR_A_1, 0);
    analogWrite(MOTOR_A_2, 0);
    analogWrite(MOTOR_B_1, 0);
    analogWrite(MOTOR_B_2, 0);
    
    // Start turning - left wheel forward, right stopped
    analogWrite(MOTOR_A_2, 0);            // Right wheel stopped
    analogWrite(MOTOR_B_2, 90);           // Left wheel forward (slower)
    
    robotState = TURNING_RIGHT;           // Update state
    motionComplete = false;               // Mark motion in progress
  }
  
  // Check if turn is complete
  if (robotState == TURNING_RIGHT) {
    lastCheck = millis();
    if (_leftTicks >= targetPulses) {     // Left wheel reached target
      stopMotors();                       // Stop turning
      robotState = FOLLOW_LINE;           // Back to line following
      motionComplete = true;              // Motion complete
      linePosition = CENTER_LINE;         // Assume we're centered
    }
  }
}

// Turn 180 degrees to reverse direction
void turnAroundMillis() {
  static unsigned long lastCheck = 0;
  const unsigned long checkInterval = 5;  // Check progress every 5ms
  static int targetPulses;                // Target encoder count
  
  // Start turn sequence if not already turning around
  if (robotState != TURNING_AROUND) {
    resetTicks();                         // Start fresh count
    targetPulses = 0;
    
    // Calculate ticks to turn halfway around (π × radius)
    float turnDistance = (3.14 * (DISTANCE_BETWEEN_WHEELS / 2));  // Half-circle distance
    targetPulses = (turnDistance / WHEEL_CIRCUMFERENCE) * PULSE_PER_REVOLUTION;  // Convert to ticks
    
    turn180(140, 140);                    // Start turning with both wheels
    robotState = TURNING_AROUND;          // Update state
    motionComplete = false;               // Mark motion in progress
    updateNeoPixels();                    // Update lights
  }
  
  // Keep checking for line during turn (don't wait for exact pulse count)
  if (robotState == TURNING_AROUND) {
    sensorValues[0] = analogRead(sensorPins[0]);  // Check outer sensors
    if (sensorValues[0] > sensorThreshold[0] || sensorValues[4] > sensorThreshold[4]) {
      stopMotors();                       // Stop when line detected
      robotState = FOLLOW_LINE;           // Back to line following
      motionComplete = true;              // Motion complete
      linePosition = CENTER_LINE;         // Assume we're centered
      updateNeoPixels();                  // Update lights
    }
  }
} 