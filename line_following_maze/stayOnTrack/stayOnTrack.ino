#include <Arduino.h>

//Line Sensors
#define NUM_SENSORS 8  // Number of sensors
int sensorPins[NUM_SENSORS] = {A0, A1, A2, A3, A4, A5, A6, A7};  // Sensor pin mapping
int sensorValues[NUM_SENSORS];  // Array to store sensor readings
int sensorMin[NUM_SENSORS];
int sensorMax[NUM_SENSORS];
int sensorThreshold[NUM_SENSORS]; // Array to store thresholds for each sensor

//Measurements
const float WHEEL_CIRCUMFERENCE = 20.4;
const int PULSE_PER_REVOLUTION = 20;
const float DISTANCE_BETWEEN_WHEELS = 22.75;

// Encoder Pulse Counters
volatile signed int leftTicks = 0;
volatile signed int rightTicks = 0;

// Motor Pins
#define MOTOR_A_1 11  // Right Backward
#define MOTOR_A_2 10  // Right Forward
#define MOTOR_B_1 9   // Left Backward
#define MOTOR_B_2 8   // Left Forward
int baseSpeed = 170;  // Base speed for straight line

//Rotation Sensors
#define MOTOR_R1 2  // Left Wheel Sensor
#define MOTOR_R2 3  // Right Wheel Sensor

// PID variables
float Kp = 0.4;    // Proportional Gain - adjust this value to change how aggressively robot corrects
float Ki = 0.0001; // Integral Gain - helps with consistent errors
float Kd = 0.15;   // Derivative Gain - helps prevent oscillation
int error = 0, lastError = 0;
float integral = 0;
float derivative = 0;
int correction = 0;

bool sensorsCalibrated = false;

void leftEncoderISR() {
  static unsigned long timer;
  if (millis() > timer) {
     leftTicks++;
     timer = millis() + 20; // 20ms debounce
  }
}

void rightEncoderISR() {
  static unsigned long timer;
  if (millis() > timer) {
    rightTicks++;
    timer = millis() + 20; // 20ms debounce
  }
}

void setup() {
  // Initialize serial communication for debugging
  Serial.begin(9600);
  
  // Set Motor Pins
  pinMode(MOTOR_A_1, OUTPUT);
  pinMode(MOTOR_A_2, OUTPUT);
  pinMode(MOTOR_B_1, OUTPUT);
  pinMode(MOTOR_B_2, OUTPUT);
  digitalWrite(MOTOR_A_1, LOW);
  digitalWrite(MOTOR_A_2, LOW);
  digitalWrite(MOTOR_B_1, LOW);
  digitalWrite(MOTOR_B_2, LOW); 

  // Attach Interrupts for Encoders
  attachInterrupt(digitalPinToInterrupt(MOTOR_R1), leftEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MOTOR_R2), rightEncoderISR, CHANGE);
  
  // Start with calibration
  calibrateSensors();
}

void loop() {
  if (sensorsCalibrated) {
    // Read sensor values
    readSensors();
    
    // Print sensor values for debugging
    printSensorValues();
    
    // Follow the line using PID control
    followLine();
  }
}

void followLine() {
  // Calculate line position (weighted average)
  int position = calculateLinePosition();
  
  // Calculate error from center position
  int center = (NUM_SENSORS - 1) * 1000 / 2;  // Center of the sensor array
  error = position - center;
  
  // Update PID terms
  integral += error;
  derivative = error - lastError;
  lastError = error;
  
  // Limit integral to prevent windup
  integral = constrain(integral, -10000, 10000);
  
  // Calculate correction value
  correction = (Kp * error) + (Ki * integral) + (Kd * derivative);
  
  // Apply correction to motor speeds
  int leftSpeed = baseSpeed - correction;
  int rightSpeed = baseSpeed + correction;
  
  // Constrain speed values to valid range
  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);
  
  // Drive motors with PID-adjusted speeds
  moveForward(leftSpeed, rightSpeed);
  
  // Debug output
  Serial.print("Position: "); Serial.print(position);
  Serial.print(" Error: "); Serial.print(error);
  Serial.print(" Correction: "); Serial.print(correction);
  Serial.print(" Left: "); Serial.print(leftSpeed);
  Serial.print(" Right: "); Serial.println(rightSpeed);
}

int calculateLinePosition() {
  long weightedSum = 0;
  long sum = 0;
  
  for (int i = 0; i < NUM_SENSORS; i++) {
    int value = sensorValues[i];
    weightedSum += (long)value * i * 1000;
    sum += value;
  }
  
  // Handle case when no line is detected
  if (sum == 0) {
    return 0;
  }
  
  return weightedSum / sum;
}

void readSensors() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    sensorValues[i] = analogRead(sensorPins[i]);
  }
}

void printSensorValues() {
  static unsigned long lastPrintTime = 0;
  
  // Only print every 500ms to avoid flooding serial
  if (millis() - lastPrintTime > 500) {
    lastPrintTime = millis();
    
    Serial.print("Sensors: ");
    for (int i = 0; i < NUM_SENSORS; i++) {
      Serial.print(sensorValues[i]);
      Serial.print(" ");
    }
    Serial.println();
  }
}

void calibrateSensors() {
  Serial.println("Starting sensor calibration...");
  Serial.println("Move robot over line back and forth");
  
  // Initialize min and max values
  for (int i = 0; i < NUM_SENSORS; i++) {
    sensorMin[i] = 1023;  // Set min to max possible
    sensorMax[i] = 0;     // Set max to min possible
  }
  
  // Calibrate for 5 seconds
  unsigned long startTime = millis();
  
  // Turn in place to find line during calibration
  analogWrite(MOTOR_A_2, 150);  // Right Forward
  analogWrite(MOTOR_B_1, 150);  // Left Backward
  
  while (millis() - startTime < 5000) {
    // Read all sensors
    for (int i = 0; i < NUM_SENSORS; i++) {
      int sensorValue = analogRead(sensorPins[i]);
      
      // Update min/max values
      if (sensorValue < sensorMin[i]) {
        sensorMin[i] = sensorValue;
      }
      if (sensorValue > sensorMax[i]) {
        sensorMax[i] = sensorValue;
      }
    }
  }
  
  // Stop motors
  stopMotors();
  
  // Calculate threshold values (midpoint between min and max)
  for (int i = 0; i < NUM_SENSORS; i++) {
    sensorThreshold[i] = (sensorMin[i] + sensorMax[i]) / 2;
    
    Serial.print("Sensor ");
    Serial.print(i);
    Serial.print(" - Min: ");
    Serial.print(sensorMin[i]);
    Serial.print(" Max: ");
    Serial.print(sensorMax[i]);
    Serial.print(" Threshold: ");
    Serial.println(sensorThreshold[i]);
  }
  
  Serial.println("Calibration complete!");
  sensorsCalibrated = true;
  
  // Short pause before starting
  delay(1000);
}

void moveForward(int leftSpeed, int rightSpeed) {
  // Stop any backward motion
  analogWrite(MOTOR_A_1, 0);  // Right Backward
  analogWrite(MOTOR_B_1, 0);  // Left Backward
  
  // Set forward motion with specified speeds
  analogWrite(MOTOR_A_2, rightSpeed);  // Right Forward
  analogWrite(MOTOR_B_2, leftSpeed);   // Left Forward
}

void stopMotors() {
  analogWrite(MOTOR_A_1, 0);
  analogWrite(MOTOR_A_2, 0);
  analogWrite(MOTOR_B_1, 0);
  analogWrite(MOTOR_B_2, 0);
}
