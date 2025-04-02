#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

// NeoPixel setup
#define NEOPIXEL_PIN 5  // Use any available digital pin
#define NUM_PIXELS 4    // 4 NeoPixels
// Pixel mapping:
// Pixel 0: Bottom Left
// Pixel 1: Bottom Right
// Pixel 2: Top Right
// Pixel 3: Top Left
Adafruit_NeoPixel pixels(NUM_PIXELS, NEOPIXEL_PIN, NEO_RGB + NEO_KHZ800);

enum RobotState { FOLLOW_LINE, TURNING, TURNING_LEFT, TURNING_RIGHT, TURNING_AROUND, OBSTACLE_DETECTED, AVOIDING_OBSTACLE, CHECKING_FOR_PATH_AHEAD };
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
static const int DISTANCE_FROM_BASE_TO_CONE = 33; // Distance is in ticks
const int target = DISTANCE_FROM_BASE_TO_CONE;

// Encoder Pulse Counters
volatile signed int _leftTicks = 8;
volatile signed int _rightTicks = 0;

// Ultrasonic Sensor
#define TRIG_PIN 12
#define ECHO_PIN 13
#define MAX_DISTANCE 50 // Maximum distance in cm
#define OBSTACLE_THRESHOLD 17 // Distance in cm to consider an obstacle

// Motor Pins
#define MOTOR_A_1 11  // Right Backward
#define MOTOR_A_2 10  // Right Forward
#define MOTOR_B_1 9   // Left Backward
#define MOTOR_B_2 8   // Left Forward
int baseSpeed = 180;  // Adjusted to match your speed

//Servo Control
#define GRIPPER_OPEN 1750
#define GRIPPER_CLOSE 1220
#define SERVO 4
const int pulse = 2000;
int previousTime = 0;
const int gripperInterval = 20;

//Rotation Sensors
#define MOTOR_R1 2  // Left Wheel Sensor
#define MOTOR_R2 3  // Right Wheel Sensor

#define ISR_INTERVAL  20 // interval of 20 milli seconds to update counter by interupt

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

// Conditions
bool coneInSquare = true;
bool sensorsCalibrated = false;
bool conePickedUp = false;
bool gameStarted = false;
bool coneDroppedOff = false;
bool gameEnded = false;
bool motionComplete = true; 
static bool pathChecked = false;

int error = 0, lastError = 0;
float integral = 0;
float derivative = 0;

//PID
float Kp;// Proportional Gain
float Ki;  // Integral Gain
float Kd;  // Derivative Gain
int correction; 


int pulses;
int angle;
int raduis = DISTANCE_BETWEEN_WHEELS;
int turn_Circumference = 2 * 3.14 * raduis;
float turnDistances = 0; // ARC of a circle

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  
  // Initialize NeoPixels
  pixels.begin();
  pixels.setBrightness(50); // Set brightness (0-255)
  setColor(255, 0, 0); // Red at startup
  
  // Initialize Ultrasonic Sensor
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  
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

  //SERVO
  pinMode(SERVO, OUTPUT);
  digitalWrite(SERVO, LOW);
}

void loop() {
  // put your main code here, to run repeatedly:
  int unsigned currentTime = millis();
  if(conePickedUp) {
    if (currentTime - previousTime >= gripperInterval) {
      previousTime = currentTime;
      gripper(GRIPPER_CLOSE);
    }
  } else {
      if (currentTime - previousTime >= gripperInterval) {
        previousTime = currentTime;
        gripper(GRIPPER_OPEN);
      }
  }
  
  // Update lights early in the loop
  updateNeoPixels();
  
  if(coneInSquare && !sensorsCalibrated){
    calibrateSensors();
  }

  if (sensorsCalibrated && !conePickedUp) {
    conePickedUp = true;
    return;
  }

  if (sensorsCalibrated && !gameStarted && conePickedUp) {
    turnLeftMillis(90);
    if(robotState != FOLLOW_LINE) return;
    gameStarted = true;
  }

  if (gameStarted && !gameEnded) {
    getLinePosition();
    
    float dist = getDistance();
    
    if (dist != -1) /* That means an object has been detected*/ {  
      // Check if object is detected within threshold
      if (dist < OBSTACLE_THRESHOLD) {
        Serial.println("*** OBSTACLE DETECTED! ***");
        Serial.print("Obstacle at: ");
        Serial.print(dist);
        Serial.println(" cm - Turning to avoid");
        stopMotors();
        // Turn robot 180 degrees right to avoid obstacle
        turn180(140, 170);
        return; // Skip the rest of the loop
      }
    }
    
    

    if (linePosition != CENTER_LINE) {
      // checkPathAhead();
      // if(!pathChecked) return;
      if (linePosition == T_JUNCTION) {
        turnLeftMillis(90);
      } else if (linePosition == LEFT_LINE) {
        turnLeftMillis(80);
        
        // Continue turning until line is detected
        readSensors();
        bool lineDetected = false;
        for (int i = 0; i < NUM_SENSORS; i++) {
          if (sensorValues[i] > sensorThreshold[i]) {
            lineDetected = true;
            break;
          }
        }
        
        if (!lineDetected) {
          updateNeoPixels(); // Update lights before returning
          return; // Keep turning until line is detected
        }
      } else if (linePosition == NO_LINE) {
        turnAroundMillis();
      } else if (linePosition == RIGHT_LINE) {
        linePosition = CENTER_LINE;
        robotState = FOLLOW_LINE;
        //turnRightMillis(90);
        moveForwardPID(baseSpeed, baseSpeed, false, true);
      }
    }

    if (linePosition == CENTER_LINE) {
      moveForwardPID(baseSpeed, baseSpeed, false, true);
    }
  }

  // Update NeoPixel colors based on robot state - keep this one 
  // as a final update in case state changed during this loop
  updateNeoPixels();
}

void checkPathAhead() {
  static bool checkingPath = false;
  static int forwardTicks = 10; // Move forward this many ticks before checking
  static LinePosition storedPosition = CENTER_LINE; // Store the turn direction
  pathChecked = false;

  if (!checkingPath) {  
    // Store the intended LinePosition before checking
    storedPosition = linePosition;
    resetTicks();

    // Move forward slightly
    moveForwardPID(200, 200, true, false);
    checkingPath = true;
  }

  // Wait until the robot moves forward by forwardTicks
  if (checkingPath && (_leftTicks >= forwardTicks || _rightTicks >= forwardTicks)) {
    stopMotors();
    checkingPath = false;
    // Check if a path exists ahead
    readSensors();
    bool pathExists = sensorValues[2] > sensorThreshold[2] || sensorValues[3] > sensorThreshold[3] || sensorValues[4] > sensorThreshold[4] || sensorValues[5] > sensorThreshold[5];

    if (pathExists) {
      robotState = FOLLOW_LINE;
        // If a path is found, follow it
      linePosition = CENTER_LINE;
      motionComplete = true;
      pathChecked = true;
    } else {
      linePosition = storedPosition;  // Otherwise, proceed with the original turn
      motionComplete = false;
      pathChecked = true;
    }
    
    // Update LEDs after state change
    updateNeoPixels();
  }
}

void getLinePosition() {
  readSensors();
  leftTurn = sensorValues[4] > sensorThreshold[4] && sensorValues[5] > sensorThreshold[5] && sensorValues[6] > sensorThreshold[6] && sensorValues[7] > sensorThreshold[7] && sensorValues[0] < sensorThreshold[0] && sensorValues[1] < sensorThreshold[1] && sensorValues[2] < sensorThreshold[2];
  rightTurn = sensorValues[5] < sensorThreshold[5] && sensorValues[6] < sensorThreshold[6] && sensorValues[7] < sensorThreshold[7] && sensorValues[0] > sensorThreshold[0] && sensorValues[1] > sensorThreshold[1] && sensorValues[2] > sensorThreshold[2];
  tJunctionOrBase = sensorValues[0] > sensorThreshold[0] && sensorValues[1] > sensorThreshold[1] && sensorValues[2] > sensorThreshold[2] && sensorValues[3] > sensorThreshold[3] && sensorValues[4] > sensorThreshold[4] && sensorValues[5] > sensorThreshold[5] && sensorValues[6] > sensorThreshold[6] && sensorValues[7] > sensorThreshold[7];
  deadEnd = sensorValues[0] < sensorThreshold[0] && sensorValues[1] < sensorThreshold[1] && sensorValues[2] < sensorThreshold[2] && sensorValues[3] < sensorThreshold[3] && sensorValues[4] < sensorThreshold[4] && sensorValues[5] < sensorThreshold[5] && sensorValues[6] < sensorThreshold[6] && sensorValues[7] < sensorThreshold[7];
  
  if(motionComplete) { // State Locking 
    if (leftTurn) {
      linePosition = LEFT_LINE;
      //stopMotors();
    } else if (rightTurn) {
      linePosition = RIGHT_LINE;
      //stopMotors();
    } else if (deadEnd) {
      linePosition = NO_LINE;
      //stopMotors();
    } else if (tJunctionOrBase){
      linePosition = T_JUNCTION;
      //stopMotors();
    } else {
      linePosition = CENTER_LINE;
    }
  }
}

void turnLeftMillis(int angle) {
  static unsigned long lastCheck = 0;
  const unsigned long checkInterval = 5;
  static int targetPulses;

  if (robotState != TURNING_LEFT) {
    resetTicks();
    targetPulses = 0;
    float turnDistance = (angle / 360.0) * turn_Circumference;  
    targetPulses = (turnDistance / WHEEL_CIRCUMFERENCE) * PULSE_PER_REVOLUTION;

    analogWrite(MOTOR_A_1, 0);
    analogWrite(MOTOR_A_2, 0);
    analogWrite(MOTOR_B_1, 0);
    analogWrite(MOTOR_B_2, 0);
    
    // Right motor forward, left motor stopped for left turn
    analogWrite(MOTOR_A_2, 140);  // Right Forward
    analogWrite(MOTOR_B_2, 0);    // Left Forward (stopped)
    
    robotState = TURNING_LEFT;  //Lock state to "TURNING_LEFT"
    motionComplete = false;
    
    // Update LEDs immediately after state change
    updateNeoPixels();
  }

  if (robotState == TURNING_LEFT) {
    lastCheck = millis();
    if (_rightTicks >= targetPulses) {
      stopMotors();
      robotState = FOLLOW_LINE;  // Unlock state after turn is complete
      motionComplete = true;
      linePosition = CENTER_LINE;
      
      // Update LEDs immediately after state change
      updateNeoPixels();
    }
  }
}

void turnRightMillis(int angle) {
  static unsigned long lastCheck = 0;
  const unsigned long checkInterval = 5;
  static int targetPulses;

  if (robotState != TURNING_RIGHT) {
    resetTicks();  // Reset left encoder ticks
    targetPulses = 0;
    float turnDistance = (angle / 360.0) * turn_Circumference;  
    targetPulses = (turnDistance / WHEEL_CIRCUMFERENCE) * PULSE_PER_REVOLUTION;

    analogWrite(MOTOR_A_1, 0);
    analogWrite(MOTOR_A_2, 0);
    analogWrite(MOTOR_B_1, 0);
    analogWrite(MOTOR_B_2, 0);
    
    // Left motor forward, right motor stopped for right turn
    analogWrite(MOTOR_A_2, 0);     // Right Forward (stopped)
    analogWrite(MOTOR_B_2, 90);   // Left Forward
    
    robotState = TURNING_RIGHT;  // Lock state to "TURNING_RIGHT"
    motionComplete = false;
  }

  if (robotState == TURNING_RIGHT) {
    lastCheck = millis();
    if (_leftTicks >= targetPulses) {
      stopMotors();
      robotState = FOLLOW_LINE;  // Unlock state after turn is complete
      motionComplete = true;
      linePosition = CENTER_LINE;
    }
  }
}

void turnAroundMillis() {
  static unsigned long lastCheck = 0;
  const unsigned long checkInterval = 5;
  static int targetPulses;
  if (robotState != TURNING_AROUND) {
    resetTicks();
    targetPulses = 0;

    float turnDistance = (3.14 * (DISTANCE_BETWEEN_WHEELS / 2)); // Half the turning circumference
    targetPulses = (turnDistance / WHEEL_CIRCUMFERENCE) * PULSE_PER_REVOLUTION;

    turn180(140, 140); // Left wheel moves backward, right moves forward

    robotState = TURNING_AROUND; // Lock state to "TURNING_AROUND"
    motionComplete = false;
    
    // Update LEDs immediately after state change
    updateNeoPixels();
  }

  if (robotState == TURNING_AROUND) {
    sensorValues[0] = analogRead(sensorPins[0]);
    if (sensorValues[0] > sensorThreshold[0] || sensorValues[4] > sensorThreshold[4]) {
      stopMotors();
      robotState = FOLLOW_LINE;  // Unlock state after turn is complete
      motionComplete = true;
      linePosition = CENTER_LINE;
      
      // Update LEDs immediately after state change
      updateNeoPixels();
    }
  }
}

void calibrateSensors() {
  static bool firstRun = true;
  readSensors();

  // Initialize sensor min/max values on the first run
  if (firstRun) {
    for (int i = 0; i < NUM_SENSORS; i++) {
      sensorMin[i] = 1023;  // Set min to max ADC value
      sensorMax[i] = 0;     // Set max to min ADC value
    }
    firstRun = false;
  }

  // Check if target distance is reached
  if (_leftTicks > target || _rightTicks > target) {
    stopMotors();
    sensorsCalibrated = true;

    // Calculate and store threshold values
    for (int i = 0; i < NUM_SENSORS; i++) {
      sensorThreshold[i] = (sensorMin[i] + sensorMax[i]) / 2;
      Serial.print("Threshold [");
      Serial.print(i);
      Serial.print("]: ");
      Serial.println(sensorThreshold[i]);
      Serial.println(sensorMin[i]);
      Serial.println(sensorMax[i]);
    }

    return;
  }

  // Read sensor values and update min/max
  for (int i = 0; i < NUM_SENSORS; i++) {
    int sensorValue = analogRead(sensorPins[i]);

    if (sensorValue < sensorMin[i]) {
        sensorMin[i] = sensorValue;
    }
    if (sensorValue > sensorMax[i]) {
        sensorMax[i] = sensorValue;
    }
  }

  moveForwardPID(200, 200, true, false);
}

void moveForwardPID(int _leftSpeed, int _rightSpeed, bool withOutLine, bool lineTracking) {
  if (withOutLine) {
    // PID Variables
    Kp = 6.5;  // Proportional Gain
    Ki = 0.1;  // Integral Gain
    Kd = 5;  // Derivative Gain

    // Calculate error
    error = _leftTicks - _rightTicks;

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

  // Move motors
  moveForward(_leftSpeed, _rightSpeed);
}


void stopMotors() {
  analogWrite(MOTOR_A_1, 0);
  analogWrite(MOTOR_A_2, 0);
  analogWrite(MOTOR_B_1, 0);
  analogWrite(MOTOR_B_2, 0);
  
  // Update LEDs immediately when stopping
  updateNeoPixels();
}

void moveForward(int _leftSpeed, int _rightSpeed) {
  analogWrite(MOTOR_A_2, _rightSpeed);  // Right Forward
  analogWrite(MOTOR_B_2, _leftSpeed);   // Left Forward
}

void turn180(int _leftSpeed, int _rightSpeed) {
    analogWrite(MOTOR_A_1, _rightSpeed);  // Right Backward
    analogWrite(MOTOR_A_2, 0);
    analogWrite(MOTOR_B_1, 0);
    analogWrite(MOTOR_B_2, _leftSpeed);   // Left Forward
}

void gripper(int pulse) {
  static unsigned long timer;
  static int lastPulse;
  if (millis() > timer) {
    if (pulse >0) {
      lastPulse = pulse;
    } else {
      pulse = lastPulse;
    }

    digitalWrite(SERVO, HIGH);
    delayMicroseconds(pulse);
    digitalWrite(SERVO, LOW);
    timer = millis() + 20;
  }
}

int calculateLinePosition() {
  long weightedSum = 0;
  long sum = 0;

  for (int i = 0; i < NUM_SENSORS; i++) {
    int value = sensorValues[i];

    // Determine if the sensor is on the line
    //int reading = (value > sensorThreshold[i]) ? 1 : 0;
    weightedSum += (long)value * i * 1000;
    sum += value;
  }

  return weightedSum / sum;  // No line detected
}

void readSensors() {
  for (int i = 0; i < NUM_SENSORS; i++) {
      sensorValues[i] = analogRead(sensorPins[i]);
  }
}

void resetTicks() {
  _leftTicks = 0;
  _rightTicks = 0;
}

// Function to update NeoPixel colors based on robot state
void updateNeoPixels() {
  if (robotState == FOLLOW_LINE) {
    // Forward: All lights (0-3) GREEN
    setColor(0, 255, 0);
  } else if (robotState == TURNING_LEFT) {
    // TurnLeftMillis: lights 0 and 3 (left side) ORANGE, lights 1 and 2 (right side) GREEN
    pixels.setPixelColor(0, pixels.Color(255, 165, 0)); // Bottom Left - ORANGE
    pixels.setPixelColor(3, pixels.Color(255, 165, 0)); // Top Left - ORANGE
    pixels.setPixelColor(1, pixels.Color(0, 255, 0));   // Bottom Right - GREEN
    pixels.setPixelColor(2, pixels.Color(0, 255, 0));   // Top Right - GREEN
    pixels.show();
  } else if (robotState == TURNING_AROUND) {
    // TurnAroundMillis: lights 1 and 2 (right side) ORANGE, lights 0 and 3 (left side) GREEN
    pixels.setPixelColor(1, pixels.Color(255, 165, 0)); // Bottom Right - ORANGE
    pixels.setPixelColor(2, pixels.Color(255, 165, 0)); // Top Right - ORANGE
    pixels.setPixelColor(0, pixels.Color(0, 255, 0));   // Bottom Left - GREEN
    pixels.setPixelColor(3, pixels.Color(0, 255, 0));   // Top Left - GREEN
    pixels.show();
  } else {
    // Stopped: All lights (0-3) RED
    setColor(255, 0, 0);
  }
}

// Helper function to set all NeoPixels to the same color
void setColor(uint8_t r, uint8_t g, uint8_t b) {
  for(int i=0; i<NUM_PIXELS; i++) {
    pixels.setPixelColor(i, pixels.Color(r, g, b));
  }
  pixels.show();
}

// Helper function to set different colors for left and right sides
void setSplitColors(uint8_t leftR, uint8_t leftG, uint8_t leftB, 
                    uint8_t rightR, uint8_t rightG, uint8_t rightB) {
  // Assuming first half of pixels are on left side, second half on right side
  int middle = NUM_PIXELS / 2;
  
  // Set left side pixels (1-2)
  for(int i=0; i<middle; i++) {
    pixels.setPixelColor(i, pixels.Color(leftR, leftG, leftB));
  }
  
  // Set right side pixels (3-4)
  for(int i=middle; i<NUM_PIXELS; i++) {
    pixels.setPixelColor(i, pixels.Color(rightR, rightG, rightB));
  }
  
  pixels.show();
}

// Function to get distance from ultrasonic sensor
float getDistance() {
  // Clear the trigger pin
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  
  // Set the trigger pin HIGH for 10 microseconds
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  // Read the echo pin, return the sound wave travel time in microseconds
  long duration = pulseIn(ECHO_PIN, HIGH, 30000); // Timeout after 30ms
  
  // Calculate the distance
  float distance = duration * 0.034 / 2; // Speed of sound is 340 m/s or 0.034 cm/μs
  
  // Print the distance
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");
  
  if (distance > 0 && distance < MAX_DISTANCE) {
    return distance;
  }
  else {
    return -1;
  }
}