#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

// Pin Definitions
#define NEOPIXEL_PIN 5
#define NUM_PIXELS 4
#define NUM_SENSORS 8
#define TRIG_PIN 12
#define ECHO_PIN 13
#define MOTOR_A_1 11
#define MOTOR_A_2 10
#define MOTOR_B_1 9
#define MOTOR_B_2 8
#define SERVO 4
#define MOTOR_R1 2
#define MOTOR_R2 3

// Constants
const float WHEEL_CIRCUMFERENCE = 20.4;
const int PULSE_PER_REVOLUTION = 20;
const float DISTANCE_BETWEEN_WHEELS = 22.75;
const int DISTANCE_FROM_BASE_TO_CONE = 33;
const int target = DISTANCE_FROM_BASE_TO_CONE;
const int GRIPPER_OPEN = 1750;
const int GRIPPER_CLOSE = 1220;
const int pulse = 2000;
const int gripperInterval = 20;
const int ISR_INTERVAL = 20;
const unsigned long checkInterval = 100;
const unsigned long flashInterval = 100;
const int MAX_DISTANCE = 50;
const int OBSTACLE_THRESHOLD = 17;
const int MAX_DISTANCE_TO_CHECK = 30;
const int MIN_DISTANCE_TO_CHECK = 5;
const int NUM_READINGS = 3;

// Enums
enum RobotState { FOLLOW_LINE, TURNING, TURNING_LEFT, TURNING_RIGHT, TURNING_AROUND, OBSTACLE_DETECTED, AVOIDING_OBSTACLE, CHECKING_FOR_PATH_AHEAD };
enum LinePosition { T_JUNCTION, LEFT_LINE, RIGHT_LINE, NO_LINE, CENTER_LINE };

// Global Variables
Adafruit_NeoPixel pixels(NUM_PIXELS, NEOPIXEL_PIN, NEO_RGB + NEO_KHZ800);
RobotState robotState = FOLLOW_LINE;
LinePosition linePosition = CENTER_LINE;
int sensorPins[NUM_SENSORS] = {A0, A1, A2, A3, A4, A5, A6, A7};
int sensorValues[NUM_SENSORS];
int sensorMin[NUM_SENSORS];
int sensorMax[NUM_SENSORS];
int sensorThreshold[NUM_SENSORS];
bool leftTurn, rightTurn, tJunctionOrBase, deadEnd;
volatile signed int _leftTicks = 8;
volatile signed int _rightTicks = 0;
int baseSpeed = 180;
int previousTime = 0;
bool otherRobotDetected = false;
float distanceReadings[NUM_READINGS];
int readingCount = 0;
int readingIndex = 0;
unsigned long lastCheckTime = 0;
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
float Kp, Ki, Kd;
int correction;
int pulses;
int angle;
int raduis = DISTANCE_BETWEEN_WHEELS;
int turn_Circumference = 2 * 3.14 * raduis;
float turnDistances = 0;
unsigned long lastFlashTime = 0;
bool flashState = false;

// Line Sensors
int sensorMin[NUM_SENSORS];
int sensorMax[NUM_SENSORS];
int sensorThreshold[NUM_SENSORS]; // Array to store thresholds for each sensor

// Line Positions
bool leftTurn; 
bool rightTurn;
bool tJunctionOrBase;
bool deadEnd;

// Measurements
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
#define MAX_DISTANCE_TO_CHECK 30 // Maximum distance in cm to check for other robot coming to drop cone off
#define MIN_DISTANCE_TO_CHECK 5 // Minimum distance in cm to check for other robot coming to drop cone off
#define NUM_READINGS 3  // Number of readings to average

// Motor Pins
#define MOTOR_A_1 11  // Right Backward
#define MOTOR_A_2 10  // Right Forward
#define MOTOR_B_1 9   // Left Backward
#define MOTOR_B_2 8   // Left Forward
int baseSpeed = 180;  // Adjusted to match your speed

// Servo Control
#define GRIPPER_OPEN 1750
#define GRIPPER_CLOSE 1220
#define SERVO 4
const int pulse = 2000;
int previousTime = 0;
const int gripperInterval = 20;

// Rotation Sensors
#define MOTOR_R1 2  // Left Wheel Sensor
#define MOTOR_R2 3  // Right Wheel Sensor

// Interrupt Service Routines
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

// Setup and Main Loop
void setup() {
  Serial.begin(9600);
  pixels.begin();
  pixels.setBrightness(50);
  setColor(255, 0, 0);
  
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  
  pinMode(MOTOR_A_1, OUTPUT);
  pinMode(MOTOR_A_2, OUTPUT);
  pinMode(MOTOR_B_1, OUTPUT);
  pinMode(MOTOR_B_2, OUTPUT);
  digitalWrite(MOTOR_A_1, LOW);
  digitalWrite(MOTOR_A_2, LOW);
  digitalWrite(MOTOR_B_1, LOW);
  digitalWrite(MOTOR_B_2, LOW);

  attachInterrupt(digitalPinToInterrupt(MOTOR_R1), leftEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MOTOR_R2), rightEncoderISR, CHANGE);

  pinMode(SERVO, OUTPUT);
  digitalWrite(SERVO, LOW);
}

void loop() {
  while(!otherRobotDetected){
    unsigned long currentTime = millis();
    if (currentTime - lastCheckTime >= checkInterval) {
      lastCheckTime = currentTime;
      
      int distance = getDistance();
      if (distance < MAX_DISTANCE_TO_CHECK && distance > MIN_DISTANCE_TO_CHECK){        
        distanceReadings[readingIndex] = distance;
        readingIndex = (readingIndex + 1) % NUM_READINGS;
        
        if (readingCount < NUM_READINGS) {
          readingCount++;
        }
        
        if (readingCount >= NUM_READINGS) {
          Serial.println("*** OTHER ROBOT CONFIRMED! ***");
          delay(3000);
          Serial.println("Starting Race in 3 seconds...");
          otherRobotDetected = true;
        }
      } else {
        readingCount = 0;
      }
    }
  }

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
    
    if (dist != -1) {  
      if (dist < OBSTACLE_THRESHOLD) {
        Serial.println("*** OBSTACLE DETECTED! ***");
        Serial.print("Obstacle at: ");
        Serial.print(dist);
        Serial.println(" cm - Turning to avoid");
        stopMotors();
        turn180(140, 170);
        return;
      }
    }
    
    switch (linePosition) {
      case T_JUNCTION:
        turnLeftMillis(90);
        break;
        
      case LEFT_LINE:
        turnLeftMillis(90);
        readSensors();
        {
          bool lineDetected = false;
          for (int i = 0; i < NUM_SENSORS; i++) {
            if (sensorValues[i] > sensorThreshold[i]) {
              lineDetected = true;
              break;
            }
          }
          
          if (!lineDetected) {
            updateNeoPixels();
            return;
          }
        }
        break;
        
      case NO_LINE:
        turnAroundMillis();
        break;
        
      case RIGHT_LINE:
        linePosition = CENTER_LINE;
        robotState = FOLLOW_LINE;
        moveForwardPID(baseSpeed, baseSpeed, false, true);
        break;
        
      case CENTER_LINE:
      default:
        moveForwardPID(baseSpeed, baseSpeed, false, true);
        break;
    }
  }

  updateNeoPixels();
}

// Sensor Functions
void readSensors() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    sensorValues[i] = analogRead(sensorPins[i]);
  }
}

void calibrateSensors() {
  static bool firstRun = true;
  readSensors();

  if (firstRun) {
    for (int i = 0; i < NUM_SENSORS; i++) {
      sensorMin[i] = 1023;
      sensorMax[i] = 0;
    }
    firstRun = false;
  }

  if (_leftTicks > target || _rightTicks > target) {
    stopMotors();
    sensorsCalibrated = true;

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

int calculateLinePosition() {
  long weightedSum = 0;
  long sum = 0;

  for (int i = 0; i < NUM_SENSORS; i++) {
    int value = sensorValues[i];
    weightedSum += (long)value * i * 1000;
    sum += value;
  }

  return weightedSum / sum;
}

float getDistance() {
  static unsigned long lastReadTime = 0;
  static float lastMeasurement = -1;
  unsigned long currentTime = millis();
  
  if (currentTime - lastReadTime >= 200) {
    lastReadTime = currentTime;
    
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    
    long duration = pulseIn(ECHO_PIN, HIGH, 30000);
    float distance = duration * 0.034 / 2;
    
    if (distance > 0 && distance < MAX_DISTANCE) {
      lastMeasurement = distance;
    } else {
      lastMeasurement = -1;
    }
  }
  
  return lastMeasurement;
}

// Movement Functions
void moveForward(int _leftSpeed, int _rightSpeed) {
  analogWrite(MOTOR_A_2, _rightSpeed);
  analogWrite(MOTOR_B_2, _leftSpeed);
}

void stopMotors() {
  analogWrite(MOTOR_A_1, 0);
  analogWrite(MOTOR_A_2, 0);
  analogWrite(MOTOR_B_1, 0);
  analogWrite(MOTOR_B_2, 0);
  updateNeoPixels();
}

void turn180(int _leftSpeed, int _rightSpeed) {
  analogWrite(MOTOR_A_1, _rightSpeed);
  analogWrite(MOTOR_A_2, 0);
  analogWrite(MOTOR_B_1, 0);
  analogWrite(MOTOR_B_2, _leftSpeed);
}

void resetTicks() {
  _leftTicks = 0;
  _rightTicks = 0;
}

void moveForwardPID(int _leftSpeed, int _rightSpeed, bool withOutLine, bool lineTracking) {
  if (withOutLine) {
    Kp = 6.5;
    Ki = 0.1;
    Kd = 5;

    error = _leftTicks - _rightTicks;
    integral += error;
    derivative = error - lastError;
    lastError = error;
    integral = constrain(integral, -10, 10);

  } else if (lineTracking) {
    readSensors();
    int position = calculateLinePosition();

    Kp = 0.4;
    Ki = 0.00001;
    Kd = 0.15;

    int center = (NUM_SENSORS - 1) * 1000 / 2;
    error = position - center;
    integral += error;
    derivative = error - lastError;
    lastError = error;   
  }

  correction = (Kp * error) + (Ki * integral) + (Kd * derivative);
  _leftSpeed -= correction;
  _rightSpeed += correction;

  _leftSpeed = constrain(_leftSpeed, 0, 255);
  _rightSpeed = constrain(_rightSpeed, 0, 255);

  moveForward(_leftSpeed, _rightSpeed);
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
    
    analogWrite(MOTOR_A_2, 140);
    analogWrite(MOTOR_B_2, 0);
    
    robotState = TURNING_LEFT;
    motionComplete = false;
    updateNeoPixels();
  }

  if (robotState == TURNING_LEFT) {
    lastCheck = millis();
    if (_rightTicks >= targetPulses) {
      stopMotors();
      robotState = FOLLOW_LINE;
      motionComplete = true;
      linePosition = CENTER_LINE;
      updateNeoPixels();
    }
  }
}

void turnRightMillis(int angle) {
  static unsigned long lastCheck = 0;
  const unsigned long checkInterval = 5;
  static int targetPulses;

  if (robotState != TURNING_RIGHT) {
    resetTicks();
    targetPulses = 0;
    float turnDistance = (angle / 360.0) * turn_Circumference;  
    targetPulses = (turnDistance / WHEEL_CIRCUMFERENCE) * PULSE_PER_REVOLUTION;

    analogWrite(MOTOR_A_1, 0);
    analogWrite(MOTOR_A_2, 0);
    analogWrite(MOTOR_B_1, 0);
    analogWrite(MOTOR_B_2, 0);
    
    analogWrite(MOTOR_A_2, 0);
    analogWrite(MOTOR_B_2, 90);
    
    robotState = TURNING_RIGHT;
    motionComplete = false;
  }

  if (robotState == TURNING_RIGHT) {
    lastCheck = millis();
    if (_leftTicks >= targetPulses) {
      stopMotors();
      robotState = FOLLOW_LINE;
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

    float turnDistance = (3.14 * (DISTANCE_BETWEEN_WHEELS / 2));
    targetPulses = (turnDistance / WHEEL_CIRCUMFERENCE) * PULSE_PER_REVOLUTION;

    turn180(140, 140);
    robotState = TURNING_AROUND;
    motionComplete = false;
    updateNeoPixels();
  }

  if (robotState == TURNING_AROUND) {
    sensorValues[0] = analogRead(sensorPins[0]);
    if (sensorValues[0] > sensorThreshold[0] || sensorValues[4] > sensorThreshold[4]) {
      stopMotors();
      robotState = FOLLOW_LINE;
      motionComplete = true;
      linePosition = CENTER_LINE;
      updateNeoPixels();
    }
  }
}

// Gripper Functions
void gripper(int pulse) {
  static unsigned long timer;
  static int lastPulse;
  if (millis() > timer) {
    if (pulse > 0) {
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

// NeoPixel Functions
void updateNeoPixels() {
  unsigned long currentMillis = millis();
  
  if (robotState == TURNING_LEFT || robotState == TURNING_RIGHT || robotState == TURNING_AROUND) {
    if (currentMillis - lastFlashTime >= flashInterval) {
      lastFlashTime = currentMillis;
      flashState = !flashState;
    }
    
    if (robotState == TURNING_LEFT) {
      if (flashState) {
        pixels.setPixelColor(0, pixels.Color(255, 165, 0));
        pixels.setPixelColor(3, pixels.Color(255, 165, 0));
        pixels.setPixelColor(1, pixels.Color(0, 0, 0));
        pixels.setPixelColor(2, pixels.Color(0, 0, 0));
      } else {
        pixels.setPixelColor(0, pixels.Color(0, 0, 0));
        pixels.setPixelColor(1, pixels.Color(0, 0, 0));
        pixels.setPixelColor(2, pixels.Color(0, 0, 0));
        pixels.setPixelColor(3, pixels.Color(0, 0, 0));
      }
      pixels.show();
    
      if (flashState) {
        pixels.setPixelColor(0, pixels.Color(0, 0, 0));
        pixels.setPixelColor(3, pixels.Color(0, 0, 0));
        pixels.setPixelColor(1, pixels.Color(255, 165, 0));
        pixels.setPixelColor(2, pixels.Color(255, 165, 0));
      } else {
        pixels.setPixelColor(0, pixels.Color(0, 0, 0));
        pixels.setPixelColor(1, pixels.Color(0, 0, 0));
        pixels.setPixelColor(2, pixels.Color(0, 0, 0));
        pixels.setPixelColor(3, pixels.Color(0, 0, 0));
      }
      pixels.show();
    }
  } else if (robotState == FOLLOW_LINE) {
    pixels.setPixelColor(2, pixels.Color(255, 255, 255));
    pixels.setPixelColor(3, pixels.Color(255, 255, 255));
    pixels.setPixelColor(0, pixels.Color(255, 0, 0));
    pixels.setPixelColor(1, pixels.Color(255, 0, 0));
    pixels.show();
  } else {
    setColor(255, 0, 0);
  }
}

void setColor(uint8_t r, uint8_t g, uint8_t b) {
  for(int i=0; i<NUM_PIXELS; i++) {
    pixels.setPixelColor(i, pixels.Color(r, g, b));
  }
  pixels.show();
}

void setSplitColors(uint8_t leftR, uint8_t leftG, uint8_t leftB, 
                    uint8_t rightR, uint8_t rightG, uint8_t rightB) {
  int middle = NUM_PIXELS / 2;
  
  for(int i=0; i<middle; i++) {
    pixels.setPixelColor(i, pixels.Color(leftR, leftG, leftB));
  }
  
  for(int i=middle; i<NUM_PIXELS; i++) {
    pixels.setPixelColor(i, pixels.Color(rightR, rightG, rightB));
  }
  
  pixels.show();
}

// Line Position Functions
void getLinePosition() {
  readSensors();
  
  leftTurn = 
    sensorValues[4] > sensorThreshold[4] && 
    sensorValues[5] > sensorThreshold[5] && 
    sensorValues[6] > sensorThreshold[6] && 
    sensorValues[7] > sensorThreshold[7] && 
    sensorValues[0] < sensorThreshold[0] && 
    sensorValues[1] < sensorThreshold[1] && 
    sensorValues[2] < sensorThreshold[2];
  
  rightTurn = 
    sensorValues[5] < sensorThreshold[5] && 
    sensorValues[6] < sensorThreshold[6] && 
    sensorValues[7] < sensorThreshold[7] && 
    sensorValues[0] > sensorThreshold[0] && 
    sensorValues[1] > sensorThreshold[1] && 
    sensorValues[2] > sensorThreshold[2];
  
  tJunctionOrBase = 
    sensorValues[0] > sensorThreshold[0] && 
    sensorValues[1] > sensorThreshold[1] && 
    sensorValues[2] > sensorThreshold[2] && 
    sensorValues[3] > sensorThreshold[3] && 
    sensorValues[4] > sensorThreshold[4] && 
    sensorValues[5] > sensorThreshold[5] && 
    sensorValues[6] > sensorThreshold[6] && 
    sensorValues[7] > sensorThreshold[7];
  
  deadEnd = 
    sensorValues[0] < sensorThreshold[0] && 
    sensorValues[1] < sensorThreshold[1] && 
    sensorValues[2] < sensorThreshold[2] && 
    sensorValues[3] < sensorThreshold[3] && 
    sensorValues[4] < sensorThreshold[4] && 
    sensorValues[5] < sensorThreshold[5] && 
    sensorValues[6] < sensorThreshold[6] && 
    sensorValues[7] < sensorThreshold[7];
  
  if(motionComplete) {
    if (leftTurn) {
      linePosition = LEFT_LINE;
    } else if (rightTurn) {
      linePosition = RIGHT_LINE;
    } else if (deadEnd) {
      linePosition = NO_LINE;
    } else if (tJunctionOrBase){
      linePosition = T_JUNCTION;
    } else {
      linePosition = CENTER_LINE;
    }
  }
}

void checkPathAhead() {
  static bool checkingPath = false;
  static int forwardTicks = 10;
  static LinePosition storedPosition = CENTER_LINE;
  pathChecked = false;

  if (!checkingPath) {  
    storedPosition = linePosition;
    resetTicks();
    moveForwardPID(200, 200, true, false);
    checkingPath = true;
  }

  if (checkingPath && (_leftTicks >= forwardTicks || _rightTicks >= forwardTicks)) {
    stopMotors();
    checkingPath = false;
    readSensors();
    bool pathExists = sensorValues[2] > sensorThreshold[2] || sensorValues[3] > sensorThreshold[3] || sensorValues[4] > sensorThreshold[4] || sensorValues[5] > sensorThreshold[5];

    if (pathExists) {
      robotState = FOLLOW_LINE;
      linePosition = CENTER_LINE;
      motionComplete = true;
      pathChecked = true;
    } else {
      linePosition = storedPosition;
      motionComplete = false;
      pathChecked = true;
    }
    
    updateNeoPixels();
  }
}