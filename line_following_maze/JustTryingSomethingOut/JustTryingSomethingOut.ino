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
static const int DISTANCE_FROM_BASE_TO_CONE = 33; // Distance is in ticks
const int target = DISTANCE_FROM_BASE_TO_CONE;

// Encoder Pulse Counters
volatile signed int _leftTicks = 8;
volatile signed int _rightTicks = 0;

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
  
  if(coneInSquare && !sensorsCalibrated){
    //calibrateSensors
    calibrateSensors();
  }

  if (sensorsCalibrated && !conePickedUp) {
    //pickUpCone
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
    Serial.print("Robotstate: "); Serial.println(robotState);
    Serial.print("LinePosition: "); Serial.println(linePosition);
    

    if (linePosition != CENTER_LINE) {
      // checkPathAhead();
      // if(!pathChecked) return;
      if (linePosition == T_JUNCTION) {
        turnLeftMillis(90);
        Serial.println(robotState);
        Serial.println("TURNING_LEFT2");
      } else if (linePosition == LEFT_LINE) {
        turnLeftMillis(80);
        Serial.println(robotState);
        Serial.println("TURNING_LEFT");
        
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
          return; // Keep turning until line is detected
        }
      } else if (linePosition == NO_LINE) {
        turnAroundMillis();
        Serial.println(robotState);
        Serial.println("TURNING_AROUND");
      } else if (linePosition == RIGHT_LINE) {
        linePosition == CENTER_LINE;
        robotState = FOLLOW_LINE;
        //turnRightMillis(90);
        moveForwardPID(baseSpeed, baseSpeed, false, true);
        Serial.println(robotState);
        Serial.println("TURNING_RIGHT");        
      }
    }

    if (linePosition == CENTER_LINE) {
      moveForwardPID(baseSpeed, baseSpeed, false, true);
      Serial.println(robotState);
      Serial.println("FollowLine");
    }
  }

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
    Serial.print("Checking Path: ");
  }

  // Wait until the robot moves forward by forwardTicks
  if (checkingPath && (_leftTicks >= forwardTicks || _rightTicks >= forwardTicks)) {
    stopMotors();
    checkingPath = false;
    Serial.println("PathChecked");
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
    Serial.print("Target Pulses: "); Serial.println(targetPulses);

    analogWrite(MOTOR_A_1, 0);
    analogWrite(MOTOR_A_2, 0);
    analogWrite(MOTOR_B_1, 0);
    analogWrite(MOTOR_B_2, 0);
    
    // Right motor forward, left motor stopped for left turn
    analogWrite(MOTOR_A_2, 140);  // Right Forward
    analogWrite(MOTOR_B_2, 0);    // Left Forward (stopped)
    
    robotState = TURNING_LEFT;  //Lock state to "TURNING_LEFT"
    motionComplete = false;
  }

  if (robotState == TURNING_LEFT) {
    lastCheck = millis();
    Serial.print("Turning Left: "); Serial.println(robotState);
    if (_rightTicks >= targetPulses) {
      stopMotors();
      //Serial.println(robotState);
      robotState = FOLLOW_LINE;  // Unlock state after turn is complete
      Serial.print("Turn Complete ");
      motionComplete = true;
      linePosition = CENTER_LINE;
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
    Serial.print("Target Pulses: "); Serial.println(targetPulses);

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
    Serial.print("Turning Right: "); Serial.println(robotState);
    if (_leftTicks >= targetPulses) {
      stopMotors();
      //Serial.println(robotState);
      robotState = FOLLOW_LINE;  // Unlock state after turn is complete
      Serial.print("Turn Complete ");
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
    Serial.print("Target Pulses: "); Serial.println(targetPulses);
    
    turn180(140, 160); // Left wheel moves forward, right moves backward

    robotState = TURNING_AROUND; // Lock state to "TURNING_AROUND"
    motionComplete = false;
  }

  if (robotState == TURNING_AROUND) {
    Serial.print("Target Pulses: "); Serial.println(targetPulses);
    sensorValues[0] = analogRead(sensorPins[0]);
    if (sensorValues[0] > sensorThreshold[0] || sensorValues[4] > sensorThreshold[4]) {
      stopMotors();
      robotState = FOLLOW_LINE;  // Unlock state after turn is complete
      Serial.println("Turn Around Complete");
      motionComplete = true;
      linePosition = CENTER_LINE;
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
  Serial.println("Stoping");
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