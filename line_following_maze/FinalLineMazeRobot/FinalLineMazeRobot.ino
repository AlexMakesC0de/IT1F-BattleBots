#include <Arduino.h>
#include "Config.h"
#include "NeoPixels.h"
#include "LightSensors.h"
#include "UltrasonicSensor.h"
#include "MotorControl.h"
#include "GripperControl.h"
#include "LinePosition.h"

void setup() {
  Serial.begin(9600);
  
  setupNeoPixels();
  setupUltrasonicSensor();
  setupMotors();
  setupGripper();
  
  // Attach Interrupts for Encoders
  attachInterrupt(digitalPinToInterrupt(MOTOR_R1), leftEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MOTOR_R2), rightEncoderISR, CHANGE);
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