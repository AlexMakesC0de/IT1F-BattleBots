#include <Arduino.h>
#include "include/Motors.h"
#include "include/LightSensors.h"
#include "include/UltrasonicSensor.h"
#include "include/NeoPixel.h"
#include "include/Gripper.h"

// Robot State Machine
enum RobotState { FOLLOW_LINE, TURNING, TURNING_LEFT, TURNING_RIGHT, TURNING_AROUND, CHECKING_FOR_PATH_AHEAD };
enum LinePosition { T_JUNCTION, LEFT_LINE, RIGHT_LINE, NO_LINE, CENTER_LINE };
RobotState robotState = FOLLOW_LINE;
LinePosition currentLinePosition = CENTER_LINE;

// External declarations for variables defined in other files
extern int sensorValues[];  // Declared in LightSensors.cpp

// Basic settings
int baseSpeed = 200;

// Conditions
bool coneInSquare = true;
bool conePickedUp = false;
bool gameStarted = false;
bool coneDroppedOff = false;
bool gameEnded = false;
bool motionComplete = true; 
static bool pathChecked = false;
bool robotRunning = false;

// Function to determine the current line position type from sensor values
LinePosition determineCurrentLinePosition() {
    // Read sensors
    determineLinePosition();
    
    // Count active sensors on left, center, and right
    int leftActive = 0;
    int centerActive = 0;
    int rightActive = 0;
    
    // Left sensors (0-2)
    for (int i = 0; i <= 2; i++) {
        if (isBlack(i, sensorValues[i])) {
            leftActive++;
        }
    }
    
    // Center sensors (3-4)
    for (int i = 3; i <= 4; i++) {
        if (isBlack(i, sensorValues[i])) {
            centerActive++;
        }
    }
    
    // Right sensors (5-7)
    for (int i = 5; i <= 7; i++) {
        if (isBlack(i, sensorValues[i])) {
            rightActive++;
        }
    }
    
    // Determine line position based on active sensors
    if (!onLine) {
        return NO_LINE;
    } else if (leftActive >= 2 && centerActive >= 1 && rightActive >= 2) {
        return T_JUNCTION;
    } else if (leftActive >= 2 && centerActive == 0) {
        return LEFT_LINE;
    } else if (rightActive >= 2 && centerActive == 0) {
        return RIGHT_LINE;
    } else {
        return CENTER_LINE;
    }
}

// Function that handles the robot's state machine
void handleRobotState() {
    // Get current line position
    currentLinePosition = determineCurrentLinePosition();
    
    // Print current state and line position for debugging
    Serial.print("State: ");
    switch (robotState) {
        case FOLLOW_LINE: Serial.print("FOLLOW_LINE"); break;
        case TURNING: Serial.print("TURNING"); break;
        case TURNING_LEFT: Serial.print("TURNING_LEFT"); break;
        case TURNING_RIGHT: Serial.print("TURNING_RIGHT"); break;
        case TURNING_AROUND: Serial.print("TURNING_AROUND"); break;
        case CHECKING_FOR_PATH_AHEAD: Serial.print("CHECKING_FOR_PATH_AHEAD"); break;
    }
    
    Serial.print(" | Line Position: ");
    switch (currentLinePosition) {
        case T_JUNCTION: Serial.println("T_JUNCTION"); break;
        case LEFT_LINE: Serial.println("LEFT_LINE"); break;
        case RIGHT_LINE: Serial.println("RIGHT_LINE"); break;
        case NO_LINE: Serial.println("NO_LINE"); break;
        case CENTER_LINE: Serial.println("CENTER_LINE"); break;
    }
    
    // Implement the logic as provided
    if (currentLinePosition != CENTER_LINE) {
        if (currentLinePosition == T_JUNCTION) {
            turnLeft90();
            Serial.println(robotState);
            Serial.println("TURNING_LEFT2");
        } else if (currentLinePosition == LEFT_LINE) {
            turnLeft90();
            Serial.println(robotState);
            Serial.println("TURNING_LEFT");
        } else if (currentLinePosition == NO_LINE) {
            turnAround();
            Serial.println(robotState);
            Serial.println("TURNING_AROUND");
        } else if (currentLinePosition == RIGHT_LINE) {
            currentLinePosition = CENTER_LINE;
            robotState = FOLLOW_LINE;
            //turnRight90();
            moveForwardPID(baseSpeed, baseSpeed, false, true);
            Serial.println(robotState);
            Serial.println("TURNING_RIGHT");        
        }
    }

    if (currentLinePosition == CENTER_LINE) {
        moveForwardPID(baseSpeed, baseSpeed, false, true);
        Serial.println(robotState);
        Serial.println("FollowLine");
    }
}

void setup() {
    // Initialize Serial
    Serial.begin(9600);
    
    // Setup hardware
    setupMotors();
    setupLightSensors();
    setupUltrasonicSensor();
    setupNeoPixel();
    setupGripper();
    
    Serial.println("Robot initializing...");
    delay(1000);    

    gameStarted = true;

    if (!sensorsCalibrated && coneInSquare && !conePickedUp && gameStarted) {
        // Start with open grippers
        openGripper();
        delay(100);
        
        // Calibrate sensors
        calibrateSensors();
        sensorsCalibrated = true;
        
        // Pick up cone
        Serial.println("Picking up cone");
        closeGripper();
        conePickedUp = true;
        coneInSquare = false;
        delay(100);

        Serial.println("Initial turn to start maze navigation");
        turnLeft90();
        delay(100);
    }
}

void loop() {
    if (gameStarted && !gameEnded) {
        // Use state machine instead of direct followLine call
        handleRobotState();
        
        // Slow down the loop slightly for stability
        delay(10);
    }
}