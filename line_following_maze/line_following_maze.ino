#include <Arduino.h>
#include "include/Motors.h"
#include "include/LightSensors.h"
#include "include/UltrasonicSensor.h"
#include "include/NeoPixel.h"
#include "include/Gripper.h"

// Basic settings
int baseSpeed = 150;

// Conditions
bool coneInSquare = true;
bool conePickedUp = false;
bool gameStarted = false;
bool coneDroppedOff = false;
bool gameEnded = false;
bool motionComplete = true; 
static bool pathChecked = false;
bool robotRunning = false;

void setup() {
    // Initialize Serial
    Serial.begin(9600);
    
    // Setup hardware
    setupMotors();
    setupLightSensors();
    setupUltrasonicSensor();
    setupNeoPixel();
    setupGripper();
    
    Serial.println("Robot initialized. Starting in 3 seconds...");
    delay(3000);    

    gameStarted = true;

    if (!sensorsCalibrated && coneInSquare && !conePickedUp && gameStarted) {
        // Start with open grippers
        openGripper();
        delay(100);
        
        // Calibrate sensors
        calibrateSensors();
        sensorsCalibrated = true;
        Serial.println("Sensors calibrated and start sequence executed");

        // Pick up cone
        closeGripper();
        conePickedUp = true;
        coneInSquare = false;
        delay(100);

        turnLeft90();
        delay(100);
    }
}

void loop() {
    if (gameStarted) {
        // Main control loop
        followLine();  // Follow the line using our PID control
        
        // Slow down the loop slightly for stability
        delay(10);
    }
}