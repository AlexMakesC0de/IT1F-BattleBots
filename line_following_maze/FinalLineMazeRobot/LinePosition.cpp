#include "LinePosition.h"
#include "LightSensors.h"
#include "MotorControl.h"
#include "NeoPixels.h"

// Read sensors and determine line position (left, right, center, etc.)
void getLinePosition() {
  readSensors();  // Get fresh readings
  
  // Left turn: Left sensors (4-7) detect line, right sensors (0-2) do not
  leftTurn = 
    sensorValues[4] > sensorThreshold[4] && 
    sensorValues[5] > sensorThreshold[5] && 
    sensorValues[6] > sensorThreshold[6] && 
    sensorValues[7] > sensorThreshold[7] && 
    sensorValues[0] < sensorThreshold[0] && 
    sensorValues[1] < sensorThreshold[1] && 
    sensorValues[2] < sensorThreshold[2];
  
  // Right turn: Right sensors (0-2) detect line, left sensors (5-7) do not
  rightTurn = 
    sensorValues[5] < sensorThreshold[5] && 
    sensorValues[6] < sensorThreshold[6] && 
    sensorValues[7] < sensorThreshold[7] && 
    sensorValues[0] > sensorThreshold[0] && 
    sensorValues[1] > sensorThreshold[1] && 
    sensorValues[2] > sensorThreshold[2];
  
  // T-junction or base: All sensors detect line
  tJunctionOrBase = 
    sensorValues[0] > sensorThreshold[0] && 
    sensorValues[1] > sensorThreshold[1] && 
    sensorValues[2] > sensorThreshold[2] && 
    sensorValues[3] > sensorThreshold[3] && 
    sensorValues[4] > sensorThreshold[4] && 
    sensorValues[5] > sensorThreshold[5] && 
    sensorValues[6] > sensorThreshold[6] && 
    sensorValues[7] > sensorThreshold[7];
  
  // Dead end: No sensors detect line
  deadEnd = 
    sensorValues[0] < sensorThreshold[0] && 
    sensorValues[1] < sensorThreshold[1] && 
    sensorValues[2] < sensorThreshold[2] && 
    sensorValues[3] < sensorThreshold[3] && 
    sensorValues[4] < sensorThreshold[4] && 
    sensorValues[5] < sensorThreshold[5] && 
    sensorValues[6] < sensorThreshold[6] && 
    sensorValues[7] < sensorThreshold[7];
  
  // Only update position if previous motion is complete (prevents confusion during turns)
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
      linePosition = CENTER_LINE;  // Default to center line if no condition met
    }
  }
}

// Check if there's a path ahead by moving forward slightly
void checkPathAhead() {
  static bool checkingPath = false;          // Flag for state tracking
  static int forwardTicks = 10;              // Distance to move forward
  static LinePosition storedPosition = CENTER_LINE;  // Remember original position
  pathChecked = false;                       // Reset check flag

  // Start checking phase
  if (!checkingPath) {  
    storedPosition = linePosition;           // Store current position
    resetTicks();                            // Reset encoder counters
    moveForwardPID(200, 200, true, false);   // Move forward briefly
    checkingPath = true;                     // Set checking flag
  }

  // Once we've moved forward enough
  if (checkingPath && (_leftTicks >= forwardTicks || _rightTicks >= forwardTicks)) {
    stopMotors();                            // Stop moving
    checkingPath = false;                    // End checking phase
    readSensors();                           // Read sensors
    
    // Look for line with center sensors (2-5)
    bool pathExists = sensorValues[2] > sensorThreshold[2] || 
                      sensorValues[3] > sensorThreshold[3] || 
                      sensorValues[4] > sensorThreshold[4] || 
                      sensorValues[5] > sensorThreshold[5];

    if (pathExists) {
      // Found a path - follow it
      robotState = FOLLOW_LINE;              // Start line following
      linePosition = CENTER_LINE;            // Assume centered on line
      motionComplete = true;                 // Allow new movements
      pathChecked = true;                    // Mark path as checked
    } else {
      // No path found - go back to previous plan
      linePosition = storedPosition;         // Restore original position
      motionComplete = false;                // Keep in turning state
      pathChecked = true;                    // Mark path as checked
    }
    
    updateNeoPixels();                       // Update status lights
  }
} 