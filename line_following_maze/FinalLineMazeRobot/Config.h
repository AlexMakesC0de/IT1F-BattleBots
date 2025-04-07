#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

// Pin Definitions - Hardware connections
#define NEOPIXEL_PIN 5      // Data pin for NeoPixel strip
#define NUM_PIXELS 4        // Total number of LEDs in the strip

#define NUM_SENSORS 8       // Number of line sensors in array

#define TRIG_PIN 12         // Ultrasonic sensor trigger
#define ECHO_PIN 13         // Ultrasonic sensor echo

#define MOTOR_A_1 11        // Right motor backward
#define MOTOR_A_2 10        // Right motor forward
#define MOTOR_B_1 9         // Left motor backward
#define MOTOR_B_2 8         // Left motor forward

#define MOTOR_R1 2          // Left wheel encoder
#define MOTOR_R2 3          // Right wheel encoder

#define SERVO 4             // Gripper servo control

// Constants for array sizes and thresholds
#define NUM_READINGS 3      // Number of readings for object detection

// Constants - Robot physical properties and behavior settings
extern const float WHEEL_CIRCUMFERENCE;       // How far robot moves in one wheel rotation (cm)
extern const int PULSE_PER_REVOLUTION;        // Encoder pulses per wheel revolution
extern const float DISTANCE_BETWEEN_WHEELS;   // Track width (cm)

extern const int DISTANCE_FROM_BASE_TO_CONE;  // Ticks to travel from base to cone

extern const int GRIPPER_OPEN;                // Servo pulse width for open gripper
extern const int GRIPPER_CLOSE;               // Servo pulse width for closed gripper
extern const int pulse;                       // Default pulse duration
extern const int gripperInterval;             // Delay between servo updates
extern const int ISR_INTERVAL;                // Minimum time between encoder counts

extern const unsigned long checkInterval;     // How often to check for objects
extern const unsigned long flashInterval;     // LED flashing speed

extern const int MAX_DISTANCE;                // Max distance the ultrasonic can detect
extern const int OBSTACLE_THRESHOLD;          // Distance to trigger obstacle detection

extern const int MAX_DISTANCE_TO_CHECK;       // Upper limit for robot detection
extern const int MIN_DISTANCE_TO_CHECK;       // Lower limit for robot detection

// Enums - Robot states to control behavior
enum RobotState { 
    FOLLOW_LINE,            
    TURNING_LEFT,           
    TURNING_RIGHT,         
    TURNING_AROUND,         
    OBSTACLE_DETECTED,     
    AVOIDING_OBSTACLE,     
    CHECKING_FOR_PATH_AHEAD
};

enum LinePosition { 
    T_JUNCTION,            
    LEFT_LINE,             
    RIGHT_LINE,             
    NO_LINE,                
    CENTER_LINE             
};

// Global Variables - Used throughout the program
extern Adafruit_NeoPixel pixels;              // LED control object
extern RobotState robotState;                 // Current robot state
extern LinePosition linePosition;             // Current line position

extern int sensorPins[NUM_SENSORS];           // Pin mappings for sensors
extern int sensorValues[NUM_SENSORS];         // Raw sensor readings
extern int sensorMin[NUM_SENSORS];            // Minimum calibration values
extern int sensorMax[NUM_SENSORS];            // Maximum calibration values
extern int sensorThreshold[NUM_SENSORS];      // Threshold for line detection

extern bool leftTurn, rightTurn, tJunctionOrBase, deadEnd;  // Line position flags

extern volatile signed int _leftTicks;        // Left encoder counter
extern volatile signed int _rightTicks;       // Right encoder counter

extern int baseSpeed;                         // Default motor speed

extern int previousTime;                      // For timing calculations
extern bool otherRobotDetected;               // Flag for robot detection
extern float distanceReadings[NUM_READINGS];  // Recent distance measurements

extern int readingCount;                      // Valid readings counter
extern int readingIndex;                      // Current position in readings array

extern unsigned long lastCheckTime;           // Last time we checked distance

// Game Variables
extern bool coneInSquare;                   
extern bool sensorsCalibrated;               
extern bool conePickedUp;                   
extern bool gameStarted;                      
extern bool coneDroppedOff;                  
extern bool gameEnded;                        
extern bool motionComplete;                   
extern bool pathChecked;                      

// PID Control Variables
extern int error, lastError;                  
extern float integral;                        
extern float derivative;                      
extern float Kp, Ki, Kd;                      // PID gain constants (Proportional – Integral – Derivative)
extern int correction;                        // PID correction value

// Movement Variables
extern int pulses;                            // Count of encoder pulses
extern int angle;                             // Angle to turn
extern int radius;                            // Robot turning radius
extern int turn_Circumference;                // Distance to complete a turn
extern float turnDistances;                   // Arc length for turns
extern unsigned long lastFlashTime;           // Last time LED state changed
extern bool flashState;                       // Current LED flash state

#endif 