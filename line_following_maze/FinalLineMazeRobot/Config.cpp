#include "Config.h"

// Constants - Physical measurements and timing values
const float WHEEL_CIRCUMFERENCE = 20.4;      // Wheel perimeter in cm
const int PULSE_PER_REVOLUTION = 20;         // Encoder ticks per wheel rotation
const float DISTANCE_BETWEEN_WHEELS = 22.75; // Width between wheels in cm

const int DISTANCE_FROM_BASE_TO_CONE = 33;   // Distance in encoder ticks

const int GRIPPER_OPEN = 1750;               // Pulse width to open gripper (μs)
const int GRIPPER_CLOSE = 1220;              // Pulse width to close gripper (μs)
const int pulse = 2000;                      // Default pulse width (μs)
const int gripperInterval = 20;              // Time between servo updates (ms)
const int ISR_INTERVAL = 20;                 // Interrupt throttling (ms)

const unsigned long checkInterval = 100;     // Object detection interval (ms)

const unsigned long flashInterval = 100;     // LED blink rate (ms)

const int MAX_DISTANCE = 50;                 // Maximum ultrasonic range (cm)
const int OBSTACLE_THRESHOLD = 17;           // Distance to count as obstacle (cm)

const int MAX_DISTANCE_TO_CHECK = 30;        // Max robot detection range (cm)
const int MIN_DISTANCE_TO_CHECK = 5;         // Min robot detection range (cm)

// Global Variables Initialization
Adafruit_NeoPixel pixels(NUM_PIXELS, NEOPIXEL_PIN, NEO_RGB + NEO_KHZ800); // LED strip setup
RobotState robotState = FOLLOW_LINE;         // Start in line following mode
LinePosition linePosition = CENTER_LINE;     // Assume centered on line

int sensorPins[NUM_SENSORS] = {A0, A1, A2, A3, A4, A5, A6, A7}; // Sensor pins
int sensorValues[NUM_SENSORS];               // Current sensor readings
int sensorMin[NUM_SENSORS];                  // Min value during calibration
int sensorMax[NUM_SENSORS];                  // Max value during calibration
int sensorThreshold[NUM_SENSORS];            // Detection thresholds

// Line Position Flags
bool leftTurn = false;                      
bool rightTurn = false;                    
bool tJunctionOrBase = false;               
bool deadEnd = false;                        

volatile signed int _leftTicks = 8;          // Left wheel encoder count
volatile signed int _rightTicks = 0;         // Right wheel encoder count

int baseSpeed = 180;                         // Normal movement speed (0-255)

int previousTime = 0;                        // Last timestamp
bool otherRobotDetected = false;             
float distanceReadings[NUM_READINGS];        // Recent distance measurements

int readingCount = 0;                        // Number of valid readings
int readingIndex = 0;                        // Current reading position

unsigned long lastCheckTime = 0;             // Last object check time

// Game Variables
bool coneInSquare = true;                   
bool sensorsCalibrated = false;             
bool conePickedUp = false;                  
bool gameStarted = false;                   
bool coneDroppedOff = false;                
bool gameEnded = false;                     
bool motionComplete = true;                
bool pathChecked = false;                   

// PID Control Variables
int error = 0;                              
int lastError = 0;                          
float integral = 0;                         
float derivative = 0;                       
float Kp, Ki, Kd;                            // PID gain constants (Proportional – Integral – Derivative)
int correction;                              // Steering adjustment

// Movement Variables
int pulses;                                  // Counter for pulses
int angle;                                   // Turning angle
int radius = DISTANCE_BETWEEN_WHEELS;        // Turning radius (cm)
int turn_Circumference = 2 * 3.14 * radius;  // Full turn distance (cm)
float turnDistances = 0;                     // Distance to turn
unsigned long lastFlashTime = 0;             // Last LED flash time
bool flashState = false;                     // LED on/off state 