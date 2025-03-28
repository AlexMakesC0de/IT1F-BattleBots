#include <Adafruit_NeoPixel.h>
 
// Pin and configuration constants
#define   GRIPPER_PIN           4         // Servo pin for the gripper
#define   GRIPPER_OPEN          1750      // Pulse width to open the gripper
#define   GRIPPER_CLOSED        1220      // Pulse width to close the gripper
#define   SERVO_INTERVAL        20        // Time between servo pulses (ms)
#define   GRIPPER_TOGGLE        1000      // Toggle gripper every second

#define   TRIG_PIN              13        // Ultrasonic sensor trigger pin
#define   ECHO_PIN              12        // Ultrasonic sensor echo pin

#define   MOTOR_A1              10        // Motor control pin (left backward)
#define   MOTOR_A2              11        // Motor control pin (left forward)
#define   MOTOR_B1              6         // Motor control pin (right backward)
#define   MOTOR_B2              9         // Motor control pin (right forward)

#define   MOTOR_DEVIATION       14        // Correction for motor speed imbalance
#define   MOTOR_SPEED           180       // Motor forward and backward speed

#define   NEOPIXEL_PIN          5         // NeoPixel LED control pin
#define   NUMPIXELS             4         // Number of NeoPixel LEDs

#define   ENCODER_R1            2         // Left Wheel Sensor
#define   ENCODER_R2            3         // Right Wheel Sensor

volatile int pulseCountR1 = 0;
volatile int pulseCountR2 = 0;

const int SAFE_DISTANCE = 20; // Detect obstacle at 20 cm

// Wheel & Encoder Properties
const float WHEEL_DIAMETER = 0.065; // 65mm = 0.065m
const int PPR = 20; // Pulses per revolution (example value)
const float WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * 3.14159;

// Movement Variables
const int P_MOVE = (0.50 / WHEEL_CIRCUMFERENCE) * PPR; // Pulses for 0.5m
const int P_TURN_90 = PPR / 2;  // Pulses for a 90-degree turn

// 180° turn requires two full wheel rotations
const int P_TURN_180 = PPR;

void countPulseR1() { pulseCountR1++; }
void countPulseR2() { pulseCountR2++; }

// Light sensor pins and initial threshold
const int LIGHT_SENSOR[8] = {A0, A1, A2, A3, A4, A5, A6, A7};
int LIGHT_VALUE  =   850;  // Initial light threshold value

Adafruit_NeoPixel pixels(NUMPIXELS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

// Timing variables for various intervals
long currentMillis;
bool currentLightState = false;
unsigned long previousMillis = 0;
const long interval = 100; // Interval for periodic tasks (ms)
bool end = false;
 
// Ultrasonic sensor timing variables
unsigned long distanceMillis = 0;
float duration = 0;

// Function to take multiple distance readings and calculate an average
int getAverageDistance(int samples = 5) {
  int sum = 0;
  int validSamples = 0;

  for (int i = 0; i < samples; i++) {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    long duration = pulseIn(ECHO_PIN, HIGH);
    int distance = duration / 58;  // Convert to cm

    // Filter out extreme outliers (e.g., 0 cm or >200 cm)
    if (distance > 0 && distance < 200) {
      sum += distance;
      validSamples++;
    }

    delay(10);  // Small delay between readings
  }

  if (validSamples == 0) return 0;  // Avoid division by zero
  return sum / validSamples;        // Return average distance
}

void setup() {
  // Configure pins
  pinMode(MOTOR_A1, OUTPUT);
  pinMode(MOTOR_A2, OUTPUT);
  pinMode(MOTOR_B1, OUTPUT);
  pinMode(MOTOR_B2, OUTPUT);

  pinMode(ENCODER_R1, INPUT);
  pinMode(ENCODER_R2, INPUT);

  attachInterrupt(digitalPinToInterrupt(ENCODER_R1), countPulseR1, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_R2), countPulseR2, RISING);

  pinMode(GRIPPER_PIN, OUTPUT);
  digitalWrite(GRIPPER_PIN, LOW);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  
  pixels.begin();
}

void loop() {
  int distance = getAverageDistance();  // Get smoothed distance

  if (distance > 0 && distance <= 20) {  // If object is 5 cm or closer
    avoidObstacle();
  } else {
    moveForward();
  }

delay(100);  // Take a new reading every 500ms
}

void avoidObstacle() {
  stopMotors();
  delay(500);

  turnLeft90();
  moveForwardTimed(1000); // Move forward for 0.5 sec

  turnRight90();
  moveForwardTimed(2000); // Move forward for 1 sec

  turnRight90();
  moveForwardTimed(1000); // Move forward for 0.5 sec

  turnLeft90();
}

void moveForwardTimed(int duration) {
  Serial.print("Moving forward for ");
  Serial.print(duration);
  Serial.println(" ms");

  analogWrite(MOTOR_A1, MOTOR_SPEED);
  analogWrite(MOTOR_A2, 0);
  analogWrite(MOTOR_B1, 0);
  analogWrite(MOTOR_B2, MOTOR_SPEED);
  delay(duration);
  stopMotors();
}

void moveForward() {
  pulseCountR1 = 0;
  pulseCountR2 = 0;

  while (pulseCountR1 < P_MOVE && pulseCountR2 < P_MOVE) {
      analogWrite(MOTOR_A1, MOTOR_SPEED); // Left Forward
      analogWrite(MOTOR_A2, 0);
      analogWrite(MOTOR_B1, 0);
      analogWrite(MOTOR_B2, MOTOR_SPEED); // Right Forward
  }
    stopMotors();
}

void moveBackward() {
  pulseCountR1 = 0;
  pulseCountR2 = 0;

  while (pulseCountR1 < P_MOVE && pulseCountR2 < P_MOVE) {
      analogWrite(MOTOR_A1, 0);
      analogWrite(MOTOR_A2, MOTOR_SPEED); // Left Backward
      analogWrite(MOTOR_B1, MOTOR_SPEED); // Right Backward
      analogWrite(MOTOR_B2, 0);
  }
    stopMotors();
}

void turnLeft90() {
  pulseCountR1 = 0;
  pulseCountR2 = 0;

  while (pulseCountR1 < P_TURN_90 && pulseCountR2 < P_TURN_90) {
    analogWrite(MOTOR_A1, 0);
    analogWrite(MOTOR_A2, MOTOR_SPEED); // Left wheel backward
    analogWrite(MOTOR_B1, 0);
    analogWrite(MOTOR_B2, MOTOR_SPEED); // Right wheel forward
  }
  stopMotors();
}

void turnRight90() {
  pulseCountR1 = 0;
  pulseCountR2 = 0;

  while (pulseCountR1 < P_TURN_90 && pulseCountR2 < P_TURN_90) {
    analogWrite(MOTOR_A1, MOTOR_SPEED); // Left wheel forward
    analogWrite(MOTOR_A2, 0);
    analogWrite(MOTOR_B1, MOTOR_SPEED + 30); // Right wheel backward
    analogWrite(MOTOR_B2, 0);
  }
  stopMotors();
}

void turnAround() {
  // Reset encoder counts
  pulseCountR1 = 0;
  pulseCountR2 = 0;

  int speed = 180;

  while (pulseCountR1 < P_TURN_180 && pulseCountR2 < P_TURN_180) {
    analogWrite(MOTOR_A1, speed);
    analogWrite(MOTOR_A2, 0);
    analogWrite(MOTOR_B1, speed);
    analogWrite(MOTOR_B2, 0);
  }

  stopMotors();
}

void followLine() {
  //
}

void stopMotors() {
  analogWrite(MOTOR_A1, 0);
  analogWrite(MOTOR_A2, 0);
  analogWrite(MOTOR_B1, 0);
  analogWrite(MOTOR_B2, 0);
}