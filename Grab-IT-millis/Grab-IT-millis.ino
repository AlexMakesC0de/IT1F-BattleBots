// GRIPPER
#define GRIPPER_PIN 4        // Servo pin for the gripper
#define GRIPPER_OPEN 1750    // 1750 μs rotation
#define GRIPPER_CLOSED 1220  // 1220 μs rotation
#define SERVO_INTERVAL 20    // Time between servo pulses (ms)

static unsigned long timer = 0;
static int lastPulse = GRIPPER_OPEN;

void setupGripper() {
  pinMode(GRIPPER_PIN, OUTPUT);
  digitalWrite(GRIPPER_PIN, LOW);
}

void gripper(int pulse) {
  static unsigned long timer = 0;
  static int lastPulse;
  unsigned long currentTime = millis();

  if (pulse > 0) {
    lastPulse = pulse;
  } else {
    pulse = lastPulse;
  }

  if (currentTime >= timer) {
    digitalWrite(GRIPPER_PIN, HIGH);
    delayMicroseconds(pulse);
    digitalWrite(GRIPPER_PIN, LOW);
    timer = currentTime + SERVO_INTERVAL;
  }
}

void openGripper() {
  // Send multiple pulses to ensure the servo reaches the position
  for (int i = 0; i < 25; i++) {
    gripper(GRIPPER_OPEN);
    delay(20);
  }
}

void closeGripper() {
  // Send multiple pulses to ensure the servo reaches the position
  for (int i = 0; i < 25; i++) {
    gripper(GRIPPER_CLOSED);
    delay(20);
  }
}

// MOTORS
#define MOTOR_A_1 11  // Right Backward
#define MOTOR_A_2 10  // Right Forward
#define MOTOR_B_1 9   // Left Backward
#define MOTOR_B_2 8   // Left Forward
#define MOTOR_DEVIATION 14  // Correction for motor speed imbalance
#define MOTOR_SPEED 180     // Motor forward and backward speed

#define ENCODER_R1 2  // Left Wheel Sensor
#define ENCODER_R2 3  // Right Wheel Sensor

// Wheel & Encoder Properties
const float WHEEL_DIAMETER = 0.065;  // 65mm = 0.065m
const int PPR = 20;                  // Pulses per revolution
const float WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * 3.14159;

// Movement Variables
const int P_MOVE = (0.50 / WHEEL_CIRCUMFERENCE) * PPR;  // Pulses for 0.5m
const int P_TURN_90 = PPR / 2.4;                        // Pulses for 90-degree turn
const int P_TURN_180 = PPR;                             // Pulses for 180-degree turn
const int P_MOVE_30CM = 30;                             // Pulses for 30cm movement
const int P_MOVE_40CM = 40;                             // Pulses for 40cm movement
const int P_MOVE_10CM = 10;                             // Pulses for 10cm movement
const int P_MOVE_25CM = 25;                             // Pulses for 25cm movement

// External pulse count variables
extern volatile int pulseCountR1;
extern volatile int pulseCountR2;

volatile int pulseCountR1 = 0;
volatile int pulseCountR2 = 0;

void countPulseR1() {
  pulseCountR1++;
}
void countPulseR2() {
  pulseCountR2++;
}

void setupMotors() {
  pinMode(MOTOR_A_1, OUTPUT);
  pinMode(MOTOR_A_2, OUTPUT);
  pinMode(MOTOR_B_1, OUTPUT);
  pinMode(MOTOR_B_2, OUTPUT);

  pinMode(ENCODER_R1, INPUT);
  pinMode(ENCODER_R2, INPUT);

  attachInterrupt(digitalPinToInterrupt(ENCODER_R1), countPulseR1, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_R2), countPulseR2, RISING);
}

void moveForward25cm() {
  pulseCountR1 = 0;
  pulseCountR2 = 0;

  while (pulseCountR1 < P_MOVE_25CM && pulseCountR2 < P_MOVE_25CM) {
    analogWrite(MOTOR_A_2, MOTOR_SPEED);  // Right Forward
    analogWrite(MOTOR_A_1, 0);            // Right Backward
    analogWrite(MOTOR_B_2, MOTOR_SPEED);  // Left Forward
    analogWrite(MOTOR_B_1, 0);            // Left Backward
  }
  stopMotors();
}

void stopMotors() {
  analogWrite(MOTOR_A_1, 0);
  analogWrite(MOTOR_A_2, 0);
  analogWrite(MOTOR_B_1, 0);
  analogWrite(MOTOR_B_2, 0);
}

void setup() {
  // Setup hardware
  setupMotors();
  setupGripper();
}

void loop() {
  // Initial close with continuous pulsing
  for (int i = 0; i < 50; i++) {
    gripper(GRIPPER_CLOSED);
    delay(20);
  }
  delay(1000);  // Wait 1 second

  // Open gripper
  openGripper();
  delay(1000);  // Wait 1 second

  // Close gripper again with continuous pulsing
  for (int i = 0; i < 50; i++) {
    gripper(GRIPPER_CLOSED);
    delay(20);
  }
  delay(1000);  // Wait 1 second

  // Move forward 30cm while maintaining gripper power
  moveForward25cm();
  for (int i = 0; i < 100; i++) {  // Increased iterations for longer movement
    gripper(GRIPPER_CLOSED);
    delay(20);
  }

  // Move forward 10cm while maintaining gripper power
  moveForward25cm();
  for (int i = 0; i < 50; i++) {
    gripper(GRIPPER_CLOSED);
    delay(20);
  }

  // Stop motors but keep gripper powered
  stopMotors();

  // Keep gripper powered indefinitely
  while (1) {
    gripper(GRIPPER_CLOSED);
    delay(20);
  }
}