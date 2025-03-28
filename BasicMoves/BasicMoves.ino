// === PIN DEFINITIONS ===
const int MOTOR_A1 = 10; // Left Forward
const int MOTOR_A2 = 11; // Left Backward
const int MOTOR_B1 = 6;  // Right Backward
const int MOTOR_B2 = 9;  // Right Forward

const int ENCODER_R1 = 2; // Left Encoder
const int ENCODER_R2 = 3; // Right Encoder

const int GRIPPER_PIN = 4; // PWM control without servo library

// === SENSOR DEFINITIONS ===
const int sensorPins[] = {A0, A1, A2, A3, A4, A5, A6, A7};
const int numSensors = 8;
const int BLACK_THRESHOLD = 750;

// === MOVEMENT SETTINGS ===
const int baseSpeed = 160;
const int maxSpeed = 200;
const int minSpeed = 100;
const int PPR = 20;
const float WHEEL_DIAMETER = 0.065;
const float WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * 3.14159;
const int P_TURN = PPR / 2;
const int P_TURN_180 = PPR;

volatile int pulseCountR1 = 0;
volatile int pulseCountR2 = 0;

int lastError = 0;

// === GRIPPER SETTINGS ===
#define GRIPPER_OPEN 1750
#define GRIPPER_CLOSE 1220
unsigned long gripperTimer = 0;
int gripperPulse = 0;

// === SETUP ===
void setup() {
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

  // Start sequence
  openGripper();
  moveForwardDistance(35); // Move ~35cm
  closeGripper();
  turnLeft90();
}

// === LOOP ===
void loop() {
  readSensors();

  if (atIntersection()) {
    mazeDecision();
  } else if (allWhite()) {
    lostLineRecovery();
  } else {
    followLine();
  }

  gripper(gripperPulse);
}

// === SENSOR HANDLING ===
int sensorValues[8];

void readSensors() {
  for (int i = 0; i < numSensors; i++) {
    sensorValues[i] = analogRead(sensorPins[i]);
  }
}

bool allWhite() {
  for (int i = 0; i < numSensors; i++) {
    if (sensorValues[i] > BLACK_THRESHOLD) return false;
  }
  return true;
}

bool atIntersection() {
  int count = 0;
  for (int i = 0; i < numSensors; i++) {
    if (sensorValues[i] > BLACK_THRESHOLD) count++;
  }
  return (count >= 5);
}

// === MOVEMENT FUNCTIONS ===
void followLine() {
  int error = calculateLineError();
  int correction = error * 20;
  int leftSpeed = constrain(baseSpeed - correction, minSpeed, maxSpeed);
  int rightSpeed = constrain(baseSpeed + correction, minSpeed, maxSpeed);

  move(leftSpeed, rightSpeed);
  lastError = error;
}

int calculateLineError() {
  long weightedSum = 0;
  int blackCount = 0;
  for (int i = 0; i < numSensors; i++) {
    if (sensorValues[i] > BLACK_THRESHOLD) {
      weightedSum += i * 1000;
      blackCount++;
    }
  }

  if (blackCount == 0) return lastError;

  int average = weightedSum / blackCount;
  int center = (numSensors - 1) * 1000 / 2;
  return center - average;
}

void lostLineRecovery() {
  move(baseSpeed, baseSpeed / 2); // Slight right correction
}

void mazeDecision() {
  delay(200);
  readSensors();

  if (sensorValues[0] > BLACK_THRESHOLD) {
    turnLeft90();
  } else if (sensorValues[3] > BLACK_THRESHOLD || sensorValues[4] > BLACK_THRESHOLD) {
    moveForwardDistance(5); // move slightly forward
  } else if (sensorValues[7] > BLACK_THRESHOLD) {
    turnRight90();
  } else {
    turnAround180();
  }
}

void moveForwardDistance(int cm) {
  int targetPulses = (cm / WHEEL_CIRCUMFERENCE) * PPR;
  pulseCountR1 = 0;
  pulseCountR2 = 0;

  while (pulseCountR1 < targetPulses && pulseCountR2 < targetPulses) {
    move(baseSpeed, baseSpeed);
  }

  stopMotors();
}

void turnLeft90() {
  pulseCountR1 = 0;
  pulseCountR2 = 0;

  while (pulseCountR1 < P_TURN && pulseCountR2 < P_TURN) {
    analogWrite(MOTOR_A1, 0);
    analogWrite(MOTOR_A2, 180);
    analogWrite(MOTOR_B1, 0);
    analogWrite(MOTOR_B2, 180);
  }

  stopMotors();
}

void turnRight90() {
  pulseCountR1 = 0;
  pulseCountR2 = 0;

  while (pulseCountR1 < P_TURN && pulseCountR2 < P_TURN) {
    analogWrite(MOTOR_A1, 180);
    analogWrite(MOTOR_A2, 0);
    analogWrite(MOTOR_B1, 180);
    analogWrite(MOTOR_B2, 0);
  }

  stopMotors();
}

void turnAround180() {
  pulseCountR1 = 0;
  pulseCountR2 = 0;

  while (pulseCountR1 < P_TURN_180 && pulseCountR2 < P_TURN_180) {
    analogWrite(MOTOR_A1, 180);
    analogWrite(MOTOR_A2, 0);
    analogWrite(MOTOR_B1, 180);
    analogWrite(MOTOR_B2, 0);
  }

  stopMotors();
}

void move(int leftSpeed, int rightSpeed) {
  analogWrite(MOTOR_A1, leftSpeed);
  analogWrite(MOTOR_A2, 0);
  analogWrite(MOTOR_B1, 0);
  analogWrite(MOTOR_B2, rightSpeed);
}

void stopMotors() {
  analogWrite(MOTOR_A1, 0);
  analogWrite(MOTOR_A2, 0);
  analogWrite(MOTOR_B1, 0);
  analogWrite(MOTOR_B2, 0);
}

// === GRIPPER CONTROL ===
void openGripper() {
  gripperPulse = GRIPPER_OPEN;
}

void closeGripper() {
  gripperPulse = GRIPPER_CLOSE;
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
    timer = currentTime + 20;
  }
}

void countPulseR1() { pulseCountR1++; }
void countPulseR2() { pulseCountR2++; }
