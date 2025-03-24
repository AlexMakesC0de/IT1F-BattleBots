// Object avoidance
// Motor speed constant
const int MOTOR_SPEED = 200;

// Motor control pins
const int MOTOR_A_1 = 6;  // Left motor backward
const int MOTOR_A_2 = 9;  // Left motor forward
const int MOTOR_B_1 = 11; // Right motor backward
const int MOTOR_B_2 = 10; // Right motor forward

// Ultrasonic sensor pins
const int TRIG_PIN = 13;
const int ECHO_PIN = 12;

// Timing variables
unsigned long previousMillis = 0;
const int STOP_DURATION = 500;       // Time to stop before turning
const int TURN_DURATION_90 = 490;    // Adjust for 90-degree turn
const int FORWARD = 900;  // Move forward after turning
const int EXTRA_FORWARD = 1500;

// Movement states
enum State { MOVING_FORWARD, STOPPED, TURNING_RIGHT1, MOVING_FORWARD1, TURNING_LEFT1, MOVING_FORWARD_EXTRA, TURNING_LEFT2, MOVING_FORWARD2, TURNING_RIGHT2, MOVING_FORWARD3};
State botState = MOVING_FORWARD;

void setup() {
  pinMode(MOTOR_A_1, OUTPUT);
  digitalWrite(MOTOR_A_1, HIGH);
  pinMode(MOTOR_A_2, OUTPUT);
  digitalWrite(MOTOR_A_2, HIGH);
  pinMode(MOTOR_B_1, OUTPUT);
  digitalWrite(MOTOR_B_1, HIGH);
  pinMode(MOTOR_B_2, OUTPUT);
  digitalWrite(MOTOR_B_2, HIGH);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  Serial.begin(9600);
}

// Function to measure distance using ultrasonic sensor
int getDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH);
  int distance = duration * 0.034 / 2; // Convert to cm
  delay(50); // Small delay to stabilize readings
  return distance;
}

// Motor movement functions
void moveForward(int speed) {
  Serial.println("Moving Forward");
  analogWrite(MOTOR_A_1, 0);
  analogWrite(MOTOR_A_2, speed);
  analogWrite(MOTOR_B_1, 0);
  analogWrite(MOTOR_B_2, speed);
}

void stopBot() {
  Serial.println("Stopping");
  analogWrite(MOTOR_A_1, 0);
  analogWrite(MOTOR_A_2, 0);
  analogWrite(MOTOR_B_1, 0);
  analogWrite(MOTOR_B_2, 0);
}

void turnRight() {
  Serial.println("Turning Right...");
  analogWrite(MOTOR_A_1, MOTOR_SPEED);
  analogWrite(MOTOR_A_2, 0);
  analogWrite(MOTOR_B_1, 0);
  analogWrite(MOTOR_B_2, MOTOR_SPEED);
}

void turnLeft() {
  Serial.println("Turning Left...");
  analogWrite(MOTOR_A_1, 0);
  analogWrite(MOTOR_A_2, MOTOR_SPEED);
  analogWrite(MOTOR_B_1, MOTOR_SPEED);
  analogWrite(MOTOR_B_2, 0);
}

void loop() {
  unsigned long currentMillis = millis();
  int distance = getDistance();

  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  switch (botState) {
    case MOVING_FORWARD:
      if (distance > 15) {
        moveForward(MOTOR_SPEED);
      } else {
        stopBot();
        botState = STOPPED;
        previousMillis = currentMillis;
      }
      break;

    case STOPPED:
      if (currentMillis - previousMillis >= STOP_DURATION) {
        turnRight();
        botState = TURNING_RIGHT1;
        previousMillis = currentMillis;
      }
      break;

    case TURNING_RIGHT1:
      if (currentMillis - previousMillis >= TURN_DURATION_90) {
        moveForward(MOTOR_SPEED);
        botState = MOVING_FORWARD1;
        previousMillis = currentMillis;
      }
      break;

    case MOVING_FORWARD1:
      if (currentMillis - previousMillis >= FORWARD) {
        turnLeft();
        botState = TURNING_LEFT1;
        previousMillis = currentMillis;
      }
      break;

    case TURNING_LEFT1:
      if (currentMillis - previousMillis >= TURN_DURATION_90) {
        moveForward(MOTOR_SPEED);
        botState = MOVING_FORWARD_EXTRA;
        previousMillis = currentMillis;
      }
      break;

    case MOVING_FORWARD_EXTRA:
      if (currentMillis - previousMillis >= EXTRA_FORWARD) {
        turnLeft();
        botState = TURNING_LEFT2;
        previousMillis = currentMillis;
      }
      break;

    case TURNING_LEFT2:
      if (currentMillis - previousMillis >= TURN_DURATION_90) {
        moveForward(MOTOR_SPEED);
        botState = MOVING_FORWARD2;
        previousMillis = currentMillis;
      }
      break;

    case MOVING_FORWARD2:
      if (currentMillis - previousMillis >= FORWARD) {
        turnRight();
        botState = TURNING_RIGHT2;
        previousMillis = currentMillis;
      }
      break;

    case TURNING_RIGHT2:
      if (currentMillis - previousMillis >= TURN_DURATION_90) {
        moveForward(MOTOR_SPEED);
        botState = MOVING_FORWARD3;
        previousMillis = currentMillis;
      }
      break;

    case MOVING_FORWARD3:
      if (currentMillis - previousMillis >= FORWARD) {
        botState = MOVING_FORWARD;
      }
      break;
  }
}







