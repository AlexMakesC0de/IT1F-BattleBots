// BASIC MOVES (FUNCTIONS)

// Pin Definitions
const int MOTOR_A1 = 10; // Left Forward
const int MOTOR_A2 = 11; // Left Backward
const int MOTOR_B1 = 6;  // Right Backward
const int MOTOR_B2 = 9;  // Right Forward

const int ENCODER_R1 = 2; // Left Wheel Sensor
const int ENCODER_R2 = 3; // Right Wheel Sensor

volatile int pulseCountR1 = 0;
volatile int pulseCountR2 = 0;

// Wheel & Encoder Properties
const float WHEEL_DIAMETER = 0.065; // 65mm = 0.065m
const int PPR = 20; // Pulses per revolution
const float WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * 3.14159; // 3.14159 

// Movement Variables
const int P_MOVE = (1.0 / WHEEL_CIRCUMFERENCE) * PPR; // Pulses for 1m
const int P_TURN_90 = PPR / 2;  // Pulses for a 90-degree turn

void countPulseR1() { pulseCountR1++; }
void countPulseR2() { pulseCountR2++; }

void setup() {
    Serial.begin(9600);

    pinMode(MOTOR_A1, OUTPUT);
    pinMode(MOTOR_A2, OUTPUT);
    pinMode(MOTOR_B1, OUTPUT);
    pinMode(MOTOR_B2, OUTPUT);

    pinMode(ENCODER_R1, INPUT);
    pinMode(ENCODER_R2, INPUT);
    
    attachInterrupt(digitalPinToInterrupt(ENCODER_R1), countPulseR1, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCODER_R2), countPulseR2, RISING);

    Serial.println("Robot Ready...");
}

void loop() {
    moveForward();
    delay(1000);
    
    moveBackward();
    delay(1000);
    
    turnLeft90();
    delay(1000);
    
    turnRight90();
    delay(1000);
}

// 📌 **Move Forward 1 Meter**
void moveForward() {
    Serial.println("Moving Forward 1m...");
    pulseCountR1 = 0;
    pulseCountR2 = 0;

    while (pulseCountR1 < P_MOVE && pulseCountR2 < P_MOVE) {
        analogWrite(MOTOR_A1, 180); // Left Forward
        analogWrite(MOTOR_A2, 0);
        analogWrite(MOTOR_B1, 0);
        analogWrite(MOTOR_B2, 180); // Right Forward
    }
    stopMotors();
}

// 📌 **Move Backward 1 Meter**
void moveBackward() {
    Serial.println("Moving Backward 1m...");
    pulseCountR1 = 0;
    pulseCountR2 = 0;

    while (pulseCountR1 < P_MOVE && pulseCountR2 < P_MOVE) {
        analogWrite(MOTOR_A1, 0);
        analogWrite(MOTOR_A2, 180); // Left Backward
        analogWrite(MOTOR_B1, 180); // Right Backward
        analogWrite(MOTOR_B2, 0);
    }
    stopMotors();
}

// 📌 **Turn Left 90 Degrees**
void turnLeft90() {
    Serial.println("Turning Left 90°...");
    pulseCountR1 = 0;
    pulseCountR2 = 0;

    while (pulseCountR1 < P_TURN_90 && pulseCountR2 < P_TURN_90) {
        analogWrite(MOTOR_A1, 0);
        analogWrite(MOTOR_A2, 180); // Left wheel backward
        analogWrite(MOTOR_B1, 0);
        analogWrite(MOTOR_B2, 180); // Right wheel forward
    }
    stopMotors();
}

// 📌 **Turn Right 90 Degrees**
void turnRight90() {
    Serial.println("Turning Right 90°...");
    pulseCountR1 = 0;
    pulseCountR2 = 0;

    while (pulseCountR1 < P_TURN_90 && pulseCountR2 < P_TURN_90) {
        analogWrite(MOTOR_A1, 180); // Left wheel forward
        analogWrite(MOTOR_A2, 0);
        analogWrite(MOTOR_B1, 180); // Right wheel backward
        analogWrite(MOTOR_B2, 0);
    }
    stopMotors();
}

// 📌 **Stop Motors Function**
void stopMotors() {
    analogWrite(MOTOR_A1, 0);
    analogWrite(MOTOR_A2, 0);
    analogWrite(MOTOR_B1, 0);
    analogWrite(MOTOR_B2, 0);
}
