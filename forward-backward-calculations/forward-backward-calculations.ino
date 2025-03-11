// Pin Definitions
const int MOTOR_A1 = 10; // Left Forwards
const int MOTOR_A2 = 11; // Left Backwards
const int MOTOR_B1 = 6;  // Right Backwards
const int MOTOR_B2 = 9;  // Right Forwards

const int ENCODER_R1 = 2; // Left Wheel Sensor
const int ENCODER_R2 = 3; // Right Wheel Sensor

volatile int pulseCountR1 = 0;
volatile int pulseCountR2 = 0;

// Wheel & Encoder Properties
const float WHEEL_DIAMETER = 0.065; // Example: 65mm = 0.065m
const int PPR = 20; // Pulses per revolution (example value)
const float WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * 3.14159; // Circumference in meters

const float TARGET_DISTANCE = 1.0; // 1 meter
bool movingForward = true; // Tracks movement direction
bool waiting = false;      // Tracks if the robot is pausing
unsigned long waitStartTime = 0; // Stores when waiting started

void countPulseR1() {
    pulseCountR1++;
}

void countPulseR2() {
    pulseCountR2++;
}

void setup() {
    Serial.begin(9600);

    // Motor Pins as Outputs
    pinMode(MOTOR_A1, OUTPUT);
    pinMode(MOTOR_A2, OUTPUT);
    pinMode(MOTOR_B1, OUTPUT);
    pinMode(MOTOR_B2, OUTPUT);

    // Encoder Inputs
    pinMode(ENCODER_R1, INPUT);
    pinMode(ENCODER_R2, INPUT);
    
    attachInterrupt(digitalPinToInterrupt(ENCODER_R1), countPulseR1, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCODER_R2), countPulseR2, RISING);

    Serial.println("Starting movement...");
}

void loop() {
    // Calculate traveled distance for both wheels
    float traveledDistanceR1 = (pulseCountR1 / (float)PPR) * WHEEL_CIRCUMFERENCE;
    float traveledDistanceR2 = (pulseCountR2 / (float)PPR) * WHEEL_CIRCUMFERENCE;
    float traveledDistance = (traveledDistanceR1 + traveledDistanceR2) / 2.0;

    // Debug: Print encoder readings
    Serial.print("Encoders -> R1: ");
    Serial.print(pulseCountR1);
    Serial.print(" | R2: ");
    Serial.print(pulseCountR2);
    Serial.print(" | Distance (Avg): ");
    Serial.print(traveledDistance);
    Serial.println(" m");

    if (waiting) {
        // If waiting time is over, start moving backward
        if (millis() - waitStartTime >= 1000) { // Wait for 1 second using millis()
            waiting = false;
            movingForward = false;
            pulseCountR1 = 0;
            pulseCountR2 = 0;
            Serial.println("Reversing...");
        }
        return; // Do nothing while waiting
    }

    if (movingForward) {
        if (traveledDistance < TARGET_DISTANCE) {
            moveForward();
        } else {
            stopMotors();
            Serial.println("Reached 1 meter. Stopping.");

            // Start waiting before reversing
            waiting = true;
            waitStartTime = millis(); // Record when waiting starts
        }
    } else {
        if (traveledDistance < TARGET_DISTANCE) {
            moveBackward();
        } else {
            stopMotors();
            Serial.println("Returned to start position. Stopping.");
            while (true); // Stop forever
        }
    }
}

// Function to move forward (Using same logic as your working test)
void moveForward() {
    analogWrite(MOTOR_A2, 0);
    analogWrite(MOTOR_B1, 0);
    analogWrite(MOTOR_A1, 255);
    analogWrite(MOTOR_B2, 255);
}

// Function to move backward (Using same logic as your working test)
void moveBackward() {
    analogWrite(MOTOR_A1, 0);
    analogWrite(MOTOR_B2, 0);
    analogWrite(MOTOR_A2, 255);
    analogWrite(MOTOR_B1, 255);
}

// Function to stop motors smoothly
void stopMotors() {
    analogWrite(MOTOR_A1, 0);
    analogWrite(MOTOR_A2, 0);
    analogWrite(MOTOR_B1, 0);
    analogWrite(MOTOR_B2, 0);
}
