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
const int PPR = 20; // Pulses per revolution (example value)
const float WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * 3.14159;

// 180° turn requires two full wheel rotations
const int P_TURN_180 = PPR;

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
    turnAround();  // ✅ Perform a precise 180° turn
    delay(2000);   // Pause for observation
}

// 📌 **180° Turn Function (Fixed Right Wheel)**
void turnAround() {
    Serial.println("Turning Around (180°)...");

    // Reset encoder counts
    pulseCountR1 = 0;
    pulseCountR2 = 0;

    int baseSpeed = 180;   // Normal turning speed
    int slowSpeed = 120;   // Slow speed near stopping point
    int correctionThreshold = 5;

    while (pulseCountR1 < P_TURN_180 && pulseCountR2 < P_TURN_180) {
        int remainingPulses = P_TURN_180 - max(pulseCountR1, pulseCountR2);
        int speed = (remainingPulses < 10) ? slowSpeed : baseSpeed;

        // Apply correction if one wheel moves faster
        if (pulseCountR1 > pulseCountR2 + correctionThreshold) {
            analogWrite(MOTOR_A1, speed - 30); // Left Forward (Slower)
            analogWrite(MOTOR_A2, 0);
            analogWrite(MOTOR_B1, speed);      // Right Backward (Normal)
            analogWrite(MOTOR_B2, 0);
        } else if (pulseCountR2 > pulseCountR1 + correctionThreshold) {
            analogWrite(MOTOR_A1, speed);      // Left Forward (Normal)
            analogWrite(MOTOR_A2, 0);
            analogWrite(MOTOR_B1, speed - 30); // Right Backward (Slower)
            analogWrite(MOTOR_B2, 0);
        } else {
            analogWrite(MOTOR_A1, speed);  // ✅ Left Forward
            analogWrite(MOTOR_A2, 0);
            analogWrite(MOTOR_B1, speed);  // ✅ Right Backward
            analogWrite(MOTOR_B2, 0);
        }
    }

    stopMotors();
    Serial.println("180° Turn Complete.");
}

// 📌 **Stop Motors Function**
void stopMotors() {
    analogWrite(MOTOR_A1, 0);
    analogWrite(MOTOR_A2, 0);
    analogWrite(MOTOR_B1, 0);
    analogWrite(MOTOR_B2, 0);
}
