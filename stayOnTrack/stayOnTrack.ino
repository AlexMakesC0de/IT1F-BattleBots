//Line Sensors
#define NUM_SENSORS 8  // Number of sensors
int sensorPins[NUM_SENSORS] = {A0, A1, A2, A3, A4, A5, A6, A7};  // Sensor pin mapping
int sensorValues[NUM_SENSORS];  // Array to store sensor readings
int sensorMin[NUM_SENSORS];
int sensorMax[NUM_SENSORS];
int sensorThreshold[NUM_SENSORS]; // Array to store thresholds for each sensor


// Motor control pins (adapted for two connections per motor)
const int MOTOR_A_1 = 9;  // Left Forward
const int MOTOR_A_2 = 8;  // Left Backward
const int MOTOR_B_1 = 10;  // Right Forward
const int MOTOR_B_2 = 11; // Right Backward

int error = 0, lastError = 0;
float integral = 0;
float derivative = 0;

//PID
float Kp;  // Proportional Gain
float Ki;  // Integral Gain
float Kd;  // Derivative Gain
int correction; 

void setup() {
    Serial.begin(9600);
    
    pinMode(MOTOR_A_1, OUTPUT);
    pinMode(MOTOR_A_2, OUTPUT);
    pinMode(MOTOR_B_1, OUTPUT);
    pinMode(MOTOR_B_2, OUTPUT);
    digitalWrite(MOTOR_A_1, HIGH);
    digitalWrite(MOTOR_A_2, HIGH);
    digitalWrite(MOTOR_B_1, HIGH);
    digitalWrite(MOTOR_B_2, HIGH);
}

void loop() {
    readSensors();
    int position = calculateLinePosition();

    Serial.print("Line Position: ");
    Serial.println(position);

    followLine(position);

    delay(50);
}

// Function to read sensor values
void readSensors() {
    for (int i = 0; i < NUM_SENSORS; i++) {
        sensorValues[i] = analogRead(sensorPins[i]);
    }
}

// Function to calculate line position based on sensor readings
int calculateLinePosition() {
    long weightedSum = 0;
    long sum = 0;

    for (int i = 0; i < NUM_SENSORS; i++) {
        int value = sensorValues[i];
        weightedSum += (long)value * i * 1000;
        sum += value;
    }

      return weightedSum / sum;  // No line detected
 
}

// Function to follow the line
void followLine(int position) {

    Kp = 2.1;  // Proportional Gain
    Ki = 0.0002;  // Integral Gain
    Kd = 0.2;  // Derivative Gain

    int center = (NUM_SENSORS - 1) * 1000 / 2;  // Midpoint of sensor array
    error = position - center;  // Deviation from center
    integral += error;                  // Accumulate the error over time
    derivative = error - lastError;      // Calculate change in error
    correction = (Kp * error) + (Ki * integral) + (Kd * derivative);
    Serial.println(error);

    // Step 3: Adjust Motor Speeds Based on PID Correction
    int baseSpeed = 255;  // Base speed for both motors
    int _leftSpeed = baseSpeed - correction;
    int _rightSpeed = baseSpeed + correction;

    // Ensure PWM values are within valid range (0 - 255)
    _leftSpeed = constrain(_leftSpeed, 0, 255);
    _rightSpeed = constrain(_rightSpeed, 0, 255);

    moveForward(_leftSpeed, _rightSpeed);
}

// Move Forward
void moveForward(int _leftSpeed, int _rightSpeed) {
    analogWrite(MOTOR_A_1, _leftSpeed);
    analogWrite(MOTOR_A_2, 0);  

    analogWrite(MOTOR_B_1, _rightSpeed);
    analogWrite(MOTOR_B_2, 0);
}

// Stop Motors
void stopMotors() {
    analogWrite(MOTOR_A_1, 0);
    analogWrite(MOTOR_A_2, 0);

    analogWrite(MOTOR_B_1, 0);
    analogWrite(MOTOR_B_2, 0);
}
