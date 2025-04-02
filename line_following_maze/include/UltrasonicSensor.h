#ifndef ULTRASONIC_SENSOR_H
#define ULTRASONIC_SENSOR_H

#define TRIG_PIN              12        // Ultrasonic sensor trigger pin
#define ECHO_PIN              13        // Ultrasonic sensor echo pin

void setupUltrasonicSensor();
float getDistance();
void updateDistance();

#endif 