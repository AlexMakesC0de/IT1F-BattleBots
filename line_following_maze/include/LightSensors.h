#ifndef LIGHT_SENSORS_H
#define LIGHT_SENSORS_H

// Constants for sensor indices
#define LEFT_CENTER_SENSOR  3    // Middle-left sensor
#define RIGHT_CENTER_SENSOR 4    // Middle-right sensor

// Function declarations
void setupLightSensors();
void readSensors();
void printLightSensorValues(); 
bool isBlack(int reading);
bool isBlack(int sensorIndex, int reading);
void updateCalibration();
void finalizeCalibration();
void calibrateSensors();
void determineLinePosition();
int getLineError();
void getSensorCombinations();

// Variables accessible to other files
extern int linePosition;
extern bool onLine;
extern int dynamicThreshold;
extern bool sensorsCalibrated;
extern int sensorThreshold[];

#endif 