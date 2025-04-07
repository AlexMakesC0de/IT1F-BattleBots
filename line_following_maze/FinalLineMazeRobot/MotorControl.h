#ifndef MOTORCONTROL_H
#define MOTORCONTROL_H

#include "Config.h"

void setupMotors();
void leftEncoderISR();
void rightEncoderISR();
void moveForward(int _leftSpeed, int _rightSpeed);
void stopMotors();
void turn180(int _leftSpeed, int _rightSpeed);
void resetTicks();
void moveForwardPID(int _leftSpeed, int _rightSpeed, bool withOutLine, bool lineTracking);
void turnLeftMillis(int angle);
void turnRightMillis(int angle);
void turnAroundMillis();

#endif 