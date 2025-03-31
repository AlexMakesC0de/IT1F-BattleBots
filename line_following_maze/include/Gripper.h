#ifndef GRIPPER_H
#define GRIPPER_H

#define GRIPPER_PIN           4         // Servo pin for the gripper
#define GRIPPER_OPEN          1750      // 1750 μs rotation
#define GRIPPER_CLOSED        1220      // 1220 μs rotation
#define SERVO_INTERVAL        20        // Time between servo pulses (ms)

void setupGripper();
void openGripper();
void closeGripper();
void gripper(int pulse);

#endif 