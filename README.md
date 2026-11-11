<div align="center">

# IT1F BattleBots

**Arduino code for an autonomous robot that picks up a cone, solves a line maze and avoids obstacles.**

![Arduino](https://img.shields.io/badge/Arduino-C%2B%2B-00878F?logo=arduino&logoColor=white)
![Adafruit NeoPixel](https://img.shields.io/badge/library-Adafruit%20NeoPixel-555)

</div>

This is the code for Project BattleBot, a first-year project at NHL Stenden (class IT1F). The robot waits at the start until it sees the other robot, drives forward to grab a cone with its gripper, then follows a black line through a maze while steering around obstacles.

The repository also keeps the smaller test sketches written along the way, one per skill, so each part of the robot can be tried on its own.

## Features

- **Start signal.** The ultrasonic sensor waits for three readings in a row between 5 and 30 cm, which means the other robot is in front, before the run starts.
- **Sensor calibration on the move.** While driving from the start to the cone, the robot records the minimum and maximum of each of its 8 line sensors and sets each sensor's threshold halfway between.
- **Gripper.** A servo closes on the cone and holds it for the rest of the run.
- **Maze solving with the left-hand rule.** At a T-junction or a left branch it turns left, at a dead end it turns around, and otherwise it follows the line.
- **PID control.** Encoder-based PID keeps the robot driving straight and line-based PID keeps it centred on the line.
- **Obstacle avoidance.** Anything closer than 17 cm makes the robot stop and turn 180 degrees.
- **Indicator lights.** Four NeoPixels flash orange in the direction the robot is turning.

## Hardware

Pin assignments from [`Config.h`](line_following_maze/FinalLineMazeRobot/Config.h) in the final sketch:

| Part | Pins |
| --- | --- |
| 8 analog line sensors | A0 to A7 |
| Right motor (backward, forward) | 11, 10 |
| Left motor (backward, forward) | 9, 8 |
| Wheel encoders (left, right) | 2, 3 (interrupts) |
| Gripper servo | 4 |
| NeoPixel strip, 4 LEDs | 5 |
| Ultrasonic sensor (trigger, echo) | 12, 13 |

The earlier test sketches use their own pin numbers, so check the top of each sketch before uploading it.

## Getting started

1. Install the [Arduino IDE](https://www.arduino.cc/en/software).
2. Install the **Adafruit NeoPixel** library from the Library Manager.
3. Clone the repository:
   ```bash
   git clone https://github.com/AlexMakesC0de/IT1F-BattleBots.git
   ```
4. Open `line_following_maze/FinalLineMazeRobot/FinalLineMazeRobot.ino`, select your board and port, and upload.
5. Open the Serial Monitor at 9600 baud to follow what the robot detects.

Tuning values (speeds, PID gains, gripper positions, distances) are in `Config.cpp` next to the sketch.

## Project structure

```text
line_following_maze/
  FinalLineMazeRobot/          Final competition code, split into modules:
    FinalLineMazeRobot.ino       main state machine
    Config.h / Config.cpp        pins, constants and shared state
    MotorControl.*               motors, encoders, PID and turns
    LightSensors.*               reading and calibrating the line sensors
    LinePosition.*               deciding where the line is
    UltrasonicSensor.*           distance measurement
    GripperControl.*             servo gripper
    NeoPixels.*                  indicator lights
  line_following_maze.ino      Earlier modular version (include/ and src/)
  JustTryingSomethingOut/      Single-file experiment
  stayOnTrack/                 Line following test
BasicMoves/                    Drive forward, backward and turn
forward-backward-calculations/ Driving set distances with the encoders
turn_left/ turn_right/ turn_around/  Turning tests
gripper-millis/ Grab-IT-millis/      Gripper tests without delay()
Object_avoidance/ ObjectAvFixed/     Obstacle avoidance tests
stayOnTrack/                   Line following test
```

## License

This repository has no license file, so all rights are reserved by its authors.
