# Intelligent-Autonomous-Wheelchair-Robotics
This is my MSc dissertation project. I designed and tested a smart wheelchair concept focused on indoor safety. My main goal was not “full autonomy”, but safe movement using an always-active safety layer. The wheelchair can be driven in manual mode, and it can also run in an assisted/autonomous mode, but in both cases the system keeps checking the environment and reacts when it detects danger.

I started with Webots simulation to test different indoor scenarios safely (corridors, turns, obstacles). After that, I validated the same idea using an ESP32 prototype concept with real sensors and an IoT interface. This project helped me understand how real robotics systems work: sensing → decision making → control → testing and improvement.

What I built (main functions)

Manual + assisted/autonomous modes with continuous safety supervision

Obstacle detection and avoidance using 3 ultrasonic sensors (front/left/right)

Emergency braking + speed limiting when obstacles are too close

Stuck detection and recovery behaviour in simulation (if the robot cannot move safely)

Tilt/accident detection using an IMU (MPU6050) to detect unsafe tilt or impact-like events

IoT control and alerts using the Blynk app (manual control, mode switching, and safety notifications)

How the system works (simple explanation)

The ultrasonic sensors continuously measure distance on the front, left and right sides.

If an obstacle is inside a threshold distance, the system takes safety action:

reduce speed, stop, or avoid the obstacle depending on the situation

The IMU checks for unsafe tilt or sudden changes that may indicate an accident risk.

In prototype mode, the wheelchair can also be controlled using a Blynk interface, and the user can receive safety alerts.

In this project I learned that safety is not only about “detecting”. It is also about stable readings, good thresholds, timing, and handling sensor noise so the wheelchair does not react randomly.

Tech used
Hardware (prototype concept)

ESP32

L298N motor driver

3× HC-SR04 ultrasonic sensors (front / left / right)

MPU6050 IMU (tilt / accident detection)

Simulation

Webots (tested using Webots R2023B)

Programming / Tools

C++ (embedded logic / control)

Python (simulation controller / scripting as used in my workflow)

Blynk (IoT UI: control + notifications)

Repository structure

docs/ – dissertation PDF (portfolio version)

media/ – screenshots / photos / diagrams / videos

hardware/ – ESP32 prototype code + wiring notes

simulation/ – Webots world file and controller files

Screenshots / Demo

Screenshots are in: media/

Dissertation PDF is in: docs/


How to run 
A) Webots simulation (R2023B)

Open Webots.

Go to File → create new world by pasting corridor_world.wbt file/ or create a world as u want.

create a controller folder and add wheelchair_keyboard.py in that folder.

Add or select my controller file wheelchair_keyboard.py in the controller settings.

Click Run and test:

obstacle detection (front/left/right)

braking / speed limiting behaviour

recovery logic in tricky situations

B) ESP32 prototype

Open the code from hardware/.

Replace placeholders (Wi-Fi/Blynk values) with your own credentials.

Upload to ESP32 using Arduino IDE / PlatformIO.

Connect sensors and driver (HC-SR04 + MPU6050 + motor driver).

Test sensor readings first, then test braking and alerts.

Open Blynk app to control modes and view safety notifications.

Security note

I removed real Wi-Fi, Blynk, and other secret keys from the code before uploading. The repo contains placeholders, so anyone who wants to run it on hardware must add their own credentials.

What I learned from this project

Safety-first robotics thinking (supervision + reliable behaviour)

Sensor integration and dealing with noise and timing issues

Designing control logic and testing step by step

Using simulation first, then validating on prototype hardware

Writing a full technical dissertation report with results and limitations

Author

Safeer Ahmed
Email: safeerahmed5471@gmail.com
