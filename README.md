# Intelligent-Autonomous-Wheelchair-Robotics
Sensor-based indoor wheelchair safety: obstacle detection, emergency braking, tilt detection + IoT control (ESP32 + Webots).

## What I built
- Manual + assisted/autonomous modes with continuous safety supervision  
- Obstacle detection and avoidance using ultrasonic sensing  
- Emergency braking and speed limiting when hazards are detected  
- Stuck detection and recovery behaviour in simulation  
- Tilt/accident detection using an IMU  
- User control + safety alerts via Blynk IoT interface (prototype)

## Tech Stack
- **Hardware:** ESP32, L298N motor driver, 3× HC-SR04 (front/left/right), MPU6050 IMU  
- **Simulation:** Webots  
- **Languages:** C/C++  
- **IoT/UI:** Blynk (control + notifications)  
- **OS:** Ubuntu Linux

## Repository structure
- `docs/` – dissertation PDF (portfolio version)  
- `media/` – screenshots/photos/diagrams  
- `hardware/` – ESP32 prototype code + wiring notes  
- `simulation/` – Webots simulation files and controllers  

## Key learning
Safety-first design, sensor integration, testing/debugging, and clear technical documentation.
