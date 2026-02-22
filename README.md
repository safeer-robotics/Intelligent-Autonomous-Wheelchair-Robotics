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
- **Languages:** C++, python.
- **IoT/UI:** Blynk (control + notifications)  
- **OS:** Ubuntu Linux

## Repository structure
- `docs/` – dissertation PDF (portfolio version)  
- `media/` – screenshots/photos/diagrams  
- `hardware/` – ESP32 prototype code + wiring notes  
- `simulation/` – Webots simulation files and controllers

## Screenshots / Demo
- See project images in: `media/`
- Dissertation (portfolio PDF): `docs/`

## How to run (quick)
### Simulation (Webots R2023B)
1. Open the Webots, create new file. paste .wbt file.
2. Add wheelchair_keyboard.py controller file in controller.
3. Run the controller and test obstacle + braking behaviours

### Hardware (ESP32)
1. Open the ESP32 code in `hardware/`
2. Update Wi-Fi/Blynk placeholders
3. Upload to ESP32 and test sensors (HC-SR04 + MPU6050)

## Key learning
Safety-first design, sensor integration, testing/debugging, and clear technical documentation.
