# Project Kagura, YATO-umbrella system and Project Megumin

This repository contains the source code for the Project Kagura, YATO-umbrella system, and Project Megumin. The system is composed of three main components: the Kagura system, the YATO-umbrella system, a transmitter controller that communicates with both systems, and the Project Megumin which is focused on controlling the vector thrust of a solid fuel rocket engine.

## Kagura System

The Kagura System is an advanced modular ground platform designed for mobile operations, telemetry acquisition, and remote control.
It is currently structured for recreational robotics, disaster response research, and experimental mobile AI applications.

The system features a robust combination of lightweight mechanical design, dynamic motion control, and real-time sensor data fusion.

Core System Architecture:

Component	Description
Microcontroller	Arduino Mega 2560 (Primary Real-Time Controller)
High-Level Controller	NVIDIA Jetson platform (Connected via Serial for advanced AI processing)
Motors	Dual Cytron Motor Drivers with PWM and Direction Control (Skid-Steer Configuration)
Sensors	
MPU6050 (IMU for motion sensing and tilt compensation)

Dual Quadrature Encoders (for wheel position and speed tracking)

GPS (TinyGPSPlus library for geolocation)

Digital Compass (QMC5883L Magnetometer)
| Communications |

RF24 nRF24L01 Module (for long-range remote control via Joystick)

Serial Link (Mega → Jetson for telemetry and AI coordination)
| Peripheral Controls |

Dual Relays (Control for alarm system and lighting systems)

Dual Motor Power Relays (Enable/Disable Motors Safely)

Features:
Joystick-Based Manual Driving:
Skid-steer movement via dual motor control, driven by real-time joystick mapping (nRF24L01 wireless).

Motion Stabilization (Prototype Stage):
MPU6050 gyroscope and accelerometer data low-pass filtered for improved stability and smoother motor control.

Sensor Fusion for Navigation:
GPS, Compass, and Encoders provide hybrid location, heading, and velocity data — serialized for Jetson-side processing.

Emergency Safeguards:
Integrated physical motor relays and safety switches ensure rapid manual deactivation.

Real-Time Telemetry:
Gyroscope readings, GPS location, compass heading, joystick commands, motor relay status, and encoder speeds are continuously reported via Serial output.

Lighting and Alarm Control:
Independent lighting system and alarm system toggled remotely via joystick buttons, managed through dual relays.

Unique Capabilities:
Full skid-steer control with encoder feedback for precise ground maneuvering.

Real-time multi-sensor fusion pre-processed on Arduino Mega and offloaded to NVIDIA Jetson for future AI expansion.

Modular structure allows for rapid hardware upgrades (e.g., future addition of LIDARs, expanded sensor payloads, autonomous navigation routines).

Designed for field reliability using only open-source libraries and COTS (commercial off-the-shelf) components.

## Project Megumin

Project Megumin is focused on controlling the vector thrust of a solid fuel rocket engine. The system uses two servos to control the two axes of the vector thrust control. An MPU6050 sensor is used to measure the forces acting on the rocket, and an Arduino Nano is used to calculate how far to move the servo in order for the rocket to move upwards in a stabilized manner, controlled only by the thrust vector control (TVC).

The code for the Project Megumin is contained in the file `rocket_stabilization_test_code.ino`.

## Transmitter Controller

The transmitter controller is a device that sends control signals to both the Kagura and YATO-umbrella systems. It uses an RF24 radio module for wireless communication. The controller has a joystick and a button for user input. The joystick's X and Y positions and the button state are sent to the Kagura and YATO-umbrella systems.

The transmitter controller also has an OLED display that shows a bitmap image.

The code for the transmitter controller is contained in the file `transmitter_for_mobilized_platform_and_turret_test_code.ino`.

## Installation

1. Clone the repository to your local machine.
2. Open the `.ino` files in the Arduino IDE.
3. Connect your Arduino board to your computer.
4. Select the correct board and port in the Arduino IDE.
5. Upload the code to your Arduino board.

## Usage

Use the joystick and button on the transmitter controller to control the Kagura and YATO-umbrella systems. The joystick controls the movement of the motors, and the button toggles the lights and alarm on the Kagura system and the servo trigger on the YATO-umbrella system.

## Dependencies

This project uses the following libraries:

- `Wire.h`
- `SPI.h`
- `nRF24L01.h`
- `RF24.h`
- `MPU6050_tockn.h`
- `CytronMotorDriver.h`
- `Adafruit_GFX.h`
- `Adafruit_SSD1306.h`
- `ezButton.h`
- `I2Cdev.h`
- `MPU6050_6Axis_MotionApps20.h`
- `Servo.h`

Make sure to install these libraries in the Arduino IDE before running the code.

## Contributing

Pull requests are welcome. For major changes, please open an issue first to discuss what you would like to change.

## License

This project is licensed under the GNU General Public License v3.0 - see the `LICENSE` file for details.

##  Compliance and Risk Acknowledgment Statement
Acknowledgment of Civilian Use Compliance and Risk Mitigation for Robotic Systems

I, Farid Suleymanov, hereby declare the following:
## 1. Civilian Purpose Declaration
I acknowledge that the Robotic Systems under development are intended solely for civilian, non-lethal, and non-military applications, including but not limited to:

Recreational robotics activities

Disaster response research and development

Civil defense resilience initiatives

Drone countermeasure testing (non-lethal)

At no time has the system been configured, tested, or marketed for lethal, military, or classified use.

## 3. Materials and Components Sourcing
All components, materials, and subsystems used in this project are sourced from commercially available, unrestricted suppliers accessible to the general public.
No restricted, ITAR-controlled, EAR-controlled, classified, or proprietary military components have been incorporated.

## 4. Misuse Prevention Engineering
The platform incorporates intentional engineering safeguards to discourage and physically inhibit misuse, including:

Structural failure modes triggered by forces exceeding recreational equipment limits.

Sensor-driven shutdown protocols to detect unauthorized weaponization attempts.

Economic barriers against illegal modification by requiring impractically expensive machining for structural reinforcement.

## 6. Ethical and Legal Compliance
I have made every reasonable effort to ensure the platform:

Complies with applicable local, national, and international laws governing non-lethal civilian technology

Operates transparently and responsibly under civilian ethical standards

Minimizes dual-use risk through material, design, and procedural controls

## 8. Responsibility Acknowledgment
I fully accept and acknowledge that while every precaution has been taken,
any third-party unauthorized modification, misuse, or illegal adaptation of the system
is beyond my control, responsibility, and original intent.
Any such misuse shall be considered a violation of this declared design philosophy
and subject to applicable legal consequences for the offending party.
