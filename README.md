🚀 Project Kagura, YATO-Umbrella System, and Project Megumin
This repository contains the source code and documentation for three experimental robotic systems:
Project Kagura, YATO-Umbrella System, and Project Megumin — alongside the Transmitter Controller used to operate them.

These systems focus on mobile ground operations, multi-sensor remote surveillance, and vector-thrust controlled rocketry.

📦 Systems Overview
⚙️ Kagura System
The Kagura System is a modular ground robotics platform for recreational robotics, disaster response, and experimental AI applications.
It integrates real-time motion control, telemetry acquisition, and sensor fusion capabilities.

Core Architecture:


Component	Description
Microcontroller	Arduino Mega 2560
High-Level Controller	NVIDIA Jetson Platform (Serial-linked)
Motors	Dual Cytron Motor Drivers (PWM & Direction Control)
Sensors	MPU6050 (IMU), Dual Quadrature Encoders, GPS, Digital Compass
Communications	nRF24L01 RF Module (Wireless Joystick) + Serial Link
Peripheral Controls	Dual Relays (Lights/Alarm), Motor Power Relays
Key Features:

Joystick-based skid-steer driving

Motion stabilization (MPU6050 filtered)

Sensor fusion (GPS, Compass, Encoders)

Emergency motor kill switch

Real-time telemetry output

Modular, COTS components only

🛡️ YATO-Umbrella System
The YATO-Umbrella System is a high-precision multi-sensor turret platform designed for situational awareness, tracking, and experimental reconnaissance.

Core Architecture:


Component	Description
Primary Controller	Arduino Mega 2560
High-Level Controller	NVIDIA Jetson Orin Nano
Sensors	FLIR Lepton Thermal Camera, Pi Camera (100x Optical Zoom), Laser Rangefinder (200m), Dual MPU6050s
Motors and Actuation	DM542 and DM860H Stepper Drivers, TMC2208, Dual Servo System
Communications	nRF24L01 RF Module + Serial Link
Key Features:

Precision multi-axis stabilization

Thermal + optical image fusion

Real-time ranging up to 200 meters

Emergency motor shutdown switch

High-fidelity microstepping for smooth tracking

Fully modular and field-upgradable

🔥 Project Megumin
Project Megumin focuses on the vector thrust control (TVC) of a solid-fuel rocket engine using:

Arduino Nano microcontroller

Dual-axis servo system

MPU6050 IMU for stabilization feedback

Full stabilization solely by TVC with no aerodynamic surfaces

Source file:
rocket_stabilization_test_code.ino

🎮 Transmitter Controller
The Transmitter Controller is a custom-built wireless controller:

RF24 radio communication to Kagura and YATO-Umbrella

Joystick and button interface

Integrated OLED display for status visuals

Source file:
transmitter_for_mobilized_platform_and_turret_test_code.ino

🛠️ Installation
Clone the repository.

Open the .ino files in Arduino IDE.

Connect your Arduino board.

Select the correct board and port in Arduino IDE.

Upload the code to your board.

🚀 Usage
Use the joystick and button on the transmitter controller to remotely operate Kagura and YATO-Umbrella systems.

Joystick controls motor movement; button toggles lights, alarms, or triggers payload actions.

📚 Dependencies
Ensure the following libraries are installed in Arduino IDE:

Wire.h

SPI.h

nRF24L01.h

RF24.h

MPU6050_tockn.h

CytronMotorDriver.h

Adafruit_GFX.h

Adafruit_SSD1306.h

ezButton.h

I2Cdev.h

MPU6050_6Axis_MotionApps20.h

Servo.h

🤝 Contributing
Pull requests are welcome.
Please open an issue first to discuss any major changes.

📜 License
This project is licensed under the GNU General Public License v3.0.
See the LICENSE file for details.

⚖️ Compliance and Risk Acknowledgment Statement
Civilian Purpose Declaration
I, Farid Suleymanov, hereby declare the following:
All systems under development are intended exclusively for civilian, non-lethal, and non-military applications, including:

Recreational robotics activities

Disaster response research

Civil defense resilience initiatives

Non-lethal drone countermeasure testing

At no point have these systems been configured, tested, or marketed for lethal, military, or classified use.

Materials and Components Sourcing
All components, materials, and subsystems are sourced from commercially available, unrestricted suppliers.
No ITAR-controlled, EAR-controlled, classified, or military-specific components are used.

Misuse Prevention Engineering
The platforms incorporate safeguards including:

Structural failure modes under excessive forces

Sensor-driven shutdown protocols for unauthorized weaponization attempts

Economic barriers to illegal modification

Ethical and Legal Compliance
Operates under local, national, and international laws

Designed to uphold civilian ethical standards

Implements risk minimization through design and procedures

Responsibility Acknowledgment
While all reasonable precautions have been taken, any unauthorized misuse, modification, or illegal adaptation is beyond the original intent and control.
Such misuse is the sole responsibility of the offending party.

🧠 End of README
