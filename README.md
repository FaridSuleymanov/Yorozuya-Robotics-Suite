# Project Kagura, YATO-Umbrella System, and Project Megumin

This repository contains the source code and documentation for three experimental robotic systems: Project Kagura, YATO-Umbrella System, and Project Megumin, along with the Transmitter Controller used to operate them.
These systems focus on mobile ground operations, multi-sensor remote surveillance, and vector-thrust controlled rocketry.

## Systems Overview

### Kagura System

The Kagura System is a modular ground robotics platform designed for recreational robotics, disaster response research, and experimental AI applications. It integrates real-time motion control, telemetry acquisition, and sensor fusion capabilities.
Core Architecture

Microcontroller Arduino Mega 2560

High-Level Controller NVIDIA Jetson Platform (Serial-linked)

Motors

Dual Cytron Motor Drivers (PWM & Direction Control)

Sensors

MPU6050 (IMU), Dual Quadrature Encoders, GPS, Digital Compass

Communications

nRF24L01 RF Module (Wireless Joystick) + Serial Link

Peripheral Controls

Dual Relays (Lights/Alarm), Motor Power Relays

Key Features

Joystick-based skid-steer driving

Motion stabilization with MPU6050 (low-pass filtered)

Sensor fusion combining GPS, Compass, and Encoders

Emergency motor shutdown capability

Real-time telemetry output to Jetson

Modular design using COTS (commercial off-the-shelf) components

### YATO-Umbrella System

The YATO-Umbrella System is a high-precision multi-sensor turret platform designed for situational awareness, tracking, and non-lethal remote reconnaissance.

Core Architecture

Component

Description

Primary Controller

Arduino Mega 2560

High-Level Controller

NVIDIA Jetson Orin Nano

Sensors

FLIR Lepton Thermal Camera, Raspberry Pi Camera (Optical Zoom), Laser Rangefinder (200m), Dual MPU6050 IMUs

Motors and Actuation

DM542 and DM860H Stepper Drivers, TMC2208 Driver, Dual Servo System

Communications

nRF24L01 RF Module + Serial Link

Key Features

Precision multi-axis targeting with stabilization

Hybrid thermal and visual imaging

Real-time laser distance measurement up to 200 meters

Remote joystick control with intuitive mapping

Emergency shutdown functionality for motors

Fine microstepping for high fidelity movement

Modular structure for upgrades and expansion

### Project Megumin

Project Megumin focuses on the vector thrust control (TVC) of a solid-fuel rocket engine using:

Arduino Nano microcontroller

Dual-axis servo actuation

MPU6050 IMU for stabilization sensing

Full stabilization relying only on thrust vectoring

Source file: rocket_stabilization_test_code.ino

### Transmitter Controller

The Transmitter Controller is a custom-built wireless controller that:

Communicates with Kagura and YATO-Umbrella via nRF24L01

Features a joystick and button interface

Displays a status bitmap on an OLED display

Source file: transmitter_for_mobilized_platform_and_turret_test_code.ino

# Installation

Clone the repository.
1)Open the .ino files in the Arduino IDE.
2)Connect your Arduino board via USB.
3)Select the correct board and port in the Arduino IDE.
4)Upload the code to your board.

# Usage

Use the joystick and button on the transmitter controller to remotely operate Kagura and YATO-Umbrella systems.

The joystick controls motor movement.

The button toggles lights, alarms, or payload actions depending on the system.

# Dependencies

Ensure the following libraries are installed in the Arduino IDE:

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

# Contributing

Pull requests are welcome. For major changes, please open an issue first to discuss proposed improvements.

# License

This project is licensed under the GNU General Public License v3.0. See the LICENSE file for details.

# Compliance and Risk Acknowledgment Statement

## Civilian Purpose Declaration

I, Farid Suleymanov, hereby declare the following: All systems developed under this project are intended exclusively for civilian, non-lethal, and non-military purposes, including:

Recreational robotics activities

Disaster response research and development

Civil defense resilience initiatives

Non-lethal drone countermeasure testing

At no point have the systems been configured, tested, or marketed for military, lethal, or classified purposes.

### Materials and Components Sourcing

All components and subsystems are sourced from commercially available, unrestricted suppliers accessible to the public. No ITAR-controlled, EAR-controlled, classified, or military-restricted components are utilized.

### Misuse Prevention Engineering

The systems incorporate engineering safeguards, including:

Structural failure modes under excessive stress

Sensor-triggered shutdown protocols in the event of unauthorized modifications

Economic barriers to illegal weaponization via expensive material requirements

### Ethical and Legal Compliance

The systems are designed to:

Comply with applicable local, national, and international civilian technology laws

Uphold ethical standards of civilian research and development

Minimize dual-use risks through careful engineering and operational procedures

# Responsibility Acknowledgment

### While every reasonable precaution has been taken during the design and development of these systems, any unauthorized misuse, illegal adaptation, or modification is beyond the control and responsibility of the original developer. Such misuse shall be subject to applicable laws and legal consequences for the responsible party.
