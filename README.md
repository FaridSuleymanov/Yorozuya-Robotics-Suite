# Yorozuya-Robotics-Suite
*A family of civilian, open‑source robotic platforms for ground and air operations*

**License:** GPL‑3.0   •   **Maintainer:** Farid Suleymanov   •   **Development status:** Early stage, prototypes only   •   **Last update:** 2025‑04‑27

---

## 1  Overview
This repository aggregates firmware, electronics documents and build guides for three experimental robots plus their shared transmitter:

| System | Role | Highlights |
|--------|------|-----------|
| **Kagura UGV** | Modular tracked ground rover | Joystick‑driven skid‑steer, real‑time sensor fusion, Jetson AI coprocessor |
| **YATO‑Umbrella Turret** | Multi‑sensor pan‑tilt head | Thermal + optical imaging, laser range‑finding, micro‑stepped precision drive |
| **Okita Interceptor UAV** | EDF + rocket hybrid drone interceptor | EDF cruise, solid‑rocket sprint, non‑kinetic capture payloads |
| **Handheld TX** | Common transmitter | nRF24L01+ link, OLED status, joystick + buttons |

All hardware is built entirely from commercially available, unrestricted parts and is intended **exclusively for civilian, non‑lethal research**.

---

## 2  Hardware Matrix
| System | MCU / FC | Companion SBC | Propulsion / Actuation | Core Sensors | Primary Radio |
|--------|----------|---------------|------------------------|--------------|---------------|
| Kagura | Arduino Mega 2560 | Jetson Orin Nano | Dual DC motors via Cytron MD30C | MPU‑6050, GPS, dual encoders, HMC5883L | nRF24L01+ |
| YATO‑Umbrella | Arduino Mega 2560 | Jetson Orin Nano | 2‑axis steppers (DM542, DM860H) + dual servos | FLIR Lepton, RPi HQ‑Cam, laser rangefinder, dual MPU‑6050 | nRF24L01+ |
| Okita | Pixhawk‑compatible FC | (Opt.) Jetson Orin Nano | Twin 90 mm EDF + solid rocket booster | Dual IMU, ADS‑B receiver | 900 MHz SiK |

---

## 3  Repository Layout
```
├── firmware/               # Arduino / PlatformIO projects
│   ├── kagura_arduino
│   ├── yato_umbrella_arduino
│   ├── okita_arduino
│   └── Ed_remote_arduino
├── hardware/               # KiCad schematics, PCB, 3D models
├── docs/                   # Build guides, block diagrams, compliance
└── tools/                  # Utility scripts, calibration, log parsers
```

---

## 4  Contributing
Pull requests are welcome! Please open an issue to discuss major changes first. Run `clang-format` and `markdownlint` before submitting.

---

## 5  License
This project is distributed under the **GNU General Public License v3.0**. See `Doc/LICENSE` for the full text.

---

## 6  Compliance & Risk Statement (summary)
All systems are intended **solely for civilian, non‑lethal purposes** such as recreational robotics, disaster‑response R&D and drone counter‑measure research. Components are sourced from unrestricted commercial suppliers; no ITAR/EAR‑controlled hardware or software is used. Engineering safeguards include structural failure modes under excessive stress and firmware watchdog shutdowns on unauthorized modification.

> **Responsibility notice:** Any illegal adaptation or misuse is beyond the author’s control and subject to applicable laws. See `Doc/Compliance_&_Risk_Statement.md` for the full declaration.

