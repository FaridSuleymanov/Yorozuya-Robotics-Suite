# Yorozuya Robotics Suite — Compliance & Risk Statement  
*(Applies to all UAV, UGV, and hybrid platforms described in this repository.)*

---

## 1. Civilian Application Declaration

All systems, subsystems, firmware, mechanical designs, and documentation contained in this repository are developed strictly for **civilian, non-lethal applications**, including but not limited to:

- Educational and research robotics
- Disaster relief support and search operations
- Environmental monitoring and surveying
- **Kinetic, non-destructive drone interception**
- Civil defense and public safety demonstrations
- Recreational remote-controlled vehicle activities

**No configuration, hardware, or software platform herein is intended, marketed, or knowingly supplied for direct military, paramilitary, offensive, or classified applications.**

---

## 2. Publicly Sourced, Non-Restricted Components

- **Open Market Procurement:** All electronic, mechanical, and software components are sourced from publicly accessible, unrestricted commercial vendors.
- **Export Compliance:** No components subject to **ITAR**, **EAR**, **EU Dual-Use Regulation 2021/821**, or equivalent national export control regimes are incorporated.
- **Open Firmware Licensing:** All firmware is published under the GNU GPL-3.0 license and contains no hidden encryption, obfuscation, or proprietary lockout features.

---

## 3. Safety and Engineering Safeguards

- **Emergency Hardware Cutoffs:**  
  Hardware kill switches enable immediate shutdown of propulsion, actuation, and pyrotechnic systems within 10 milliseconds.
- **Electrical Overcurrent and Thermal Protection:**  
  Independent fusing (2A for outputs, 0.5A for logic rails) combined with thermal-limited voltage regulation across systems.
- **Geofencing (UAV Systems):**  
  Optional GPS-enforced operational boundaries disable propulsion upon violation.
- **Mechanical Fail-Safes (UGV Systems):**  
  Integrated emergency stop buttons and redundant wireless E-Stop capability; motor controllers hardwired for brake-on-loss-of-signal events.
- **Telemetry and Data Logging:**  
  All control signals are archived with UTC timestamps for operational traceability and incident reconstruction.
- **Weaponization Prevention:**  
  Kinetic platforms incorporate mechanical and firmware safeguards against modification for offensive purposes or integration with lethal payloads.

---

## 4. Regulatory Alignment and Standards Reference

| Domain                  | Standard / Framework                                      | Application Example                         |
|--------------------------|-----------------------------------------------------------|---------------------------------------------|
| Electrical Safety        | IEC 61010‑1                                               | PCB creepage distances, fuse protection     |
| Electromagnetic Compatibility | CISPR 22                                          | Motor and RF emissions control              |
| RF Communications        | ETSI EN 300 328 v2.2.2                                    | nRF24L01+ modules, Wi-Fi, and telemetry links |
| UAV Flight Operations    | EU Regulation (EU) 2019/945 (Subcategory A3)              | UAV mass <25 kg and operational boundaries  |
| UGV Mechanical Safety    | ISO 13482 (as non-binding design reference)               | Skid-steer platform handling and E-stop design |
| Laser Safety             | IEC 60825-1:2014                                          | Safe deployment of optical rangefinders     |

---

## 5. Export Compliance and End-User Responsibility

Distribution, use, or modification of designs and software contained herein must comply with all applicable national and international regulations, including but not limited to:

- Radio frequency usage and transmission power limits
- Licensing or certification for propulsion units such as J-class rocket motors
- Optical surveillance and privacy regulations governing camera use
- Vehicular operation laws when crossing public roads or spaces

**End-users and distributors bear sole responsibility for compliance with applicable laws.**  
The authors of this project disclaim all liability arising from misuse, unauthorized redistribution, or derivative works.

---

## 6. Liability Disclaimer and Warranty Waiver

> **No Warranty:**  
> The Yorozuya Robotics Suite is provided *"as-is"* without any express or implied warranty of merchantability or fitness for a particular purpose.
>
> **No Guarantee of Suitability:**  
> No assurance is made regarding performance under operational conditions, compliance with localized law, or specific use-case requirements.
>
> **User Responsibility:**  
> Users assume full risk for system integration, deployment, and resulting consequences.
>
> **Indemnification Clause:**  
> By utilizing this repository, users agree to indemnify and hold harmless the original developers from all claims, damages, losses, or penalties arising from their actions.

---

# Closing Statement:

The Yorozuya Robotics Suite is an open-source platform created to advance **responsible civilian innovation**, **technological education**, and **non-lethal applications**.  
It is expressly not intended for militarization, human rights violations, or activities in breach of international law.

All users are expected to uphold the highest standards of ethical deployment and safety when utilizing these systems.

---
