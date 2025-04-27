# Compliance & Risk Statement

*Applicable to all hardware, firmware and documentation contained in the ****************YATO Robotics Suite**************** repository*

---

## 1  Civilian‑Use Declaration

All systems, subsystems and source files published in this repository are developed **exclusively for civilian, non‑lethal applications**, including but not limited to:

- Recreational and educational robotics
- Disaster‑response research and prototyping
- Civil‑defence resilience studies
- Non‑kinetic drone‑countermeasure experimentation

No platform herein has been configured, marketed or knowingly supplied for lethal, military or classified purposes.

---

## 2  Open, Unrestricted Components

- **Sourcing.** Every electronic component, mechanical part and software dependency is procured from publicly accessible, non‑restricted vendors.
- **Control regimes.** No item regulated under **ITAR**, **EAR**, **EU Dual‑Use Regulation 2021/821**, or equivalent national export‑control frameworks is utilised.
- **Software.** All firmware is published under the GNU GPL‑3.0 licence and contains no proprietary, encrypted or access‑controlled binaries.

---

## 3  Dual‑Use Risk Mitigation

| Risk Vector                                                                                                                     | Mitigation Measure |
| ------------------------------------------------------------------------------------------------------------------------------- | ------------------ |
| **Weaponisation** of UAV / UGV platforms                                                                                        |                    |
| • Payload interface limited to low‑power GPIO.• Structural failure modes engineered below lethal‑payload thresholds.            |                    |
| **Unauthorized firmware modification**                                                                                          |                    |
| • Signed build option and checksum verification available.• Watchdog forces safe‑state on checksum failure.                     |                    |
| **High‑energy propulsion misuse** (rocket stage)                                                                                |                    |
| • Solid‑rocket module accepts only certified **J‑class** motors; purchase, possession, or transport of such motors may require a national High‑Power Rocketry (HPR) licence or equivalent permit in many jurisdictions. Flight controller inhibits ignition unless the hardware arming key is engaged and a valid launch‑code sequence is received.
| **Privacy intrusion** (camera/thermal)                                                                                          |                    |
| • Default firmware masks telemetry ID; operator must set unique callsign.• Range‑finder laser below Class 2M IEC 60825‑1.       |                    |

---

## 4  Safety Engineering Safeguards

- **Hardware‑level failsafe:** Dedicated kill‑switch line drops all PWM outputs and opens MOSFET drivers in <10 ms.
- **Over‑current protection:** Individually fused 2 A outputs; 0.5 A fuse on logic rail; thermal‑limiting regulators.
- **Geofence:** Optional GPS‑based boundary stops propulsion when exiting user‑defined radius.
- **Telemetry audit:** All control packets logged with UTC timestamp for post‑event traceability.

---

## 5  Regulatory & Standards Alignment

| Domain              | Standard / Guideline                                           | Application                  |
| ------------------- | -------------------------------------------------------------- | ---------------------------- |
| Electrical safety   | IEC 61010‑1                                                    | PCB creepage, fuse selection |
| RF emissions        | ETSI EN 300 328 v2.2.2                                         | nRF24L01+ transceiver        |
| UAS operations (EU) | Commission Delegated Regulation (EU) 2019/945, Sub‑category A3 | Okita UAV weight < 25 kg     |
| Laser safety        | IEC 60825‑1:2014                                               | Class 2M range‑finder module |

---

## 6  Export‑Control & End‑User Responsibility

Distribution of files in this repository from jurisdictions with export‑control legislation remains the responsibility of the distributor.  End‑users must ensure compliance with local laws governing:

1. **Radio transmit power and frequency bands**
2. **Potential dual‑use or licensing requirements for J‑class solid‑rocket motors**
3. **Camera privacy regulations**

The original author provides this work on an *as‑is* basis and **accepts no liability** for violations arising from downstream redistribution or modification.

---

## 7  Liability & Warranty Disclaimer

> The YATO Robotics Suite is supplied *without any express or implied warranty*.  The author shall not be liable for any direct, indirect, incidental or consequential damages arising from its use, even if advised of the possibility of such damage.

Users assume full responsibility for integrating, operating or altering the systems, and agree to indemnify the author against all claims resulting from improper or unlawful use.

---

