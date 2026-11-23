# Lk_5843 // Neptune Platform (MR_HYDE_NODE_01)

### Deterministic Environmental and Biometric Telemetry with Edge Computing on Open Hardware
**Decide locally. Never lose a sample.**

[![C++ Standard](https://img.shields.io/badge/C%2B%2B-17-00599C?logo=cplusplus&logoColor=white)](https://isocpp.org/)
[![Hardware](https://img.shields.io/badge/MCU-ESP32--S3%20(N16R8)-E7352C?logo=espressif&logoColor=white)](https://www.espressif.com/)
[![Firmware](https://img.shields.io/badge/EDGE%20OS-2.77-1F6FEB)](#6-embedded-c-software-architecture--hardening-roadmap)
[![Protocol](https://img.shields.io/badge/Telemetry-MQTT%203.1.1%20%2F%20JSON-660066?logo=mqtt&logoColor=white)](https://mqtt.org/)
[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)

---

## 1. System Overview & Core Objectives

Project **Lk_5843** (technically designated as **Neptune**, implemented on hardware unit **MR_HYDE_NODE_01**) is a deterministic Edge computing gateway designed for real-time environmental and biometric telemetry in smart training facilities (**SmartGym**).

Built around a dual-core **ESP32-S3 (Xtensa LX7)** microcontroller and programmed in modern **C++17** (`EDGE OS 2.77`), the node acquires five physical quantities (heart rate, structural vibration, ambient temperature, air quality, and room luminosity), conditions signals locally, and closes two autonomous control loops without any cloud dependency: a proportional PWM ventilation law and a four-state finite-state machine (FSM) driving visual status signaling.

### Guiding Principles

* **Deterministic Execution:** A time-sliced cooperative scheduler based on `millis()` enforces fixed sampling periods with zero blocking `delay()` calls in the main super-loop.
* **Edge Autonomy:** Thermal regulation, impact flagging, and local state signaling execute directly on the microcontroller within milliseconds; the cloud never sits inside the control path.
* **Resilience by Design:** A statically allocated 100-record circular FIFO buffer in SRAM, non-blocking network reconnection, and boot-time presence probing for optional sensors preserve operation under connectivity loss or partial hardware degradation.
* **Transparency & Auditability:** Every decision threshold, sampling interval, and GPIO pin assignment is defined as a named compile-time `constexpr` constant, ensuring strict reproducibility between firmware and technical documentation.

---

## 2. The Problem: Cloud Latency & Telemetry Fragility in Training Facilities

High-occupancy training rooms combine intense physical exertion, thermal build-up, air quality degradation, and mechanical impacts from dropped weights. Conventional cloud-centric IoT architectures introduce critical operational vulnerabilities:

* **Control-Path Latency:** Routing ventilation or safety alerts through remote servers introduces unacceptable delays and fails completely during Wi-Fi outages.
* **Telemetry Loss Under Disconnection:** Standard stateless publishers discard sensor readings whenever the MQTT broker becomes unreachable.
* **Loop Blocking & Timing Jitter:** Naive embedded implementations stall CPU execution during slow sensor conversions (such as 1-Wire temperature reads) or network retries, corrupting high-frequency inertial and photoplethysmographic (PPG) sampling.
* **Bandwidth Overhead:** Streaming raw 50 Hz accelerometer data to the cloud wastes bandwidth when local edge filtering can extract peak impact events deterministically.

---

## 3. System Mechanics & Analytical Metrics

### 3.1 Multi-Rate Cooperative Scheduler

The main execution loop slices CPU time deterministically across five concurrent cadences:

| Cadence / Period | Subsystem | Edge Execution Task |
| :--- | :--- | :--- |
| **Every Pass** | Network & PPG | Maintain non-blocking MQTT session; sample MAX30105 IR channel and update 4-beat BPM rolling average. |
| **20 ms (50 Hz)** | Inertial Sensing | Read MPU6050 3-axis acceleration, compute gravity-compensated magnitude, and trigger immediate alert if peak > 1.8 g. |
| **150 ms (~6.6 FPS)** | Local HUD | Render real-time telemetry, NTP clock, ECG waveform, and link status (`[MQ]` / `R:n`) on the SSD1306 OLED. |
| **2 s (0.5 Hz)** | Environmental & Control | Sample temperature, gas, and light; update 10-sample moving average; evaluate proportional PWM fan law and 4-state FSM. |
| **10 s (0.1 Hz)** | Telemetry Transport | Serialize JSON frame; drain circular FIFO chronologically if connected, or enqueue frame in SRAM if offline. |

### 3.2 Signal Conditioning & Control Laws

* **Moving-Average Filtering:** Temperature (`DS18B20`, 10-bit asynchronous conversion) and air quality (`MQ` analog sensor) pass through a 10-sample moving average (20 s window), discarding invalid `-127 °C` disconnect readings.
* **Gravity-Compensated Impact Detection:** At 50 Hz, dynamic acceleration is computed as $a_{\text{dyn}} = \left| \sqrt{a_x^2 + a_y^2 + a_z^2} - g_0 \right| / g_0$. Impacts exceeding **1.8 g** trigger a 1.2 s OLED strobe inversion and an immediate out-of-band MQTT transmission.
* **Proportional PWM Ventilation Law:** Fan duty cycle $D(T, G)$ scales linearly from 30% to 90% between **21 °C and 28 °C**, locking at **100%** if $T \ge 28\text{ °C}$ or gas readings reach $G \ge 2700$ ADC counts, and shutting off ($0\%$) below $21\text{ °C}$.
* **Four-State Finite-State Machine (FSM):** Priority-evaluated states (`CRITICO` -> Red LED + cloud alert, `MODERADO` -> Orange LED, `BIOMETRIA` -> Blue LED during active pulse contact > 35,000 IR counts, `NORMAL` -> Green LED), dimmed to 30% duty cycle in low ambient light via LDR detection.

---

## 4. Hardware Architecture & Interface Specification

| Component | Function | Interface / ESP32-S3 GPIO |
| :--- | :--- | :--- |
| **ESP32-S3 (N16R8)** | Dual-core Xtensa LX7 Edge MCU (2.4 GHz Wi-Fi) | System Core |
| **MAX30105** | Photoplethysmography (PPG) heart-rate sensor (35–220 BPM valid range) | Shared I2C Bus (`SDA: GPIO 5`, `SCL: GPIO 4`) |
| **MPU6050** | ±4 g 3-axis accelerometer for structural vibration | Shared I2C Bus (`SDA: GPIO 5`, `SCL: GPIO 4`) |
| **SSD1306 OLED (128×64)** | Local graphical HUD & boot emblem (`0x3C`) | Shared I2C Bus (`SDA: GPIO 5`, `SCL: GPIO 4`) |
| **DS18B20** | Asynchronous 10-bit digital temperature probe | 1-Wire (`GPIO 6`) |
| **MQ Gas Sensor** | Air quality analog concentration sensor (12-bit) | ADC1 (`GPIO 1`) |
| **LDR Module** | Digital ambient light comparator (LED auto-dimming) | Digital Input (`GPIO 7`) |
| **Cooling Fan + MOSFET** | Active thermal regulation & air renewal | 8-bit PWM (`GPIO 18`) |
| **RGB LED (Common Cathode)** | Hardware FSM visual state indicator | PWM (`R: GPIO 15`, `G: GPIO 16`, `B: GPIO 17`) |

---

## 5. Telemetry Protocol, Offline Resilience & Documentation

### 5.1 IEEE Technical Report

Full mathematical derivations, firmware algorithms, validation methodology, and computer-architecture mapping are documented in the formal technical report included in this repository:

* **Technical Paper (PDF):** [Lk_5843_Neptune_SmartGym_Edge_Gateway_IEEE.pdf](./Lk_5843_Neptune_SmartGym_Edge_Gateway_IEEE.pdf)

### 5.2 Loss-Tolerant MQTT & Circular FIFO Buffer

Telemetry frames (~240 bytes in JSON format) are synchronized via NTP (`pool.ntp.org`, UTC-5) and published over **MQTT 3.1.1** (`broker.hivemq.com:1883`) every 10 s or immediately upon impact events. During network outages, a **100-record circular FIFO buffer** in SRAM (~3 KB) retains approximately **16 minutes** of telemetry, overwriting the oldest sample on overflow and replaying all stored records in strict chronological order upon reconnection.

```json
{
  "device_id": "MR_HYDE_NODE_01",
  "timestamp": 1794243360,
  "metrics": {
    "temp_c": 24.2,
    "gas_adc": 1850,
    "light_detected": true,
    "vibration_peak_g": 0.35,
    "bpm_user": 78
  },
  "actuators": {
    "fan_duty_pct": 55,
    "led_status": "MODERADO"
  }
}
```

---

## 6. Embedded C++ Software Architecture & Hardening Roadmap

The firmware (`mr_hyde_edge_node.ino`) is structured under the C++17 nested namespace `mr_hyde::edge`, following strict embedded engineering constraints:

* **Single Encapsulated Controller (`EdgeNodeController`):** Owns all peripheral drivers, filter windows, cooperative timers, and the offline FIFO queue, reducing Arduino entry points to `begin()` and `tick()`.
* **Flash-Resident Configuration:** `NetworkConfig`, `HardwarePins`, and `EdgeThresholds` reside in flash memory via `static constexpr`, leaving SRAM free for runtime data.
* **Static Memory Allocation:** Fixed-size member arrays eliminate dynamic heap allocation during queue and filter operations.

### Engineering Hardening Roadmap (Next Revisions)

* **Impact-Alert Cooldown:** Minimum refractory interval between > 1.8 g transmissions to prevent alert bursts from saturating the FIFO buffer.
* **Transport Layer Security (TLS):** Migration from unencrypted port 1883 to authenticated MQTTS over port 8883 with QoS 1 delivery guarantees.
* **Control Hysteresis:** Guard bands around the 21 °C and 28 °C thermal thresholds to eliminate PWM/LED chattering.
* **Zero-Heap Serialization:** Replacing `String`-based JSON assembly with fixed `char` buffers and `snprintf`, alongside externalized credential provisioning.

---

## 7. Getting Started

1. **Board package:** Install the Arduino-ESP32 core and select an ESP32-S3 board.
2. **Libraries:** Install `PubSubClient`, `Adafruit GFX`, `Adafruit SSD1306`, `OneWire`, `DallasTemperature`, `Adafruit MPU6050`, `Adafruit Unified Sensor` and a `MAX30105` driver (SparkFun MAX3010x).
3. **Configuration:** Set your own Wi-Fi credentials and MQTT broker in the `NetworkConfig` structure. **Never commit real credentials to version control.**
4. **Wiring:** Connect the components following the pin map in Section 4, then compile and upload `mr_hyde_edge_node.ino`.
5. **Verification:** Subscribe to the configured MQTT topic with any client to inspect the live telemetry frames.

---

## 8. Ethics, Licensing & Safety Scope

### 8.1 Software License

This project is distributed under the **MIT License**. Review the [LICENSE](LICENSE) file in the root directory for full terms and conditions.

### 8.2 Safety Scope & Professional Disclaimer

Project Lk_5843 // Neptune (MR_HYDE_NODE_01) is an experimental research and engineering prototype. Its photoplethysmographic heart-rate readings provide reference telemetry only and do not constitute or replace a certified medical device. Its vibration and thermal measurements do not replace structural engineering assessments or certified HVAC system design. Because the default demonstration broker operates without transport encryption, no personally identifiable or clinical patient data should be transmitted over unencrypted builds. Neither the author nor project contributors assume liability for operational, structural, or health decisions derived from the use of this system.

---

## 9. Authorship & Academic Lineage

* **Engineering & Firmware Architecture:** Conceived, modeled, and developed in C++ by Diego Andrés Delgado Andrade (Mr_Hyde Industries) within the Software Engineering curriculum at Universidad Católica de Cuenca (UCACUE), specializing in Embedded Systems, Computer Architecture, and High-Performance Software.
* **Research Identifier:** [ORCID 0009-0008-9525-202X](https://orcid.org/0009-0008-9525-202X)
* **Related Systems in the Lk_ Series:**
  * **Lk_7392 // Mercury Platform:** Earth observation analytics and territorial viability engine (C++20 / Python / PostGIS).
  * **Lk_4282 // Bio_InGen Platform:** Clinical informatics, prescription auditability, and medical sample traceability system (Java 25 LTS / PostgreSQL).
