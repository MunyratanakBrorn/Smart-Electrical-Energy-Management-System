<div align="center">

# ⚡ Smart Electrical Energy Management System
### SEEMS

<p>
  <em>A distributed IoT platform for real-time monitoring, control, and optimization of electrical energy.</em>
</p>

<img src="System Overview/Programming Framework.png" alt="SEEMS System Diagram" width="620">

<br><br>

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
![Status](https://img.shields.io/badge/Status-Complete-success)
![Stack](https://img.shields.io/badge/MCU-ESP32%20%7C%20STM32F405-blueviolet)
![Build](https://img.shields.io/badge/Firmware-C%20%2F%20C%2B%2B-00599C)
![Cloud](https://img.shields.io/badge/Cloud-ThingsBoard-FF6F00)
![Year](https://img.shields.io/badge/Thesis-2024-informational)

</div>

> 💡 **This is a public template.** Use it to bootstrap your own energy-monitoring node — click **"Use this template"** above.

---

## 📌 Table of Contents

- [About the Project](#-about-the-project)
- [Key Features](#-key-features)
- [System Architecture](#-system-architecture)
- [End-to-End Workflow](#-end-to-end-workflow)
- [Project Visuals](#-project-visuals)
- [Tech Stack](#-tech-stack)
- [Hardware Components](#-hardware-components)
- [Communication Protocols](#-communication-protocols)
- [Getting Started](#-getting-started)
- [Repository Structure](#-repository-structure)
- [Authors](#-authors)
- [License](#-license)

---

## 🌐 About the Project

The **Smart Electrical Energy Management System (SEEMS)** is a scalable, IoT-based solution that monitors, controls, and optimizes electrical energy consumption in real time. Built on a **distributed architecture** — STM32 controller nodes paired with an ESP32 gateway and a **ThingsBoard cloud platform** — the system is suitable for **residential, commercial, and industrial** deployments.

Developed as a **Final Year Thesis** for the **Bachelor of Technology in Electronics Engineering** at **Preah Kossomak Polytechnic Institute** (Class of 2024).

---

## ✨ Key Features

| | Feature | Description |
|---|---------|-------------|
| 📊 | **Real-Time Monitoring** | Live voltage, current, power, and energy readings |
| 🎛️ | **Local & Remote Control** | Switch loads via HMI touchscreen or remotely over the cloud |
| ☁️ | **Cloud Dashboard (ThingsBoard)** | Remote telemetry, visualization, and control via MQTT |
| 🔆 | **Dimmer Control (TRIAC)** | Phase-angle dimming for adjustable loads |
| 🧩 | **Distributed & Modular** | Add controller nodes without re-architecting the system |
| 🗄️ | **Onboard Data Logging** | Local logging over I2C/SPI for resilience |
| 🖥️ | **Intuitive HMI** | Touch dashboard built with SquareLine Studio |
| 🔌 | **Reliable Actuation** | Relay-based load switching with safe-state handling |

---

## 🧠 System Architecture

SEEMS follows a **gateway–node** topology. A central **ESP32 gateway** coordinates one or more **STM32F405 controller nodes** for local sensing and actuation, then publishes telemetry to the **ThingsBoard IoT platform** over **MQTT** for remote monitoring and control.

<div align="center">
  <a href="System Overview/Hardware System diagram.jpg" target="_blank">
    <img src="System Overview/Hardware System diagram.jpg" alt="SEEMS Architecture Diagram" width="720" style="border-radius:8px;">
  </a>
  <br>
  <em>Click to view full resolution.</em>
</div>

| Layer | Component | Responsibility |
|-------|-----------|----------------|
| **Edge** | STM32F405 Node | Measure V/I, compute power & energy, drive relays & TRIAC dimmer |
| **Gateway** | ESP32-WROOM-32U | Aggregate data, manage Wi-Fi/Ethernet, publish MQTT to cloud |
| **Cloud** | ThingsBoard | Remote dashboards, telemetry storage, control commands |
| **Interface** | HMI Display | Real-time feedback and local user control |
| **Logging** | Data Logger (I2C/SPI) | Persist measurements locally |

---

## 🔄 End-to-End Workflow

```mermaid
flowchart LR
    A[AC Load] -->|sensing| B[Energy Meter<br/>BL0910]
    B -->|UART| C[STM32F405<br/>Controller Node]
    C -->|power / energy| D[HMI Display]
    C -->|I2C / SPI| H[Data Logger]
    C -->|UART| E[ESP32 Gateway]
    E -->|MQTT over Wi-Fi / Ethernet| F[(ThingsBoard<br/>IoT Platform)]
    F -->|remote command| E
    D -->|local command| C
    E -->|control| C
    C -->|Relay switch| G[Load ON/OFF]
    C -->|Timer PWM| I[TRIAC Dimmer]
```

**Sequence**

1. **Measurement** — The BL0910 IC senses AC voltage and current.
2. **Processing** — The STM32F405 computes power and energy metrics.
3. **Display & Logging** — Results are pushed to the HMI and logged locally via I2C/SPI.
4. **Communication** — Data travels to the ESP32 over UART.
5. **Gateway & Cloud** — The ESP32 publishes telemetry to ThingsBoard via MQTT (over Wi-Fi/Ethernet).
6. **Actuation** — Relays switch loads and the TRIAC dimmer adjusts brightness, based on local or remote input.

---

## 🎬 Live Demo

<div align="center">
  <img src="seems-animation.svg" alt="SEEMS animated demo: relay ON/OFF switching and live energy metering" width="760">
  <br>
  <em>Animated overview — relay <strong>ON/OFF</strong> load switching and live <strong>energy metering</strong> (voltage, current, power, energy).</em>
</div>

> ℹ️ The animation is an inline SVG (`seems-animation.svg`) — it plays automatically on GitHub with no external dependencies.

---

## 🖼️ Project Visuals

<table>
  <tr>
    <th align="center">Main Dashboard</th>
    <th align="center">Real-Time Graph</th>
  </tr>
  <tr>
    <td><img src="System Overview/main.png" width="400"/></td>
    <td><img src="System Overview/Screenshot 2025-07-16 092032.png" width="400"/></td>
  </tr>
</table>

---

## 🧰 Tech Stack

**Languages:** C · C++

| Target | Toolchain / Framework |
|--------|------------------------|
| ESP32 Gateway | PlatformIO + Arduino Framework |
| STM32 Node | STM32CubeIDE (HAL) |
| HMI | SquareLine Studio |
| Cloud Platform | ThingsBoard (MQTT) |

---

## 🔩 Hardware Components

| Component | Description | Role |
|-----------|-------------|------|
| **ESP32-WROOM-32U** | Wi-Fi-enabled MCU | Data aggregation, MQTT cloud gateway |
| **STM32F405RG** | High-performance Cortex-M4 MCU | Sensing, processing, load & dimmer control |
| **BL0910** | Energy metering IC | AC voltage/current sensing |
| **TRIAC Dimmer** | Phase-angle control circuit | Adjustable lighting/load dimming |
| **HMI Display** | SquareLine Studio UI | User interface |
| **12V Songle Relays** | Electromechanical switch | Load on/off control |
| **Data Logger** | I2C/SPI storage | Local measurement logging |

---

## 🔗 Communication Protocols

| Protocol | Link | Purpose |
|----------|------|---------|
| **UART** | STM32 ↔ ESP32 · STM32 ↔ Energy Meter | Inter-MCU & sensor data exchange |
| **MQTT** | ESP32 ↔ ThingsBoard | Cloud telemetry & remote control |
| **Wi-Fi / Ethernet** | ESP32 ↔ Network | Internet connectivity |
| **I2C / SPI** | STM32 ↔ Data Logger | Local data logging |
| **Timer PWM** | STM32 ↔ TRIAC | Phase-angle dimming control |

---

## 🚀 Getting Started

### Prerequisites

- [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html)
- [PlatformIO for VS Code](https://platformio.org/)
- A [ThingsBoard](https://thingsboard.io/) account or self-hosted instance
- Hardware components listed above

### Hardware Setup

1. Assemble components per the schematics in `Hardware/`.
2. Verify connections between the STM32, ESP32, BL0910 sensors, data logger, TRIAC dimmer, and HMI.

### Firmware Upload

| Target | Steps |
|--------|-------|
| **STM32 Node** | Open `Firmware/STM32_F405_Controller Node/` in STM32CubeIDE → build → flash |
| **ESP32 Gateway** | Open `Firmware/ESP32_Gateway_SEEMs/` in PlatformIO → set Wi-Fi + ThingsBoard MQTT credentials → upload |
| **HMI** | Open `Firmware/HMI Screen display/` in SquareLine Studio → export → load `.tft` via SD card |

### Cloud Setup (ThingsBoard)

1. Create a new **Device** in ThingsBoard and copy its access token.
2. Set the token and broker host in the ESP32 firmware configuration.
3. Import or build a dashboard to visualize incoming telemetry.

### Run

1. Power on all devices.
2. The HMI loads the dashboard.
3. The ESP32 connects to the network and publishes telemetry to ThingsBoard.
4. The system is live for real-time local **and** remote monitoring and control.

---

## 📁 Repository Structure

```
📦 SEEMS
├── 📂 Firmware/
│   ├── 📂 ESP32_Gateway_SEEMs/
│   ├── 📂 STM32_F405_Controller Node/
│   └── 📂 HMI Screen display/
├── 📂 Hardware/
├── 📂 System Overview/
├── 📂 Thesis Report and presentation/
└── 📄 README.md
```

---

## 👥 Authors

| Name | Role |
|------|------|
| **Brorn Munyratanak** | Co-developer |
| **Noch Kakada** | Co-developer |

Final Year Thesis · **Electronics Engineering** · **Preah Kossomak Polytechnic Institute** (Class of 2024)

> Special thanks to our advisors and faculty for their guidance throughout the project.

---

## 📜 License

Licensed under the [MIT License](https://opensource.org/licenses/MIT).

<div align="center">
  <sub>Built with ⚡ for a smarter, more efficient energy future.</sub>
</div>
