# Smart Electrical Energy Management System (SEEMS)

<p align="center">
  <img src="System Overview/Programming Framework.png" alt="Project Logo">
</p>

<p align="center">
  <a href="https://opensource.org/licenses/MIT"><img src="https://img.shields.io/badge/License-MIT-yellow.svg" alt="License: MIT"></a>
  <a href="#"><img src="https://img.shields.io/badge/Status-Complete-success" alt="Project Status"></a>
  <a href="#"><img src="https://img.shields.io/badge/Tech-ESP32%20%7C%20STM32-blueviolet" alt="Technology"></a>
</p>

A comprehensive IoT solution for monitoring, controlling, and optimizing electrical energy usage. This project was developed to fulfill the requirements for the Bachelor of Technology in Electronics Engineering at Preah Kossomak Polytechnic Institute.

## 🌟 Introduction

The **Smart Electrical Energy Management System (SEEMS)** addresses the growing need for intelligent energy consumption. By providing real-time data and remote control capabilities, this system empowers users to reduce energy waste, lower electricity costs, and contribute to a more stable power grid. The project is built on a distributed architecture, ensuring scalability and robustness for residential, commercial, or industrial applications.

---

## 🎨 Project Visuals

### 🖥️ HMI Screen Display
A local Human-Machine Interface (HMI) provides an intuitive dashboard for at-a-glance monitoring and control.

> **Note:** Replace these placeholder images with screenshots of your actual HMI. Upload your images to a folder in this repository (e.g., create a `docs/images` folder) and update the links.

| Main Dashboard | Real-time Data Graph |
| :---: | :---: |
| <img src="System Overview/main.png" alt="System Overview/main.png" width="400"/> | <img src="System Overview/dasborad.jpg" alt="HMI Graph" width="400"/> |

---

### 💡 System Architecture
The system employs a distributed architecture where a central gateway communicates with one or more controller nodes. This design is both resilient and scalable. The complete, high-resolution diagrams are available in the `System Overview` folder.

<p align="center">
  <a href="System Overview/Hardware System diagram.jpg">
    <img src="System Overview/Hardware System diagram.jpg" alt="System Architecture Diagram" width="700"/>
  </a>
</p>

---

## ⚙️ How It Works

The system operates in a clear, sequential flow:

1.  **Sensing:** The **STM32F405 Controller Node** continuously measures electrical parameters (voltage, current) using dedicated sensors.
2.  **Local Processing & Display:** The STM32 processes the raw data to calculate power and energy consumption. This information is immediately sent to the **HMI Screen** for local, real-time visualization.
3.  **Gateway Communication:** The STM32 simultaneously transmits the data to the **ESP32 Gateway** via a reliable serial protocol (e.g., UART).
4.  **Central Aggregation & Control:** The **ESP32 Gateway** aggregates data from all nodes. It handles Wi-Fi connectivity, allowing for potential cloud integration, data logging, or remote access. It also relays control commands back to the nodes.
5.  **Actuation:** Based on user input from the HMI or remote commands, the STM32 Controller Node controls power relays to switch electrical loads on or off.

---

## 🛠️ Technical Specifications

### Hardware
| Component                   | Model/Type                    | Role in Project                                             |
| --------------------------- | ----------------------------- | ----------------------------------------------------------- |
| 🎛️ **Central Gateway**      | ESP32-WROOM-32U               | Wi-Fi communication, data aggregation, system coordinator   |
| 🔬 **Controller Node**        | STM32F405RG                   | High-speed data acquisition, processing, and relay control  |
| 🖥️ **HMI Display**            | Squareline studio     | User interface for data visualization and direct control    |
| ⚡️ **Sensors**                | BL0910      | Measures AC current and voltage                             |
| 🔌 **Actuators**              |12V Songle Relays        | Switches high-voltage electrical loads                      |

### Software & Firmware
*   **Programming Languages:** C/C++
*   **Frameworks & IDEs:**
    *   **ESP32:** PlatformIO with Arduino Framework
    *   **STM32:** STM32CubeIDE
    *   **HMI:** (Squareline studio)
*   **Communication Protocols:** UART (Node-to-Gateway), Wi-Fi (Gateway-to-Network)

---

## 🚀 Getting Started

To set up and run this project, follow these steps.

### Prerequisites
*   Install [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html).
*   Install [PlatformIO IDE for VS Code](https://platformio.org/).
*   Have all the necessary hardware components listed above.

### 1. Hardware Assembly
*   Assemble the electronic components according to the schematics and PCB layouts provided in the `Hardware/` directory.
*   Ensure all connections between the STM32, ESP32, HMI, and sensor modules are secure.

### 2. Firmware Flashing
1.  **STM32 Controller Node:** Open the project from `Firmware/STM32_F405_Controller Node/` in STM32CubeIDE. Compile and flash the firmware to the STM32F405 board.
2.  **ESP32 Gateway:** Open the project from `Firmware/ESP32_Gateway_SEEMs/` in PlatformIO. Configure your Wi-Fi credentials in the source code, then compile and upload the firmware.
3.  **HMI Display:** Open the project from `Firmware/HMI Screen display/` using the appropriate HMI editor. Compile the project and upload the `.tft` file to the display (usually via an SD card).

### 3. System Operation
*   Power on all components.
*   The HMI should display the main dashboard.
*   The ESP32 will connect to your Wi-Fi network. You can monitor its serial output for status updates.
*   The system is now operational. Real-time data will be visible on the HMI.

---

## 📂 Repository Structure
├── 📂 Firmware/
│ ├── 📂 ESP32_Gateway_SEEMs/
│ ├── 📂 HMI Screen display/
│ └── 📂 STM32_F405_Controller Node/
├── 📂 Hardware/
├── 📂 System Overview/
├── 📂 Thesis Report and presentation/
└── 📄 README.md

## 🎓 Authorship & Acknowledgments

This project was developed by **Mr. Brorn Munyratanak** & **Mr. Noch Kakada** as a final thesis for the Bachelor of Technology in **Electronics Engineering** at **Preah Kossomak Polytechnic Institute (Class of 2024)**.

We extend our sincere gratitude to our advisors and the faculty for their invaluable guidance and support.
