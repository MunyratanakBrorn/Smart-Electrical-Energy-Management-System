<!-- ============================== -->
<!-- Smart Electrical Energy Management System (SEEMS) -->
<!-- ============================== -->

<h1 align="center">⚡ Smart Electrical Energy Management System (SEEMS)</h1>

<p align="center">
  <img src="System Overview/Programming Framework.png" alt="SEEMS System Diagram" width="600">
</p>

<p align="center">
  <a href="https://opensource.org/licenses/MIT"><img src="https://img.shields.io/badge/License-MIT-yellow.svg" alt="License: MIT"></a>
  <img src="https://img.shields.io/badge/Status-Complete-success" alt="Project Status">
  <img src="https://img.shields.io/badge/Tech-ESP32%20%7C%20STM32-blueviolet" alt="Technology Stack">
</p>

<p align="justify">
  A robust and scalable IoT-based system for <strong>monitoring, controlling</strong>, and <strong>optimizing electrical energy usage</strong> in real time. This project was developed as part of the <strong>Bachelor of Technology in Electronics Engineering</strong> at <strong>Preah Kossomak Polytechnic Institute</strong>.
</p>

<hr>

<!-- ============================== -->
<!-- 🌐 Overview -->
<!-- ============================== -->

<h2>🌐 Overview</h2>

<p align="justify">
  The <strong>Smart Electrical Energy Management System (SEEMS)</strong> is designed to address the increasing need for smart energy solutions. It provides:
  <ul>
    <li>Real-time monitoring of electrical parameters</li>
    <li>Local and remote control of appliances</li>
    <li>Distributed architecture with STM32 controller nodes and an ESP32 gateway</li>
    <li>Intuitive HMI for user interaction</li>
  </ul>
  Ideal for <strong>residential, commercial</strong>, or <strong>industrial</strong> applications, SEEMS is engineered with scalability, efficiency, and reliability in mind.
</p>

<hr>

<!-- ============================== -->
<!-- 🖼️ Project Visuals -->
<!-- ============================== -->

<h2>🖼️ Project Visuals</h2>

<h3>🔧 HMI Screen Snapshots</h3>

<table>
  <tr>
    <th align="center">Main Dashboard</th>
    <th align="center">Real-time Graph</th>
  </tr>
  <tr>
    <td><img src="System Overview/main.png" width="400"/></td>
    <td><img src="System Overview/Screenshot 2025-07-16 092032.png" width="400"/></td>
  </tr>
</table>

<p><em>Tip: Replace these with real screenshots. Store in <code>docs/images/</code> for better organization.</em></p>

<!-- ============================== -->
<!-- 🧠 System Architecture -->
<!-- ============================== -->

<h2>🧠 System Architecture</h2>

<p align="justify">
  The <strong>Smart Electrical Energy Management System (SEEMS)</strong> uses a distributed architecture, where a central gateway (ESP32) communicates with one or more controller nodes (STM32F405). This modular design enables real-time monitoring, reliable control, and flexible system expansion across residential, commercial, or industrial setups.
</p>

<p align="center">
  <a href="System Overview/Hardware System diagram.jpg" target="_blank">
    <img 
      src="System Overview/Hardware System diagram.jpg" 
      alt="SEEMS Distributed System Architecture Diagram" 
      width="700"
      style="border: 1px solid #ddd; border-radius: 8px; box-shadow: 0 4px 10px rgba(0, 0, 0, 0.1);"
    />
  </a>
</p>

<p align="center">
  <em>Click the image to view it in full resolution.</em>
</p>

<ul>
  <li><strong>STM32 Controller Node:</strong> Measures voltage/current, processes data, controls relays.</li>
  <li><strong>ESP32 Gateway:</strong> Aggregates data, manages Wi-Fi/cloud connectivity, sends control commands.</li>
  <li><strong>HMI Interface:</strong> Displays real-time feedback, allows local interaction.</li>
</ul>

<hr>

<!-- ============================== -->
<!-- ⚙️ How It Works -->
<!-- ============================== -->

<h2>⚙️ How It Works</h2>

<ol>
  <li><strong>Measurement:</strong> STM32F405 node measures voltage and current using BL0910 sensors.</li>
  <li><strong>Processing & Display:</strong> STM32 calculates power/energy and sends data to the HMI screen.</li>
  <li><strong>Communication:</strong> Data is sent to ESP32 via UART.</li>
  <li><strong>Wi-Fi Gateway:</strong> ESP32 handles Wi-Fi and aggregates system data for remote access.</li>
  <li><strong>Actuation:</strong> Relays are triggered based on local/remote user input to control loads.</li>
</ol>

<hr>

<!-- ============================== -->
<!-- 🛠️ Technical Specifications -->
<!-- ============================== -->

<h2>🛠️ Technical Specifications</h2>

<h3>🔩 Hardware Components</h3>

<table>
  <tr>
    <th>Component</th>
    <th>Description</th>
    <th>Role</th>
  </tr>
  <tr>
    <td><strong>ESP32-WROOM-32U</strong></td>
    <td>Wi-Fi-enabled MCU</td>
    <td>Data aggregation, cloud gateway</td>
  </tr>
  <tr>
    <td><strong>STM32F405RG</strong></td>
    <td>High-performance MCU</td>
    <td>Sensing, processing, load control</td>
  </tr>
  <tr>
    <td><strong>HMI Display</strong></td>
    <td>Designed using Squareline Studio</td>
    <td>User interface</td>
  </tr>
  <tr>
    <td><strong>BL0910</strong></td>
    <td>Energy metering IC</td>
    <td>AC voltage/current sensing</td>
  </tr>
  <tr>
    <td><strong>12V Songle Relays</strong></td>
    <td>Electromechanical switch</td>
    <td>Load control (on/off)</td>
  </tr>
</table>

<h3>💻 Software Stack</h3>

<ul>
  <li><strong>Languages:</strong> C / C++</li>
  <li><strong>Platforms:</strong>
    <ul>
      <li><strong>ESP32:</strong> PlatformIO + Arduino Framework</li>
      <li><strong>STM32:</strong> STM32CubeIDE</li>
      <li><strong>HMI:</strong> Squareline Studio</li>
    </ul>
  </li>
  <li><strong>Protocols:</strong> UART (STM32 ↔ ESP32), Wi-Fi (ESP32 ↔ Network)</li>
</ul>

<hr>

<!-- ============================== -->
<!-- 🚀 Getting Started -->
<!-- ============================== -->

<h2>🚀 Getting Started</h2>

<h3>🧰 Prerequisites</h3>
<ul>
  <li><a href="https://www.st.com/en/development-tools/stm32cubeide.html">STM32CubeIDE</a></li>
  <li><a href="https://platformio.org/">PlatformIO for VS Code</a></li>
  <li>Hardware components as listed above</li>
</ul>

<h3>🧱 Hardware Setup</h3>
<ul>
  <li>Assemble components according to the provided schematics in <code>Hardware/</code></li>
  <li>Ensure secure connections between STM32, ESP32, sensors, and HMI</li>
</ul>

<h3>🔌 Firmware Upload</h3>
<ol>
  <li><strong>STM32 Node:</strong> Open <code>Firmware/STM32_F405_Controller Node/</code> in STM32CubeIDE and flash firmware.</li>
  <li><strong>ESP32 Gateway:</strong> Open <code>Firmware/ESP32_Gateway_SEEMs/</code> in PlatformIO, set Wi-Fi credentials, and upload code.</li>
  <li><strong>HMI:</strong> Open <code>Firmware/HMI Screen display/</code> in Squareline Studio and upload compiled <code>.tft</code> file via SD card.</li>
</ol>

<h3>🔄 Running the System</h3>
<ul>
  <li>Power on all devices</li>
  <li>HMI should load the dashboard</li>
  <li>ESP32 connects to Wi-Fi and communicates with STM32</li>
  <li>System becomes operational for real-time monitoring and control</li>
</ul>

<hr>

<!-- ============================== -->
<!-- 📁 Project Structure -->
<!-- ============================== -->

<h2>📁 Repository Structure</h2>

<pre>
📦 SEEMS
├── 📂 Firmware/
│   ├── 📂 ESP32_Gateway_SEEMs/
│   ├── 📂 STM32_F405_Controller Node/
│   └── 📂 HMI Screen display/
├── 📂 Hardware/
├── 📂 System Overview/
├── 📂 Thesis Report and presentation/
└── 📄 README.md
</pre>

<hr>

<!-- ============================== -->
<!-- 👥 Authors & Acknowledgments -->
<!-- ============================== -->

<h2>👥 Authors & Acknowledgments</h2>

<p>This project was developed by:</p>
<ul>
  <li><strong>Mr. Brorn Munyratanak</strong></li>
  <li><strong>Mr. Noch Kakada</strong></li>
</ul>

<p>Final Year Thesis – <strong>Electronics Engineering</strong><br>
<strong>Preah Kossomak Polytechnic Institute</strong> (Class of 2024)</p>

<p><em>Special thanks to our advisors and faculty members for their guidance and support throughout the project.</em></p>

<hr>

<!-- ============================== -->
<!-- 📜 License -->
<!-- ============================== -->

<h2>📜 License</h2>

<p>This project is licensed under the <a href="https://opensource.org/licenses/MIT">MIT License</a>.</p>
