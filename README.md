# 🚗 The Vehicle Blackbox

![License](https://img.shields.io/badge/license-CDAC-blue)
![Version](https://img.shields.io/badge/version-1.0.0-orange)

📹 **Working Video:** [GitHub Demo](https://github.com/unidipu24-es/The-Vehicle-Blackbox/blob/main/The_Vehicle_Blackbox.mp4)

---

## 📌 Description

The **Embedded Vehicle Blackbox for Telematics and Parameter Logging** is an advanced, real-time vehicle tracking and monitoring system designed to enhance road safety, streamline fleet management, and support comprehensive accident analysis.

This system:
- Collects **critical operational and environmental parameters** (speed, acceleration, braking, crash events, temperature, humidity, location, etc.)
- Uses **LoRaWAN** for long-range Vehicle-to-Vehicle (V2V) communication and alerts.
- Integrates **CAN bus** for direct in-vehicle diagnostics.
- Pushes telemetry to the **ThingsBoard** IoT platform for live monitoring, historical analysis, and automated alerts.

The project implements a **distributed sensor network** with two hardware nodes:
1. **Node 2 (Vehicle Unit)** — STM32 + ESP32 for sensor interfacing, data aggregation, cloud publishing, and LoRa V2V.
2. **Node 1 (Proximity Monitor)** — ESP32 for LoRa reception/transmission of critical alerts.

---

## 📑 Table of Contents

1. [Abstract](#-abstract)
2. [Features](#-features)
3. [System Architecture](#-system-architecture)
4. [System Flow](#-system-flow)
5. [Key Achievements](#-key-achievements)
6. [Contributing](#-contributing)
7. [License](#-license)
8. [Links & Resources](#-links--resources)
9. [Benefits of the System](#-benefits-of-the-system)


---

## 📝 Abstract

This project describes a **real-time vehicle tracking and monitoring system** designed to provide operational insights and geolocation data.  
A **distributed sensor network** collects various parameters, transmits alerts via LoRa, and uploads telemetry to **ThingsBoard** for visualization and alarms.

Key objectives:
- **Accident Analysis** — Capture acceleration/braking profiles, crash detection, and incident logs.
- **Telematics & Diagnostics** — Monitor idling time, environmental conditions, and vehicle health.
- **V2V Communication** — Enable LoRa-based proximity and alert messages.
- **Cloud Integration** — Use MQTT to send telemetry to ThingsBoard for live and historical monitoring.

---

## ✨ Features

- 📊 **Data Logging** – Store vehicle telemetry like Temperature, humidity, speed, braking, acceleration for historical analysis.
- 🌐 **Real-time Monitoring** – Live tracking of vehicle location and parameters.
- 📡 **V2V Communication** – LoRa SX1278 enables peer-to-peer crash and alert messaging.
- 🛠 **Diagnostics** – Detect anomalies for predictive maintenance.
- 🚦 **CAN Bus Data Acquisition** — Capture internal vehicle data like speed and RPM.
- 🛡 **Crash Detection** – Alerts sent automatically to other vehicles and cloud in emergencies.
- ☁️ **Cloud Dashboard** – Secure IoT communication and analytics + Live visualization on ThingsBoard.
- 🔋 **Low Power Design** – Optimized for continuous operation in vehicles.

---

## 🖥 System Architecture

**Node 2 (Vehicle Unit)**  
- **STM32F407VGT6** — Interfaces with sensors (DHT11, IR, Sound) and formats data.  
- **ESP32** — Internet connectivity, Google Geolocation API integration, MQTT publishing to ThingsBoard, LoRa V2V communication.
- **Sensors** — Speed (H2010), Sound (LM393), Temperature & Humidity (DHT11), IR obstacle sensor, crash detection
- **LoRa SX1278** — for V2V alerts
- **CAN Interface** — MCP2515 + MCP2551 for vehicle diagnostics.
- **Buzzer** — for local alerts

  
**Node 1 (Proximity Monitor)**  
- **ESP32** — Dedicated to LoRa reception/transmission of alerts.

**Cloud Platform: ThingsBoard**  
- MQTT broker, time-series database, rule engine, and customizable dashboard.

📌 *Architecture Diagram:*
<img width="1920" height="939" alt="Block Diagram_CDAC Project" src="https://github.com/user-attachments/assets/d3200724-03ed-42c1-bea5-18a29615332b" />

## 🔄 System Flow

### **Node 1 Flow**
- Receives crash alert via **LoRa**.
- Sends acknowledgment via LoRa if within range.

### **Node 2 Flow**
1. **Arduino Uno** → Reads sensors (DHT11, IR, H2010, Sound).
2. Sends data via **MCP2515** (SPI) → **MCP2551** → **CAN bus**.
3. **STM32** receives CAN data → outputs to FTDI (debug) + forwards to ESP32 via **UART**.
4. **ESP32** → Uploads data to **ThingsBoard** (MQTT/HTTP).
5. LoRa transmission to Node 1 on **critical events**.

---

## 🏆 Key Achievements
- **Multi-MCU Integration** — Coordinated Arduino Uno, STM32F407VGT6, and ESP32 using UART/SPI.
- **Protocol Implementation** — CAN bus, LoRa, MQTT, HTTP for end-to-end telemetry.
- **Real-Time Data Acquisition** — Direct access to native vehicle parameters + external sensor data.
- **Robust LoRa Alerts** — Reliable, independent V2V communication.
- **Full IoT Stack** — Hardware to cloud integration with ThingsBoard.
- **Performance Validation** — Boot time < 3s, alert response < 1.5s.
- **Scalable Design** — Modular architecture for easy sensor expansion.

---

## 🤝 Contributing
We welcome contributions!  
Please fork, branch, and submit PRs.  
For issues, use the **[GitHub Issues](../../issues)** section.

---

## 📜 License
This project is licensed under the **CDAC License**

---

## 🔗 Links & Resources
- [ThingsBoard](https://thingsboard.io/)
- [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html)
- [Arduino IDE](https://www.arduino.cc/en/software)
- [LoRa SX1278 Datasheet](https://www.semtech.com/lora)

---

## 💡 **Benefits of the System**  

- **Enhanced Public Safety** — Immediate detection and reporting of crashes can help emergency services respond faster, potentially saving lives.  
- **Accident Forensics** — Provides authorities with accurate incident data, reducing disputes and improving road accident investigations.  
- **Reduced Traffic Hazards** — Real-time vehicle-to-vehicle (V2V) alerts can help prevent secondary collisions near accident sites.  
- **Better Urban Planning** — Aggregated traffic and vehicle behavior data can assist city planners in identifying accident-prone zones and improving road infrastructure.  
- **Environmental Benefits** — Monitors idling and fuel consumption trends, enabling fleet operators to adopt eco-friendly driving practices.  
- **Cost Savings for Operators** — Reduces insurance claims processing time and enables proactive vehicle maintenance, lowering long-term expenses.  
- **Support for Autonomous Systems** — Serves as a reliable data source for future integration with autonomous vehicle safety networks.  
- **Rural & Remote Area Safety** — LoRa-based alerts ensure communication in areas with no cellular coverage, improving safety in underserved regions.  

