# UAV-Based Wind Field Measurement and CFD Validation System

## Overview
This repository contains the hardware integration, firmware, software, and 3D route planning algorithms for an IoT-integrated Unmanned Aerial Vehicle (UAV) system. The primary objective of this project is to acquire real-time, vertical-profile wind field data to validate Computational Fluid Dynamics (CFD) microclimate models in complex urban campus environments.

## System Architecture
The system architecture encompasses an end-to-end data pipeline from physical sensing to web-based visualization:

* **Sensing Layer:** A custom quadrotor UAV equipped with an ultrasonic anemometer and a Pixhawk 4 Mini flight controller for stable data acquisition up to an altitude of 40 meters.
* **Communication Layer (Firmware):** An ESP-32S microcontroller acting as a Modbus RTU master. It acquires data via RS-485, parses 16-bit registers into 32-bit floating-point values, and transmits the data in JSON format over serial communication.
* **Processing & Presentation Layer (Software):** A Raspberry Pi 5 serves as the edge computing node. It receives JSON data via USB Serial, performs 60-second moving average logging, and hosts a Python (Flask) backend to serve real-time telemetry data to a Chart.js frontend dashboard.

### Hardware & System Integration
![UAV Hardware Setup](Images/uav_hardware_setup.jpg)
![UAV Flight Controller Integration](Images/pixhawk4_integration.jpg)

### IoT Pipeline & Real-Time Dashboard
![Backend Terminal Log](Images/backend_terminal_log.png)
![Web Dashboard](Images/web_dashboard.png)

## Technologies Used
* **Hardware:** Quadrotor UAV, Pixhawk 4 Mini, ESP-32S, Raspberry Pi 5, Ultrasonic Anemometer.
* **Firmware / Software:** C/C++ (Arduino IDE), Python (Flask, pySerial), Modbus RTU, RS-485, HTML/CSS/JavaScript (Chart.js).
* **Simulation & Validation:** OpenFOAM ($k-\omega$ SST turbulence model), ParaView, RMSE, Pearson Correlation.
* **Route Planning:** 3D Voxel Mapping, Numpy, STL Geometry Parsing.

## CFD Simulation vs. Flight Data
Computational Fluid Dynamics (CFD) simulations were conducted using OpenFOAM to model wind dynamics across the campus. The numerical models, optimized with a 237k-cell hexahedral mesh (maximum non-orthogonality < 65), were quantitatively validated against physical flight data under various wind conditions (3 to 8 m/s). Model accuracy and reliability were evaluated using Root Mean Square Error (RMSE) and Pearson correlation coefficients.

![CFD Simulation in ParaView](Images/cfd_simulation_paraview.jpg)

## Repository Structure
* `/Firmware_ESP32`: C++ source code for ESP-32S sensor data acquisition and Modbus RTU communication.
* `/Software_RaspberryPi`: Python backend server (`server.py`) and frontend visualization dashboard (`templates/index.html`).
* `/UAV_Route_Planning`: Python algorithms for converting campus `.stl` models into voxel grids and generating 3D automated flight routes.
* `/Document`: The complete project report (`UAV_Wind_Measurement_Final_Report.pdf`) detailing methodologies, hardware setup, and data analysis.
* `/Images`: Hardware photographs and visualization screenshots used in this documentation.
