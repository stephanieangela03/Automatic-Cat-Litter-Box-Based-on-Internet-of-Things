# Automated Cat Litter Box Based on Internet of Things (IoT)

This repository contains the source code for the Automated Cat Litter Box System developed as part of an undergraduate thesis in the Computer Science program at BINUS University.

## Overview

The Automated Cat Litter Box is an IoT-based system designed to automate the cleaning and litter refill process. The system integrates sensors, actuators, and cloud monitoring to reduce manual effort and improve litter box hygiene.

This repository focuses on the firmware implementation for the ESP32 microcontroller.

## System Features

- Automatic cleaning after cat usage detection
- Automatic litter refill when level is low
- Real-time monitoring via Arduino IoT Cloud
- Manual control through cloud dashboard
- Safety mechanism to stop cleaning when cat is detected

## Technologies Used

- ESP32
- PIR Sensor
- Ultrasonic Sensor (HC-SR04)
- Stepper Motor (NEMA 17)
- Servo Motor (SG90)
- Arduino IoT Cloud
- Arduino IDE

## Topology

<img width="300" height="200" alt="topologi drawio (1) drawio" src="https://github.com/user-attachments/assets/e1500e80-add0-47d3-94a5-4a17969d12d3" />

## Prototype Photos

<img src="https://github.com/user-attachments/assets/19971688-61e3-476d-b840-3700af050ca2" alt="Alt text" width="400" height="200">
<img src="https://github.com/user-attachments/assets/ce216e0c-a362-4924-9b6c-122442d0c8df" alt="Alt text" width="300" height="400">

## Demo Video
[Demo Video](https://drive.google.com/file/d/16MWlpVKUYJ-HVTxK6Zk0srXYa2Ej0ukU/view?usp=sharing)

## Testing

Functional testing was performed using Black Box Testing to ensure all features operate according to system design.

## Notes

This repository only contains the firmware source code. Hardware design files, mechanical drawings, and full documentation are part of the academic thesis.
