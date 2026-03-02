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

## Testing

Functional testing was performed using Black Box Testing to ensure all features operate according to system design.

## Academic Context

This project was developed as a final thesis under the supervision of Dr. Mochammad Haldi Widianto, S.T., M.T. at BINUS University. The research was presented at the International Conference on Cybernetics and Intelligent Systems (ICORIS) 2024.

## Notes

This repository only contains the firmware source code. Hardware design files, mechanical drawings, and full documentation are part of the academic thesis.
