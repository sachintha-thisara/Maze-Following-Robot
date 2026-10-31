# Autonomous Maze-Following Robot

An autonomous maze-following robot developed as an engineering project at the **University of Moratuwa, Sri Lanka**.

## Project Overview

This project focuses on developing a robot capable of navigating a maze autonomously using ultrasonic sensors and programmed navigation logic.

## Key Features

- **Obstacle Detection:** HC-SR04 ultrasonic sensors for distance measurement and wall detection.
- **Navigation Algorithm:** Left-Hand / Right-Hand rule logic for maze navigation.
- **Sensor Processing:** Median filtering to improve ultrasonic sensor readings.
- **Control System:** PD-based wall-following controller for more accurate movement.
- **Wall Recovery:** Recovery logic to handle certain navigation situations.

## Hardware Components

- Arduino Microcontroller
- HC-SR04 Ultrasonic Sensors
- Motor Driver
- DC Gear Motors
- Custom-designed Robot Chassis

## Mechanical Design

The robot chassis was designed using SolidWorks.

Design files are available in the [Mechanical-Design](./Mechanical-Design) folder.

## Arduino Code

The Arduino control program is available in the [Code](./Code) folder.

## Assembly

Photos showing the robot assembly process and completed robot are available in the [Assmbly](./Assmbly) folder.

## Demonstration

Watch the robot's maze-solving demonstration:

[▶ Watch Maze-Solving Robot Video](https://drive.google.com/file/d/1A9j83h1_tlYFu_B49Y2Wqrg0i0gVXniF/view)

## Repository Structure

- `Code/` – Arduino control code
- `Mechanical-Design/` – Mechanical design files
- `Assmbly/` – Assembly and completed robot photos
- `Demonstration` – Maze-solving video link

## Project Status

Developed as an engineering project at the University of Moratuwa.
