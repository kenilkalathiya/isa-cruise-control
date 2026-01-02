# Intelligent Speed Adaptation (ISA) – CARLA Simulator

This project implements a **state-based cruise control system with Intelligent Speed Adaptation (ISA)** using the CARLA simulator.

## Features
- State-based cruise control (ACCEL / CRUISE / DECEL)
- PID-based cruise-hold
- Simulator-safe control logic (CARLA 0.9.13)
- Intelligent Speed Adaptation (ISA) using simulated speed limits
- Deterministic control using synchronous mode

## Technologies
- Python 3.7
- CARLA Simulator 0.9.13
- PID control
- State machine design

## How It Works
- The vehicle maintains a target speed using cruise control
- Speed limits change dynamically (simulated)
- Cruise control automatically adapts to new speed limits

## Current Status
✔ Cruise Control complete  
✔ ISA logic complete  
⬜ Camera sensor integration (next step)

## Author
Your Name
