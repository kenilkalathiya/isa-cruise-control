# Intelligent Speed Adaptation (ISA) – CARLA Simulator

This project implements a **state-based cruise control system with Intelligent Speed Adaptation (ISA)** using the **CARLA simulator**.  
The system maintains a target speed, adapts automatically to speed limits, and visualizes vehicle state using a front-facing RGB camera.

---

## 🚗 Features

### Cruise Control
- State-based longitudinal control (ACCEL / CRUISE / DECEL)
- PID-based cruise-hold control
- Smooth acceleration and deceleration
- Simulator-safe logic for CARLA 0.9.13

### Intelligent Speed Adaptation (ISA)
- Dynamic speed-limit handling
- Automatic adaptation of target speed
- Simulated speed-limit changes (for testing)

### Camera & Visualization
- Front-facing RGB camera attached to the vehicle
- Live camera feed using OpenCV
- Real-time overlay showing:
  - Vehicle speed
  - Current speed limit
  - Control state (ACCEL / CRUISE / DECEL)

### Simulator Stability
- Synchronous simulation mode
- Deterministic control loop
- Non-blocking sensor callbacks
- Stable OpenCV window handling


## Note on Simulation Constraints
Due to GPU memory limitations, some CARLA towns could not be loaded reliably.
To ensure stable testing, zone-based speed-limit logic was implemented to emulate
real-world speed-limit transitions while validating ISA behavior.


---


This separation allows perception, decision-making, and control to evolve independently.

---

## 🛠️ Technologies Used

- Python 3.7
- CARLA Simulator 0.9.13
- OpenCV (cv2)
- NumPy
- PID control
- State machine design

---

## ▶ How to Run

1. Start the CARLA simulator
2. Wait for the map to load
3. Run the script:

```bash
py -3.7 cruise_control_final.py

## 🧠 System Architecture

