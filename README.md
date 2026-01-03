# Intelligent Speed Assistance (ISA) & Adaptive Cruise Control (ACC) in CARLA

This project implements a **vision-assisted Intelligent Speed Assistance (ISA)** system combined with **Adaptive Cruise Control (ACC)** using the CARLA autonomous driving simulator.  
The system demonstrates realistic ADAS behavior including speed regulation, safe-distance keeping, driver override handling, and lead-vehicle interaction.

---

## 🚗 Features

### Intelligent Speed Assistance (ISA)
- Dynamic speed-limit enforcement (zone-based)
- Smooth PID-based longitudinal control
- Driver override logic with cooldown and recovery
- ISA re-engagement after override timeout

### Adaptive Cruise Control (ACC)
- Time-gap–based safe distance control
- Continuous lead-vehicle detection
- Speed adaptation based on relative distance
- Works even during ISA driver override (safety-first design)
- Emergency braking safeguard (AEB-like behavior)

### Lead Vehicle Scenario
- Deterministic lead vehicle spawning
- Waypoint-based steering to follow road curvature
- Time-based acceleration and braking profile
- Stable ACC validation scenario

### Perception & Visualization
- Front RGB camera using CARLA sensors
- OpenCV-based visualization overlays
- Real-time display of:
  - Vehicle speed
  - Target speed
  - ISA state
  - ACC lead distance
  - System timers

---

## 🧠 System Architecture

**Control Priority (Highest → Lowest):**
1. Emergency Braking (AEB-like)
2. Adaptive Cruise Control (distance safety)
3. Driver input
4. Intelligent Speed Assistance (speed limits)
5. Cruise PID controller

This hierarchy ensures **collision avoidance is never overridden**.

---

## 🛠️ Technologies Used

- Python
- CARLA Simulator
- OpenCV
- NumPy
- Control Systems (PID)
- Autonomous Driving Concepts (ADAS, ACC, ISA)

---

## ⚙️ How to Run

### Requirements
- CARLA Simulator (0.9.x)
- Python 3.8+
- OpenCV
- NumPy

### Run CARLA
```bash
CarlaUE4.exe
