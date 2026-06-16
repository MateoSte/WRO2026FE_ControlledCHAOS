# Controlled C.H.A.O.S. – Autonomous Driving System  
## WRO Future Engineers – Engineering Documentation  

---

#  Table of Contents
- Team Information  
- Project Overview  
- Hardware Architecture  
- Mechanical Design and Reasoning  
- Sensor Strategy and Early Problems  
- Gyroscope Integration  
- Vision System and Obstacle Detection  
- Obstacle Strategy  
- Navigation Strategy  
- Sensor Reliability and Error Handling  
- System Robustness  
- Performance Evaluation  
- System Limitations  
- Engineering Conclusion  
- Software Implementation  
- Hardware & Software Architecture Diagrams  

---

# Team Information

**Team name:** Controlled C.H.A.O.S.  
**Location:** Josipovac, Croatia  

**Team members:**
- Mateo Štefanek, Grade 8, Primary School Josipovac  
- Leon Žlender, Grade 7, Primary School Retfala  

Both team members have vast experience in robotics, programming, mathematics and technical competitions.

In the previous WRO season, the team achieved:
- 2nd place at WRO Open Championship Slovenia  
- RoboMission Junior category experience  

---

# Project Overview

This project represents the development of an autonomous vehicle designed for the WRO Future Engineers competition.

The goal was not only to build a robot that can complete a track, but to design a system that remains stable under:
- unpredictable conditions  
- sensor noise  
- changing environmental inputs  

During development, we realized that the main challenge was not individual components, but how to combine unreliable inputs into a stable decision-making system.

The focus shifted from:
> “making the robot move”  
to  
> “making the robot behave consistently even when parts fail”

---

#  Hardware Architecture

**Platform:** LEGO SPIKE Prime  
**Programming:** Pybricks  

### Actuators:
- 1x Large angular motor (drive system)  
- 1x Large angular motor (Ackermann steering system)  

### Sensors:
- 3x Ultrasonic sensors  
  - left wall detection  
  - right wall detection  
  - front sensor on steering axis  
- 1x Color sensor (bottom mounted)  
- Gyroscope (SPIKE Prime hub IMU)  
- Smartphone camera (USB vision system)

---

# Mechanical Design and Reasoning

We selected LEGO SPIKE Prime due to:
- familiarity from previous competitions  
- modularity  
- ease of prototyping  

### Initial approach:
- differential drive system  

### Final decision:
- Ackermann steering system  

### Why Ackermann:
- more realistic vehicle behaviour  
- reduced slipping in turns  
- better trajectory prediction  

### Trade-off:
- steering errors became more sensitive  
- required gyroscope + correction system

---

# Sensor Strategy and Early Problems

Initial system used ultrasonic sensors for wall centering using PID control.

### Issues:
- unstable sensor readings  
- oscillation between walls  
- small errors caused large steering corrections  

### Key problem:
Not PID logic — but unreliable sensor data.

### Result:
We switched from:
> wall centering  
to  
> outer wall following

This significantly improved stability.

---

# Gyroscope Integration

Gyroscope was introduced for:
- heading stability  
- smoother turns  

### Problem:
- cumulative drift after multiple turns  

### Solution:
Hybrid correction system:
- Gyroscope → short-term stability  
- Ultrasonic walls → long-term correction  

---

# Vision System and Obstacle Detection

Initial system:
- HuskyLens + ESP32  

### Problem:
- communication instability  
- system crashes in competition conditions  

### Final solution:
- smartphone camera via USB (Pybricks integration)

### Output data:
- x position  
- y position  
- width / height (noise filtering only)

---

# Obstacle Strategy

Robot behavior depends on track direction.

### Process:
1. detect start direction (color markers)  
2. select relevant obstacle color  
3. detect obstacle in camera frame  
4. rotate until safe lateral position  
5. continue movement  
6. verify area ahead  

### Key idea:
Obstacle avoidance is **continuous feedback**, not path planning.

---

# Navigation Strategy

Default mode:
> outer wall following

### Why:
- more stable than dual-wall centering  
- fewer oscillations  
- consistent reference edge  

---

# Sensor Reliability and Error Handling

### Ultrasonic issues:
- random spikes  
- false distances  

### Solution:
- multi-sample validation  
- trimmed averaging (remove min/max)

```python
for i in range(15):
    udalj.append(sensor.distance())

udalj.remove(max(udalj))
udalj.remove(min(udalj))
