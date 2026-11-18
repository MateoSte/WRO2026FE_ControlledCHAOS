# Development Log: From LEGO SPIKE Prime to a Custom Platform
### Team C.H.A.O.S. Josipovac — WRO 2026 Future Engineers
### Period covered: June 2026 – October 2026

> This document records our engineering path from the first robot ("Controlled C.H.A.O.S.", LEGO SPIKE Prime), which we used at the Croatian National Final, to the custom platform we are building for the WRO 2026 International Final.
> It explains **what we had, what went wrong, what we suspected, what we tested, and why we decided to replace each part.**
> The SPIKE Prime version of the code has been public in this repository since June 2026 and is kept unchanged in the 'Legacy' folder under the tag `v1.0-national-spike`. The control logic described here (wall following, color classification, corner entry, state sequence) is being carried over to the new platform.

**Contents**
1. [Bugs & Issues](#1-bugs--issues) 
2. Project Development Ideas
3. Hardware Architecture Upgrades
4. Mechanical Design and Reasoning
5. Sensor Strategy
6. Gyroscope Integration
7. Vision System and Obstacle Detection
8. Obstacle Strategy
9. Navigation Strategy
10. Sensor Reliability and Error Handling
11. System Robustness
12. Performance Evaluation
13. System Limitations
14. Engineering Conclusion
15. Software Implementation
16. Hardware & Software Architecture Diagrams

---

## 1. Bugs & Issues

### 1.1 Starting point: the SPIKE Prime robot

| Subsystem | Implementation (v1.0) |
|---|---|
| Controller | LEGO SPIKE Prime hub |
| Steering | Ackermann steering driven by a SPIKE motor |
| Heading | Built-in hub gyroscope (yaw) |
| Wall distance | LEGO SPIKE distance (ultrasonic) sensor |
| Vision | OpenMV Cam H7, connected to the hub through an intermediate board using the PUPRemote library |
| Wall following | `vozi_uz_zid()` — proportional (P) controller on the ultrasonic distance |
| Corner entry | Trimmed-mean of front distance readings |
| Color classification | `scan_colors()` — 50 samples, majority vote, result mapped to one of four scenario functions |

At the National Final we were the only team that maged to complete all laps in the Open Challenge. However, our own tests during preparation showed that this result was **not reliable enough**: in a large share of runs as we did the testing the robot drifted towards the walls, and every lap carried a real risk of ending the round. It caused a lot of headaches so we tried to explain the reasons for that for the teams that will tryto do the same approach:  

### 1.2 Issue overview

| ID | Issue | Challenge | Suspected / confirmed cause |
|---|---|---|---|
| Bug 1 | Heading error grows with every 90° turn | Open, Obstacle | Gyro drift with turn error accumulating over multiple turns |
| Bug 2 | Steering not repeatable, asymmetric left vs. right | Open, Obstacle | Backlash in the SPIKE steering motor, expected in LEGo design  |
| Bug 3 | Mis-calculations in turns and correction functions | Open, Obstacle | Logic errors |
| Bug 4 | Robot drifts away from the lane centre and approaches the wall | Open | Consequence of Bug 1 + Bug 2; we tried to solve it by adding left and right ultrasonic sensor |
| Bug 5 | Unreliable color recognition of traffic signs | Obstacle | Lighting sensitivity, similar hues (red/orange/magenta, green/blue) |
| Bug 6 | Unstable camera–hub communication | Obstacle | Indirect connection (camera → intermediate board → hub) |
| Bug 7 | Many chained calculations per lap; errors combine | Both challenges | System-level: each subsystem adds uncertainty |

### 1.3 Bug 1 — Heading error accumulating over the turns

**Symptom.** In the Open Challenge the robot makes 12 turns of 90° (3 laps × 4 corners). Over several laps we observed that the turns were no longer exactly 90°: after each corner the robot left the turn at a slightly wrong angle, moved further away from the planned trajectory and closer to the edge of the lane. Further more, we noticed that going into circle clockwise appeares to be less accurate that while driving in counterclockwise circle. Probably in combination of errors with gyro and Spike's motor troubles.

**What we suspected.** Our turn logic relied on the gyroscope angle being correct. Two error sources add up:
- **Gyro drift** — a small bias in the measured angular velocity is integrated over time, so the yaw angle slowly "walks" even when the robot drives straight.
- **Turn error carried forward** — if a turn ends a few degrees short or long and the next turn is calculated from where the robot actually is, the error stays in the system and the next corner starts from a wrong position.

**Evidence.** We had test done with robot connected via Bluetooth during runs with writing out on console gyro readings where it has been show that,e.g. when robot turns from 90 degree lane back to 0 heading (same as start) it never goes back to 0. There is always some degree of inaccuraccy which tends to become larger by every turn. We had programming corrections to scale down the error, but it always appears as robot acctually doesn't have a straight line anymore, but more likely a curved path where you are not sure where it will stop on its end.  

The gyro remained the main source of heading, so the underlying problem was reduced, not removed.

### 1.4 Bug 2 — Steering backlash and left/right asymmetry

**Symptom.** The same steering command did not always give the same wheel angle, and the robot reacted differently to left and right corrections.

**Cause.** SPIKE Prime motors and LEGO gears have mechanical play (backlash). For small corrections, which is exactly what driving straight needs, the backlash is of the same order as the command itself: the motor turns, but the wheels barely move, and then they jump. Or in other case, motor stops, and wheels do a slight turn backwards which is enough to create an error in direction. 

**Why it matters.** Backlash turns a smooth controller into a "dead zone + jump" system. It made Bug 1 worse (turn angles less repeatable) and caused mistakes in straight driving.

**Status.** We could reduce it with stiffer connections, but not eliminate it on LEGO hardware. This was one of the main arguments for a replacement with alternative mechanism.

### 1.5 Bug 3 — Software bugs found during testing

| Bug | What happened | Fix |
|---|---|---|
| Asymmetric turn drift | `okrenutDesno()` (turn right) contained an **extra `while` loop** that was not present in the left-turn routine, so right turns ran slightly longer and ended at a different angle than left turns. | Removed the extra loop; left and right turn routines made symmetric. |
| Bang-bang correction in movement asymmetry | Straight-line correction used an on/off (bang-bang) rule: full correction towards left or right. Combined with Bug 2, the physically asymmetric servo response was **manually upgraded** but which caused zig-zag motion. | Replaced by proportional correction (P controller) — the correction is proportional to the error. |
| Negative tolerance → infinite loop | A tolerance value in an "until angle reached" loop became negative in one case, so the exit condition could never be true and the robot kept turning. | Tolerance forced to be positive (absolute value) and loop given a safety exit,and we watched more over prefix signs. :D  |

**Lesson.** Some of what looked like "sensor problems" were in fact our own code. From this point we started testing left and right manoeuvres separately and comparing them.

### 1.6 Bug 4 — Drifting towards the wall: ultrasonic centring

**Problem.** Because of Bug 1 and Bug 2, the robot gradually lost its position in the center of the "road". The gyro tells the robot where it is *pointing*, but not *where it is* between the walls.

**Solution in v1.0.** We added distance measurement to the side walls with the ultrasonic sensors on both sides and a P controller (`vozi_uz_zid()`) that keeps the robot at a target distance from the wall, i.e. in the middle of the lane instead of slowly approaching the edge. For corner entry we used a **trimmed mean** of several front-distance readings (the highest and lowest values are discarded), so a single wrong echo could not trigger the turn too early or too late.

**Remaining limitations.**
- The corridor width changes between 600 mm and 1000 mm in the Open Challenge, so the "centre" changes from section to section.
- When the robot is not parallel to the wall, the ultrasonic beam hits the wall at an angle and the measured distance is longer than the real one.
- One ultrasonic sensor measures one direction at a time; the inner wall disappears at corners.
- In the Obstacle Challenge, a traffic sign between the robot and the wall gives a false "wall" reading.

### 1.7 Bug 5 — Color recognition of traffic signs

**Problem.** In the Obstacle Challenge everything depends on recognising red and green pillars correctly. This proved harder than expected:
- Recognition depended strongly on the lighting; values calibrated at home did not match the competition venue, or sometimeseven daylight in the room was a deciding factor of the accuracy of the test.
- Red can be confused with the orange line and the magenta parking walls; green can be confused with the blue line.
- Objects outside the field (clothes, bags) can produce false detections. Near competition's area there was a white and red goal of the handball court where we, during the test, noticed that it appeares in the reading as an red traffic sign during some of the tests.   

We are still considering will we continue to use 50 readings of the camera to calculate the trajectory path down the lane or will we adapt a solution to a more accurate version before competition. This kind of testing reduces noise but does not fix a systematic error: if the lighting shifts the colors, all 50 samples are wrong in the same way. The scenario approach also depends on the robot being exactly where the scenario expects it.

### 1.8 Bug 6 — Camera integration with SPIKE Prime

**Problem.** SPIKE Prime cannot connect an OpenMV camera directly, so the camera was connected through an intermediate board using PUPRemote (`PUPRemoteSensor` on the hub side). This worked, but created several issues:
- **Small data packet:** the message format was limited (`"bhhhh"`, `max_packet_size = 16`), so we could send only a few values per frame (e.g. color and position of one object), not a full description of the scene.
- **Firmware/API changes:** in OpenMV firmware v4.8.1+ the `sensor` module was replaced by `csi`, which broke our camera code until we updated it.
- **Port assignment errors:** wrong port configuration between camera, board and hub caused the connection to fail without a clear error message.
- **Extra point of failure:** every additional board and cable in the chain is one more thing that can fail during a round.

**Earlier attempts** Before OpenMV we tried a HuskyLens camera with an LMS-ESP32 board and a smartphone-based camera solution via Pybricks app that works offline and direct USB connection to Spike. Both were abandoned in favour of OpenMV because of smartphone doesn't have an option to do a re-calculation and adjusting of colors, and HuskyLens showed a major problems for us in connecting and maintaining a live feed towards the Spike hub.

### 1.9 Bug 7 — System-level conclusion

Each subsystem on its own was "almost good enough", but in a run they are **chained**: gyro angle → turn → position in lane → ultrasonic correction → camera sees the sign → scenario chosen → avoidance path. A small error at the start of the chain becomes a large error at the end. In practice this meant that **every lap was a risk**, even though the robot was able to get us through the National Finals.

From this analysis we derived the requirements for the next version:

| Requirement | Addresses |
|---|---|
| Steering without noticeable backlash, repeatable left/right | Bugs 2,3 |
| Better heading sensor and a way to correct drift with walls | Bugs 1,4 |
| Measuring position relative to the walls in more than one direction | Bug 4 |
| Camera connected directly to the main controller, with enough bandwidth for full detection results | Bugs 5,6 |
| Controllable camera settings (fixed exposure, gain, white balance) | Bug 5 |
| Fewer intermediate boards and connections | Bugs 6, 7 |
| Drivetrain compliant with WRO rules (driving wheels mechanically connected, one drive axle) | rules 11.3, 11.5, 11.13 |

These requirements were the starting point for our first redesign idea — a Raspberry Pi based platform — described in **Section 2: Project Development Ideas** and **Section 3: Hardware Architecture Upgrades**.

---

## 2. Project Development Ideas

### 2.1 Design principles we set for the new version

After the National Final we agreed on a few principles before choosing any components:

1. **Keep the logic, change the platform.** The control ideas from v1.0 worked (wall following with a P controller, trimmed-mean corner detection, majority-vote color classification, scenario/state sequence). The problems were in the hardware underneath them. We therefore port the logic and replace the hardware, where ever we can.Of course, in every step, improvement on sofware side is also always possible and wanted.
2. **Every new component must solve an issue** from Section 1 (Bugs 1–7). No component is added "because it is better" without a reason.
3. **Fewer links in the chain.** Every intermediate board, adapter or cable is a possible failure point during a round, so the final solutions is always better if it can be simplified.
4. **Rule compliance first.** Driving wheels mechanically connected (one drive axle, no electronic differential), one power switch, one start button, no wireless communication during rounds, max. 300 × 200 × 300 mm and 1.5 kg.
5. **Buildable and repairable by us**, with parts we can buy quickly in the EU (Soldered — Osijek (local manufacturer), Botland, Berrybase).

### 2.2 Development timeline

| Version | Period | Platform | Status |
|---|---|---|---|
| v1.0 | until National Final | LEGO SPIKE Prime + OpenMV H7 via PUPRemote | Competed; kept as tag `v1.0-national-spike` |
| v2.0 (concept) | July-August | Raspberry Pi 5 + Raspberry Pi Pico 2 + OpenMV H7 + discrete sensors | Planned (BOM, wiring, GPIO plan); not built as final solution |
| v3.0 | August – now | Soldered NULA DeepSleep ESP32-S3 + OpenMV AE3 | In development — target for the International Final |

### 2.3 Ideas considered

| Idea | Why it was attractive | Decision |
|---|---|---|
| Improve the SPIKE robot further | Already working, known code | Rejected as final solution: backlash (B2) and camera link (B6) cannot be solved on LEGO hardware, sensors very unreliable |
| Single-board computer (Raspberry Pi 5) as main controller | High processing power for vision and sensor fusion | Explored in v2.0  |
| Split architecture: computer + microcontroller | Real-time motor/servo control separated from high-level logic | Explored in v2.0; kept in simplified form in v3.0 (ESP32-S3 + camera with its own processor) |
| Camera with built-in distance and inertial sensing (OpenMV AE3) | Fewer separate sensors and connections | Adopted in v3.0 |
| 360° LiDAR (used by several podium teams in 2024–2025) | Full wall geometry in every direction, strong localisation | Considered as a part of solution but would be complete change the idea, and be costly, both in time and money, so we skipped it for the first year |

---

## 3. Hardware Architecture Upgrades

### 3.1 Architecture overview

| Function | v1.0 SPIKE | v2.0 concept (RPi) | v3.0 current (ESP32-S3) |
|---|---|---|---|
| Main controller | SPIKE Prime hub | Raspberry Pi 5 | Soldered NULA DeepSleep ESP32-S3 |
| Real-time control | SPIKE hub | Raspberry Pi Pico 2 | ESP32-S3 |
| Vision | OpenMV H7 (via PUPRemote board) | OpenMV H7 (direct UART) | OpenMV AE3 (direct UART) |
| Distance to walls | SPIKE ultrasonic | 2 × VL53L0X ToF | ToF built into OpenMV AE3 + additional sensors VL53L1X (improved version) |
| Heading | Hub gyro | MPU-6050 | IMU built into OpenMV AE3 or stand-alone MPU-6050 (still testing, and dependant about idea that AE3 can be rotated) |
| Floor lines | Camera / SPIKE sensor [TODO] | TCS34725 color sensor | [TODO] Will be analyzed if it is neccessary or number of laps can be calculated via ToF sensors on the sides of vehicle. |
| Sensor bus | LEGO ports | I²C via TCA9548A multiplexer | I²C over Soldered easyC (Qwiic-compatible) connectors, sensors connected through easyC hub ; camera on separate UART . This will ease up setting up and connecting multiple devices with the same address (like ToF sensors) by switching between them via different channels. |
| Drive motor | SPIKE motor | Pololu #4864 gearmotor | Waveshare N20, 1:150, Hall encoder |
| Motor driver | integrated | DRV8871 | SparkFun TB6612FNG |
| Battery | SPIKE battery | 3S LiPo | 3 × 18650 Li-ion (TR18650) as one source |
| Logic power | integrated | 3S LiPo with UBEC 5V/3A | UBEC → logic rail (ESP32-S3, OpenMV AE3, encoder) |

### 3.2 v2.0 concept: Raspberry Pi 5 + Pico 2

**Idea.** Split the system into two levels, which is a common pattern in Future Engineers:
- **Raspberry Pi 5** — high-level logic: reading the camera results, deciding which side to pass a sign, lap counting, parking.
- **Raspberry Pi Pico 2** — real-time level: motor PWM, encoder counting, servo, reading sensors at a fixed rate.
- **OpenMV H7** connected directly over UART — no intermediate board (solves Bug 6).

**Sensors chosen and why.**
- **2 × VL53L0X (time-of-flight)** instead of ultrasonic: narrow beam and fast measurements, so less error expected when the robot is not parallel to the wall (Bug 4) and fewer false echoes.
- **MPU-6050 (gyro + accelerometer)** as a dedicated heading sensor (Bug 1).
- **TCS34725 color sensor** pointed at the floor for the orange and blue lines, independent of the camera (Bug 5).
- **TCA9548A I²C multiplexer**, because the VL53L0X and TCS34725 share the same default I²C address (0x29); the multiplexer lets them sit on one bus without address conflicts.

**What we produced.**  Wiring diagrams, GPIO pin assignment. [TODO: link to files in  `/docs`]

**Why we did not continue with v2.0.** 
The Raspberry Pi 5 is by far the most powerful processor we considered. We did not drop it because it was too weak, but because our architecture did not need that power in the main controller, while it added complexity we could not afford before December. The decisive reasons were:

- *Vision moved into the camera, so the controller only needs results.* In v2.0 the Raspberry Pi was planned to combine camera data with the other sensors. The OpenMV AE3 processes the image on its own processor and sends only the results (color, position, size of an object). The main controller therefore handles small messages and control loops — a job for a microcontroller, not a computer with an operating system as Pi 5. 
- *Deterministic real-time control and instant start (ESP32-S3).* The ESP32-S3 runs our program directly, without an operating system. The control loop (steering, motor speed, encoder) runs at a fixed, predictable rate, and the robot is ready a moment after the power switch is turned on. A Raspberry Pi first has to boot Linux and start the program, which takes time out of the 90-second preparation window and adds a failure point (e.g. a damaged SD card). In v2.0 we would need the Pico 2 precisely to get real-time control; the ESP32-S3 covers that role alone. 
- *Fewer devices, fewer connections (Bug 6, Bug 7).* v2.0: Raspberry Pi 5 + Pico 2 + OpenMV H7 + I²C multiplexer + separate sensors → three programmable devices and two communication links between them. v3.0: ESP32-S3 + OpenMV AE3 → two programmable devices and one link. Less code to write and debug, and fewer places where the system can fail during a round.
- *Better distance sensing:* VL53L1X instead of VL53L0X. The VL53L1X time-of-flight sensor became available to us as a module with the same connection system. Compared with the VL53L0X it measures up to about 4 m instead of about 2 m and has a configurable measuring area (region of interest). This matters on a 3 × 3 m field where we want to see the far wall before a corner. 
- *easyC/Qwiic connection system — speed of changes.* Sensors connect with plug-in cables, without soldering and without designing a separate board. With little time until the final, this let us replace and rearrange sensors in minutes and test different positions quickly. 
- *Local manufacturer and support.* The controller board, ToF sensors, batteries, wiring, soldering tools, al is made by Soldered Electronics in Osijek, our home town. This gives us fast delivery and replacement parts within a day. 
