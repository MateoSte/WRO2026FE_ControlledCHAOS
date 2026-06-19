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

Both team members have several experiences in robotics, programming, mathematics and technical competitions.

In the previous WRO season, the team achieved
2nd place at WRO Open Championship Slovenia, and few years in RoboMission Junior category, we decided to go into this project.  

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
**Programming:** Pybricks / Python 

### Actuators:
- 1x Large angular motor (drive system)  
- 1x Large angular motor (Ackermann steering system)  
   ![LEGO Spike Prime Large angular motor](images/LargeSpikePRIMEangularMotor.jpg)

### Sensors:
- 3x Ultrasonic sensors  
  - left wall detection  
  - right wall detection  
  - front sensor on steering axis  
  ![LEGO Spike Prime Ultrasonic sensor](images/SpikePRIMEultrasonic.jpg)

- 1x Color sensor (bottom mounted)  
![LEGO Spike Prime Color sensor](images/SpikePRIMEcolorSensor.jpg)

- Gyroscope (SPIKE Prime hub IMU)  
- OpenMV camera integrated over Spike OpenMV microcontroller
![OpenMV cam](images/Image_openMV.jpg)

---

#  Mechanical Design and Reasoning

We decided to use LEGO Spike Prime as our tool of choice as it is familiar to us from previous competitions and has many parts which simplify the creation of solution for this task.

At the beginning of development, we used differential drive systems, as they were familiar from previous competitions. However, as this category demands it, we decided to switch to Ackermann steering. 

This decision was not immediate. The main reason for analyzing the change was because differential drive has totally different trajectories while driving, pre-built functions for easing-up of motions of vehicle, especially when calculating into the story its ability to rotate at its center. 

Ackermann steering allowed us to: 

- simulate more realistic vehicle behaviour  

- reduce unnecessary slipping during turns  

- simplify trajectory prediction during obstacle avoidance  

However, this introduced a new challenge: steering errors became more noticeable, and small inaccuracies in motor control had a larger impact on the robot’s direction. This later influenced how we designed our correction system using gyroscope and wall tracking.

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

#  Sensor Strategy and Early Problems

At the beginning of the project, we tried to rely heavily on ultrasonic sensors for navigation. The idea was to center the robot between the left and right walls using a simple PID controller. This was out initial idea: 

 ![Sketch of initial design idea](images/Screenshot_20260611-100914.png)

 

This approach worked in theory, but in practice we encountered several problems: 

- ultrasonic sensors produced unstable readings in certain distances  

- small measurement errors caused large steering corrections  

- the robot started oscillating between walls instead of moving smoothly  

The main issue was not the PID logic itself, but the reliability of input data coming from sensors. 

For example, robot needs to dynamically position its self between the inner and outer wall and drive through center of the lane, but in one case it finds out that it is too close to left wall, steers right to re-center, and in one second sees the crossroad, although it is not close to it (just the angle of sensor discovers it) and decides to make a hard turn to left, trying to keep the midle line, but it actually starts going too close to the opposite wall. 

  ![Sketch of Ultrasonic Sensor bad reading case](images/Screenshot_20260614-122419.png)

 

Because of this, we changed the navigation strategy from “centering between walls” to “following the outer wall consistently”. 

This immediately reduced oscillations and made the movement more predictable. 

 

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
After abandoning the full PID centering approach, we introduced gyroscope-based correction. 

The idea was to maintain a stable heading while following the outer wall. 

This improved stability significantly, but we discovered a new problem: after multiple turns, small angular errors started to accumulate. This drift did not cause immediate failure, but over time it caused a noticeable deviation from the expected path. 

To compensate for this, we combined gyroscope data with ultrasonic wall reference correction. This created a hybrid system where: 

- gyroscope maintains short-term stability  

- wall sensors correct long-term drift  

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

#  Vision System and Obstacle Detection

Initially, we attempted to use HuskyLens combined with an ESP32 microcontroller for obstacle detection. However, during testing we experienced communication instability between ESP32 and the SPIKE Prime hub. 

The main issue was not processing capability, but reliability. In a competition environment, even occasional communication failure caused the entire program to stop, which was unacceptable. 

Because of this, we completely redesigned the vision system, twice, and firstly moved to a smartphone-based camera solution connected via USB and integrated through Pybricks smart sensor interface, and secondly, we altered that decision to go into the story of connecting OpenMV camera via Spike OpenMV microcontroller.

This decision, in both cases, significantly reduced system complexity and removed one additional point of failure. 

The camera provides: 

> x and y position of detected object 


> width and height of detected object  

For navigation, only the x-coordinate is used, while width and height are used only for filtering noise. 

Reason why we moved from smarthpone as a sensor to OpenMV was that  the smartphone camera pipeline introduced latency and instability in color detection that was difficult to control. The image processing happened on a separate device with its own exposure and white balance logic, and the results varied significantly depending on ambient lighting conditions at the competition venue.
The OpenMV camera connected via a wired PUPRemoteSensor/PUPRemoteHub link solves both problems. The communication is fully wired (rule-compliant), and color detection is done directly on the OpenMV chip using LAB color space thresholds. The key advantage is the Threshold Editor built into OpenMV IDE — it lets you visually tune the LAB ranges for red and green pillars under the actual lighting conditions of the venue in a matter of minutes, without changing any control logic on the hub side. This makes on-site calibration straightforward and repeatable.

Initial system:
- HuskyLens + ESP32  
- smart-based camera with Pybricks integration

### Problem:
- communication instability  
- latency and instability in color detection
- system crashes in competition conditions  

### Final solution:
- OpenMV camera solution with microcontroller integration

### Output data:
- x position  
- y position  
- width / height (noise filtering only)


 ![Spike Prime integrated with Smartphone Sensor 1](images/Image_pybricks_smartphone1.jpg)

 ![Spike Prime integrated with Smartphone Sensor 2](images/Image_pybricks_smartphone2.jpg)

  ![Spike Prime integrated with Smartphone Sensor 3](images/Image_pybricks_smartphone3.jpg)

   ![Spike Prime integrated with Smartphone Sensor 4](images/Image_pybricks_smartphone4.jpg)


   ![Spike Prime integrated with OpenMV camera](images/Image_OpenMV_Spike.jpg)
---

#  Obstacle Strategy
The robot does not treat all obstacles equally. The behavior depends on the driving direction, which is determined at the beginning of the run using track markers (lines of different colors). 

Based on direction, the system selects which obstacle color is relevant. 

When an obstacle is detected by scaning of area before it: 

1. the robot rotates until the obstacle reaches a safe lateral position in the camera frame  

2. then it continues moving while maintaining avoidance trajectory  

3. after passing the obstacle, the system performs a visual check of the area ahead  

This post-check step is important because obstacles often appear in sequences. If the same type of obstacle is still detected, the robot continues avoidance behavior instead of returning to the main path too early. 

If no relevant obstacle is detected, the robot returns to outer wall navigation. 

   
### Process:
1. detect start direction (color markers)  
2. select relevant obstacle color  
3. detect obstacle in camera frame  
4. rotate until safe lateral position  
5. continue movement  
6. verify area ahead  

### Key idea:
Obstacle avoidance as a path planning.

---

#  Navigation Strategy

When no obstacle is detected, the robot follows the outer wall of the track. This decision was made after multiple failed attempts with dual-wall centering. 

Outer wall tracking proved more stable because: 

- it reduces dependency on symmetrical sensor readings  

- it avoids oscillations between two reference points  

- it provides a consistent reference edge even in irregular track sections

Default mode:
> outer wall following

### Why:
- more stable than dual-wall centering  
- fewer oscillations  
- consistent reference edge  

---

#  Sensor Reliability and Error Handling

During development, we observed several real-world issues that are not obvious in simulation. 

Ultrasonic sensors occasionally produced incorrect readings, especially at longer distances. In some cases, values appeared significantly lower than expected despite no physical obstacle being present. 

To solve this, we implemented a multi-sample validation system. A reading is only considered valid if it is consistent across multiple consecutive measurements. A single incorrect value resets the validation counter. 

Gyroscope performance improved significantly compared to previous LEGO EV3 systems. We tried them using on our models, but it just happens that in some cases robot starts going in rounds as it cannot find the correct angle. This is not the case with Spike Prime. However, minor drift still appears after multiple turns, which is corrected using wall reference alignment. 

### Ultrasonic issues:
- random spikes  
- false distances  

### Solution:
- multi-sample validation  
- trimmed averaging (remove min/max)

For example, we store several values but remove max and min values from the list, counting that only several values can be bad input

```python
for i in range(15):
    udalj.append(sensor.distance())

udalj.remove(max(udalj))
udalj.remove(min(udalj))
```

after which we calculate arithmetic mean of the rest to get the aproximation of the real value for doing the next step.


Also, we mentioned problems with our robot vision system. During development, the initial vision pipeline produced unreliable color detection results. Random spikes in recognized colors and false positive object detections caused the robot to react to non-existent obstacles, making consistent lap completion difficult. Additionally, tuning color recognition thresholds was time-consuming and tightly coupled to specific lighting conditions, with no straightforward way to recalibrate quickly on-site.
To address this, two solutions were implemented. First, a multi-sample validation approach was introduced, where a detected color is only acted upon after several consecutive consistent readings, effectively filtering out transient noise and false detections. Second, the vision system was migrated to an OpenMV camera connected via a wired interface. The OpenMV's dedicated image processor handles color  detection entirely on-chip using LAB color space thresholds, which are significantly more robust to changes in ambient lighting than raw RGB comparisons. Critically, the OpenMV IDE includes a built-in Threshold Editor that allows real-time visual adjustment of color ranges directly from the camera feed — making on-site recalibration a matter of minutes rather than requiring code changes and re-uploads to the hub.

### Vision sensor issues:
- random spikes in recognized colors 
- false objects
- adjusting of color recognition  

### Solution:
- multi-sample validation  
- implementing of OpenMV with program ability to adjust color readings with much better response due to processing ability of OpenMV itself

---
## System Robustness 


The system is designed with fallback behaviour. 

If the camera does not detect any object, the robot does not stop or enter an error state. Instead, it continues normal navigation using idea of outer wall tracking. 

This ensures that vision is an enhancement layer, not a dependency for basic movement. 


# Performance Evaluation


Based on testing, the system achieved: 

> Approximately near 100% success rate in Open Challenge conditions  

> approximately 80% success rate in Obstacle Challenge conditions  

 

Most failures occurred due to: 

- unstable sensor readings in specific track conditions  

- occasional misinterpretation of visual data in complex scenarios  

- mechanical stress on structural components due to robot weight  
System Limitations 

The main limitations identified during development are: 

### Mechanical load 
The robot structure experiences increased stress due to added weight and motor torque, which affects long-term durability.
This was the reason where we had to change inital idea of design where we had drive motors in different position and using differential transmission to run the wheels. Torque on gears produced grear slipping wherewe at one point just had to remove the gears, change directionof motor, and connect wheels direct to it.  

### Power consumption 
Version with LS-ESP32 and HuskyLens cam needed to have additional power source for camera itself. The smartphone camera increased overall energy usage, requiring monitoring of battery levels during longer runs on both smartphone and Spike Prime hub. OpenMV solved both of this problems by providing longer time of work available. 

### Voltage sensitivity 
The system operates optimally between 7.6V and 8.1V on hub. Below this range, motor response and sensor reliability decrease noticeably. A voltage check is performed at startup to ensure safe operation (visible through Pybricks console). 

 
---
### Engineering Conclusion 

This project evolved through multiple design iterations, starting from simple sensor-based navigation and gradually moving toward a layered autonomous system with redundancy and fallback mechanisms. 

The final architecture does not rely on a single perfect sensor or algorithm. Instead, it combines multiple imperfect systems in a way that they compensate for each other’s weaknesses. 

The main lesson from this project is that reliability in autonomous systems does not come from accuracy of individual components, but from how well the system behaves when those components fail or become uncertain.



---

# Software Implementation and Code-Driven Design Decisions 

During development, the final software architecture evolved into a structured object-oriented control system built around a custom `CarDriveBase` class. The purpose of this abstraction was to decouple high-level behaviour (navigation, obstacle avoidance, decision-making) from low-level motor and sensor control.

The final implementation is not a linear control program, but a layered decision system where initial scene classification drives a branching execution path, each branch independently validated through real-world testing.

---

## 1. CarDriveBase – Core Motion Abstraction

The main control system is encapsulated in the `CarDriveBase` class:

```python
class CarDriveBase:
    def __init__(self, drive_motor, steer_motor, wheel_diameter,
                 axle_track, default_speed, default_steer_speed, hub):
```

This design choice was made to solve a recurring problem in earlier versions: direct motor control scattered across multiple scripts caused inconsistent behaviour and made debugging difficult.

By centralizing motion logic, we achieved:

- consistent steering behaviour across all modules
- reusable movement primitives (`straight`, `turn`, `correct`)
- easier integration of gyroscope-based corrections
- a clean separation between navigation states and motor execution

---

## 2. Gyroscope Integration and Selective Activation

Gyroscope support is explicitly controlled through:

```python
def use_gyro(self, use):
    self.gyro = use
```

and heading correction is applied inside the cruise loop:

```python
car.correct()
```

This design reflects an important engineering decision: the gyro is not always active, but selectively enabled depending on the reliability of the current navigation mode. During straight cruise sections and corner turns it is enabled; during obstacle avoidance sequences it is explicitly disabled to prevent the correction loop from fighting against intentional lateral movement:

```python
car.use_gyro(False)
# ... avoidance turns ...
car.use_gyro(True)
```

---

## 3. Gyro Drift Mitigation Strategy

Even though the SPIKE Prime IMU performs significantly better than older EV3 systems, small cumulative heading errors were observed after multiple laps. To mitigate this, the system resets the heading reference after each bypass manoeuvre:

```python
car.use_gyro(True)
car.reset_gyro()
```

This hybrid approach combines:

- short-term stability from gyro integration during straight sections
- local reference reset after each event, preventing accumulation across laps

---

## 4. Vision System – OpenMV Camera via Wired Interface

The obstacle detection system is based on an OpenMV camera connected to the SPIKE Prime hub via a wired PUPRemoteSensor/PUPRemoteHub link. Color blob data is transmitted each frame using a structured channel:

```python
pr.add_channel('color', to_hub_fmt="bhhhh")
```

Each call to `get_color()` returns:

```python
color_id, x, y, w, h = get_color()
```

where `color_id` identifies the detected color (0 = NONE, 1 = RED, 2 = GREEN), and `x, y, w, h` describe the bounding box of the largest detected blob in the 320×240 frame. The hub-side module runs as a pure library import — the `while True` test loop is guarded under `if __name__ == "__main__":` to prevent it from blocking the main program on import.

Detection is filtered by minimum blob area before any control decision is made:

```python
if (w * h) >= MIN_BLOB_AREA and front_sensor.distance() < MAX_TRIGGER_DIST:
```

This dual-condition gate — visual confirmation plus ultrasonic proximity — was introduced after testing revealed that distant or partial detections triggered premature bypass manoeuvres.

---

## 5. Multi-Sample Color Scene Classification

Before any avoidance movement begins, the program classifies the current section of the track by sampling the OpenMV camera 50 times and aggregating results:

```python
def scan_colors():
    RED_SAMPLES = 50
    r_areas = []
    g_areas = []
    for _ in range(RED_SAMPLES):
        r, rx, ry, rw, rh, g, gx, gy, gw, gh = get_color()
        if r:
            r_areas.append(rw * rh)
        if g:
            g_areas.append(gw * gh)

    has_red   = len(r_areas) > 25
    has_green = len(g_areas) > 25
```

A color is only confirmed if it appears in more than half of the 50 samples, directly eliminating single-frame noise and lighting flicker. When both colors are present, average blob area is used to determine which pillar is closer to the vehicle:

```python
if avg_r >= avg_g:
    return "R1"  # red is closer
else:
    return "G1"  # green is closer
```

This produces one of five outcomes: `R0` (red only), `G0` (green only), `R1` (red closer, green further), `G1` (green closer, red further), or `NONE`. This classification drives all subsequent avoidance logic.

---

## 6. Scenario-Based Avoidance Branching

The main execution function `krug()` uses the classification result to select one of four dedicated avoidance routines per straight section:

```python
if exist == "G1":
    zelena_crvena()
elif exist == "R1":
    crvena_zelena()
elif exist == "G0":
    zelena_zelena()
elif exist == "R0":
    crvena_crvena()
```

Each routine encodes a specific sequence of `turn()` and `straight()` primitives tuned for that exact obstacle configuration. For example, `crvena_crvena()` (red pillar only, pass right) executes:

```python
car.turn(55, 30)
car.straight(170)
car.turn(45, -30)
```

and then hands off to `vozi_uz_zid()` to stabilize wall distance before the corner. The separation into four named functions was a deliberate engineering decision: rather than one generic avoidance function with conditional branches inside, each scenario is independently testable and adjustable without risk of affecting the others.

---

## 7. Wall-Following with Proportional Ultrasonic Control

After each avoidance manoeuvre, the vehicle realigns with the track wall using a proportional controller based on the right-side ultrasonic sensor:

```python
def vozi_uz_zid(udaljenost_mm, target_dist=None, kp=2.0, max_steer=30, ...):
    error = target_dist - dist
    steer = max(-max_steer, min(max_steer, kp * error))
    car.steer_motor.run_target(car.default_steer_speed, -steer, wait=False)
```

If no target distance is specified, the function measures the current wall distance at the start and uses it as the reference, making the behaviour adaptive to starting position rather than requiring a hardcoded constant. An `ignore_below` threshold allows the function to discard readings caused by nearby pillars that would otherwise be misinterpreted as the wall:

```python
if ignore_below is None or dist >= ignore_below:
    error = target_dist - dist
    last_valid_steer = steer
else:
    steer = last_valid_steer  # hold last valid command
```

Travel distance is measured via the drive motor encoder, making the function independent of time and consistent across different battery voltage levels.

---

## 8. Direction Initialization Using Track Colors

The function `start()` determines the driving direction before the main avoidance loop begins. It first calls `scan_colors()` and uses the result to immediately perform the initial avoidance and reach the first track line. The bottom-facing color sensor then reads the line color:

```python
bojaRaw = color_sensor.color()
if boja == "NARANCASTA":
    strana = "LEFT"
else:
    strana = "RIGHT"
```

Orange indicates the inner wall side (counterclockwise direction), blue indicates the outer wall side (clockwise). This sets the global `strana` variable used by all subsequent corner functions to decide whether to execute an inner or outer turn. This eliminates hard-coded direction logic and ensures the robot adapts dynamically to whichever starting configuration is drawn for that round.

---

## 9. Corner Execution via Distance-Referenced Turns

Corner turns are handled by two dedicated functions, `unutarnje_skretanje()` and `vanjsko_skretanje()`, both of which measure actual distance to the wall before committing to the turn:

```python
for i in range(20):
    d.append(front_sensor.distance())
for i in range(5):
    d.remove(max(d))
    d.remove(min(d))
dis = sum(d) / 10
car.straight(dis - 50)
```

Twenty samples are taken, the five highest and five lowest are discarded as outliers, and the trimmed mean is used to position the vehicle at a consistent reference distance before the 90-degree turn. This was introduced after testing showed that arriving at the corner from variable distances — due to differences in avoidance path length — caused the turn to end up misaligned with the next straight section. By anchoring to measured wall distance rather than a fixed forward travel amount, the corner entry point becomes repeatable regardless of what happened on the preceding straight.


# 11. Diagrams of Hardware and Software Architecture  

## 11.1. Hardware Diagram
   ![Hardware Diagram](images/Image_hardware_architecture.png)

## 11.2. Software Diagram
   ![Software Diagram](images/Image_software_architecture_chaos.png)


---

## 12. Robot Specifications and Physical Overview

### 12.1 Physical Dimensions

| Dimension | Value |
|---|---|
| Length | ~22 cm |
| Width | ~13.5 cm |
| Height | ~22 cm |
| Estimated weight | ~620 g |
| Wheelbase | ~12.5 cm |
| Track width | ~11.5 cm |

Note: Width, wheelbase, and track width are measured values from the final build. Length and weight are approximate. Height includes the OpenMV camera mount at full extension.

---

### 12.2 Robot Views

The following views describe the physical layout of the robot as built for the WRO Future Engineers competition.
   ![Spike Design](images/Image_Spike1.jpg)

   ![Spike Design](images/Image_Spike2.jpg)

   ![Spike Design](images/Image_Spike3.jpg)

   ![Spike Design](images/Image_Spike4.jpg)



**Top view:** The SPIKE Prime Hub is centrally mounted. The OpenMV camera is mounted at the front of the chassis on an elevated bracket, facing forward, connected via a wired PUP cable to Port E on the hub. The color sensor is mounted underneath, pointing toward the ground.

**Front view:** The front ultrasonic sensor is mounted on the steering axis, meaning its orientation rotates together with the steering angle. This design choice allows the sensor to look ahead in the direction the robot is turning, rather than always pointing directly forward.

**Side view:** The drive motor is positioned at the rear, connected to the rear wheels. The steering motor is positioned centrally in the chassis, connected to the Ackermann steering linkage. The OpenMV camera bracket extends upward and forward from the hub area to provide an unobstructed field of view of the track ahead.

**Bottom view:** The color sensor is mounted below the chassis, centered laterally, positioned to reliably read the orange and blue track surface markings used for driving direction initialization.

---

### 12.3 Key Design Choices Summary

| Component | Choice | Reason |
|---|---|---|
| Drive system | Ackermann steering | Realistic vehicle behaviour, reduced wheel slip |
| Vision input | OpenMV camera via wired PUP cable | Wired connection complies with WRO rule 11.10; LAB threshold editor enables rapid on-site color calibration |
| Navigation strategy | Outer wall following | More stable than dual-wall centering |
| Heading correction | Gyroscope + wall reference | Compensates for cumulative angular drift |
| Sensor filtering | 50-sample majority vote (scan_colors) | Eliminates single-frame noise and lighting flicker before any avoidance decision |
| Obstacle targeting | Classification into 4 scenarios (R0, R1, G0, G1) | Each pillar configuration is handled by a dedicated, independently tuned routine |

---

## 13. Component List

The following table lists all components used in the final robot build, with specifications and notes on their role in the system.

| Component | Qty | Role in System | Notes |
|---|---|---|---|
| LEGO SPIKE Prime Hub | 1 | Main controller | Runs Pybricks firmware |
| Large angular motor | 2 | Drive (1×) + Steering (1×) | Ports F and D |
| LEGO Ultrasonic sensor | 2 | Wall and obstacle detection | Ports A (right) and C (front) |
| LEGO Color sensor | 1 | Track color detection (direction init + lap count) | Port B, mounted underneath |
| Integrated IMU / Gyroscope | 1 | Heading maintenance | Internal to SPIKE Prime Hub |
| OpenMV camera | 1 | Vision system – color blob detection | Runs PUPRemoteSensor firmware; CSI interface |
| OpenMV PUP breakout | 1 | Wired data bridge to hub | Connected to Port E via standard PUP cable |
| PUP cable | 1 | Camera-to-hub data connection | Standard LEGO SPIKE Prime cable |
| LEGO beams and connectors | — | Chassis, steering linkage, mounts | Standard SPIKE Prime element set |
| LEGO wheels (large) | 2 | Rear drive wheels | |
| LEGO wheels (small) | 2 | Front steering wheels | |

---

### 13.1 Rejected Components

During development, the following components were tested and rejected:

| Component | Reason for Rejection |
|---|---|
| Smartphone camera (AppData API) | Color detection was also prone to noise under variable venue lighting |
| HuskyLens camera | Requires ESP32 bridge; communication instability caused program crashes |
| ESP32 microcontroller | Additional communication layer introduced an unacceptable point of failure |
| Left ultrasonic sensor (Port E) | Removed when Port E was reassigned to the OpenMV breakout; outer wall following with the right sensor proved sufficient |
| Dual-wall PID centering | Sensor oscillation between walls; replaced with outer wall following |
| EV3 gyroscope | Tested for comparison; exhibited more drift than SPIKE Prime IMU |

---

## 14. Power Management and Wiring

### 14.1 Power Source

The robot is powered entirely by the LEGO SPIKE Prime rechargeable battery, which delivers a nominal voltage of 7.4 V (2-cell lithium-polymer configuration).

The SPIKE Prime Hub manages power distribution internally to all connected motors and sensors. No external power regulation circuitry is required for any component used in this build. The OpenMV camera draws power through the PUP cable from the hub; no separate battery or regulator is needed.

---

### 14.2 Voltage Operating Range

| State | Voltage Range | Behaviour |
|---|---|---|
| Optimal operation | 7.6 V – 8.1 V | Full motor torque, stable sensor readings |
| Degraded operation | 7.0 V – 7.6 V | Reduced motor response, minor sensor instability |
| Unsafe | Below 7.0 V | Startup voltage check fails; program halted |

A startup voltage check is performed every time the program launches. If the hub battery falls below the safe threshold, the program logs a warning to the Pybricks console and halts before any movement occurs.

```python
# Startup battery check
print("battery:", hub.battery.voltage())
if hub.battery.voltage() < MINIMUM_VOLTAGE:
    hub.display.text("LOW BAT")
    raise SystemExit
```

---

### 14.3 Port Assignments

| Port | Component |
|---|---|
| Port A | Right ultrasonic sensor |
| Port B | Color sensor (underneath) |
| Port C | Front ultrasonic sensor |
| Port D | Steering motor |
| Port E | OpenMV breakout (PUPRemoteHub) |
| Port F | Drive motor (COUNTERCLOCKWISE direction) |
| Internal | Gyroscope (IMU) |

---

### 14.4 Wiring Notes

All sensor and motor connections use standard LEGO SPIKE Prime cables. Cable routing is managed to avoid mechanical interference with the Ackermann steering linkage and to prevent cables from contacting the rear drive wheels.

The PUP cable connecting the OpenMV breakout (Port E) to the camera module runs along the left side of the chassis toward the front camera bracket. It is secured at two points with LEGO clips to prevent movement during sharp turns. During early testing, an unsecured cable caused intermittent data dropouts that manifested as spurious NONE readings from `get_color()` — securing the cable eliminated this issue.

OpenMV camera is connected to Spike OpenMV microcontroller using this diagram: 

   ![OpenMV Diagram](Image_SpikeOpenMV_diagram.png)
---

## 15. Sensor Calibration Procedures

### 15.1 Gyroscope Calibration

The SPIKE Prime IMU does not require manual axis calibration for heading measurement. However, the heading reference must be reset at the start of each run to establish the correct zero angle.

```python
# Reset heading to zero at program start
hub.imu.reset_heading(0)
```

During testing, we observed that a heading reset performed while the robot is still physically moving can introduce an initial offset error. The correct procedure is to allow the robot to come to a complete standstill before the program begins driving, which is naturally enforced by the button-press start sequence.

---

### 15.2 OpenMV Color Threshold Calibration

Color thresholds for red and green pillar detection are defined as LAB ranges in `main.py` on the OpenMV camera:

```python
RED   = (30, 100, 15, 127, 15, 127)
GREEN = (30, 100, -64, -8, -32, 32)
```

These values must be verified and adjusted at each new venue using the OpenMV IDE Threshold Editor (Tools → Machine Vision → Threshold Editor). The procedure is:

1. Connect the OpenMV camera to a computer via USB.
2. Run the live preview and point the camera at a red pillar under venue lighting.
3. Use the Threshold Editor to select the LAB range that covers the pillar while excluding the floor and walls.
4. Repeat for the green pillar.
5. Replace the threshold tuples in `main.py` and re-upload to the camera.

This process typically takes under five minutes and is the primary calibration task before each competition session.


----
# 16.  Video Demonstration 

 

For visual demonstration of robot in action, we provide Youtube links: 

[Open Challenge](https://youtu.be/afjnYjp07ME)

[Obstacle Challenge](https://youtu.be/AVNuD6NRbn0)

 

 # 17. Conclusion

The development of Controlled C.H.A.O.S. was, at its core, a process of learning to ask better questions. Early in the project, the central engineering question was: *how do we make the robot avoid a pillar?* By the end, that question had evolved into something more precise: *how do we make the robot respond to what it actually sees, rather than what we assume will be there?*

That shift in thinking is most clearly reflected in the transition from the initial brute-force avoidance approach to the final scenario-based classification system. The first implementation relied on pre-computed turn sequences — fixed arcs calculated from ideal geometry that assumed consistent wheel grip, stable battery voltage, and an obstacle exactly where we expected it. On a perfect surface with a fully charged battery, it worked. In practice, small deviations accumulated. A slightly wider turn, a moment of wheel slip, a pillar positioned two centimeters further than expected — any one of these was enough to send the robot off its intended path. The fundamental problem was not the geometry, but the assumption that the world would cooperate with the plan.

The solution was to stop planning for a fixed scenario and start reading the actual one. The `scan_colors()` function, which samples the camera 50 times and uses majority vote and blob area to classify the obstacle configuration into one of four named categories (R0, R1, G0, G1), gave the robot a decision framework grounded in observation rather than assumption. Each category maps to a dedicated, independently tuned avoidance routine — not because it is the most elegant architecture, but because it is the most honest one: each real situation the robot can encounter is handled by logic specifically written and tested for that situation.

This approach introduced its own discipline. When a scenario routine underperformed, the fault was localized. We knew exactly which function to tune, which test to rerun, and which measurement to verify. That traceability — from observed behaviour back to a specific piece of code — made iteration faster and more reliable than it had ever been with the single generic avoidance function.

Several other design decisions followed the same underlying principle. The trimmed-mean front sensor measurement before each corner ensures the robot anchors its turn to where the wall actually is, not where it should theoretically be after driving a fixed distance. The gyroscope heading reset after each avoidance sequence prevents small angular errors from compounding across laps. The OpenMV color threshold calibration procedure exists precisely to account for venue lighting that will never match the conditions at home.

What we are bringing to the competition is not a robot that executes a perfect plan. It is a robot that observes, classifies, and responds — and a team that understands, at every layer of the stack, why each decision was made and what it would take to change it.

That understanding is, ultimately, what this journal documents.
