# OpenMV AE3 camera test – scanning the track from the start

Oct 21, 2026&#32;

## Test goal

In one afternoon and without the robot, the test must answer one question: **can the OpenMV AE3, mounted 27–28 cm high in the parking lot, see all 24 possible obstacle positions clearly enough to determine their colour reliably?** If yes, the “scan at the start, drive from the map” strategy (as used by Battlepillars) can go into the build.

The test measures four things:

1. **Visibility** – can a pillar behind the inner island be seen at all (hardest case: the pillar closest to the inner wall, in the section opposite the start).
2. **Pixel count** – height and area of the blob for the farthest pillar. Our estimate is only 4–12 px of height.
3. **Colour separation** – red vs green, and red vs the magenta of the parking lot, under the lighting you have.
4. **Repeatability** – do 10 consecutive scans give the same result when the robot (mount) is placed at the start again each time.

**Pass criteria:**

| Measure | Pass | Borderline | Fail |
| --- | --- | --- | --- |
| Blob height of the farthest pillar | ≥ 6 px | 4–5 px | < 4 px or not visible |
| Blob area of the farthest pillar | ≥ 60 px | 30–59 px | < 30 px |
| Colour accuracy over 10 scans | all positions correct in all 10 scans | 1–2 errors | ≥ 3 errors |
| Magenta detected as red | never | – | at least once |

A borderline result means the idea works, but you need more height, a narrower ROI or a different tower angle.

## Equipment and preparation

The test needs a track laid out exactly as at the competition, a mount that holds the camera at the robot's height and angle, and a laptop with OpenMV IDE.

**Equipment:**

- [ ] Track (mat, outer and inner walls with height equal to 100 mm, parking lot with magenta walls) laid out according to the WRO FE 2026 rules
- [ ] Pillars 50 × 50 × 100 mm: at least 6 red and 6 green (you calibrate the 24 positions in two or three passes)
- [ ] OpenMV AE3 with firmware that has the `csi` module (OpenMV IDE offers an upgrade when connecting)
- [ ] USB-C cable long enough to reach the laptop (2–3 m) or an extension cable
- [ ] Base plate with the exact footprint of the robot (length × width), with the front end marked
- [ ] Vertical post on the base plate, where the tower axis will be on the robot; lens at **27–28 cm** above the mat
- [ ] **Positioning jig** – the same one you will use with the robot (or an exact copy)
- [ ] Protractor printed on paper (0–180° left and right), stuck under the rotating part, with a pointer on the camera mount
- [ ] Wedge or 3D-printed bracket that tilts the camera about **15° downward**
- [ ] Tape measure, spirit level (or a phone app for the angle)
- [ ] Printed diagram of the position numbering (section “Position numbering” below)
- [ ] No servo needed: you turn the camera by hand (on the robot the servo will be driven by the ESP32-S3)

**Preparing the mount:**

1. **Post position.** On the robot, the tower axis is at the middle of the width, as close as possible to the front end (the parking lot exit). Measure that distance from the robot's front edge and write it down (\_\_\_ mm). Place the post on the base plate exactly there – the test result is valid only for that point.
2. **Rotation.** The camera rotates around the vertical axis of the post. If the lens is not exactly on the axis, the lens-to-axis offset must be the same as on the future tower.
3. **Tilt.** Fix the \~15° downward tilt firmly. Any change of tilt invalidates the calibrated ROIs.
4. **Positioning jig.** Place the base plate with its front end towards the parking lot exit. The jig rests against two surfaces that are the same in both directions according to the rules: the **outer track wall** (lateral position) and the **magenta wall on the exit side** (longitudinal position). The rules forbid the robot from touching the magenta walls, so the jig leaves a gap and is removed before the start.
5. **Two directions = two camera positions.** For CW the robot faces the opposite way and the jig rests against the other magenta wall. The camera is then at a different place on the track and looks in a different direction. That is why the script has separate ROIs for `CCW` and `CW`, and why calibration is done separately for each direction.

**Preparing the laptop:**

1. Install OpenMV IDE and connect the AE3 (plug icon at the bottom left).
2. Open `ae3_test_ScanningWholeField.py`.
3. Open the **Serial Terminal** (bottom) – all results arrive there.
4. For colour thresholds use **Tools → Machine Vision → Threshold Editor**, source “Frame Buffer”, colour space **LAB**.

### Position numbering

Each position has one number from 0 to 23, always counted **in the driving direction, starting from the start section**. According to the rules, each straight section has 6 possible obstacle positions: 3 transverse lines × inner and outer side.

$$
\text{position} = 6 \cdot S + 2 \cdot L + \text{side}
$$

| Symbol | Values | Meaning |
| --- | --- | --- |
| S (section) | 0, 1, 2, 3 | 0 = start section (with the parking lot), then in the order the robot drives through them |
| L (line) | 0, 1, 2 | transverse line on the mat; 0 = the first one the robot crosses in that section |
| side | 0 = U, 1 = V | U = closer to the inner wall (Croatian *unutarnja*), V = closer to the outer wall (*vanjska*) |

Example: position 15 = 6·2 + 2·1 + 1 → section 2, middle line, outer side. The script prints the description next to the number (`S2 L1 V`), so you don't have to calculate it.

&#91;embedded content: numbering of the 24 positions · CCW direction\]

For CW, mirror the diagram left to right: S0 remains the start section, but S1 is then the section on the other side. Under the 2024 rules, positions 1, 3 and 5 (outer side of the start section) stay empty because of the parking lot. The script still measures them, but don't expect an obstacle there.

## What the script does

The script `ae3_test_ScanningWholeField.py` does the same job as Battlepillars' `cameraTower.py`: it turns the camera to several angles, takes one image at each and determines the pillar colour inside predefined rectangles (ROIs). The difference is that all processing stays on the camera and the results are printed to the Serial Terminal. You choose the mode with the `NACIN` (mode) variable at the top of the file.

The script's variable names, modes and terminal messages are in Croatian. This document keeps them exactly as they appear in the code, with the English meaning in brackets.

### Three modes

| Mode | When to use it | What it does | What you get |
| --- | --- | --- | --- |
| `LIVE` | Setting the tilt and the colour thresholds | Captures continuously, draws a crosshair in the centre, all blobs found and the ROIs for the selected angle | Image in the Frame Buffer; next to each blob its colour, height and pixel count (`R h7 p96`) |
| `KALIBRACIJA` (calibration) | Once per direction, with pillars on all positions | At each angle it captures, finds all red, green and magenta blobs and numbers them left to right | Ready-made Python code for `ROI = {}` in the terminal; you only enter the position number instead of `-1` |
| `SKEN` (scan) | The actual test, 10 repetitions | At each angle it captures and measures every ROI; between scans it waits for you to re-place the mount | A CSV line for every measurement, a 24-character string per scan, a summary of stability and accuracy |

### Code blocks

In version 3, **every line of the script has a comment** (in Croatian) saying what it does. Here is an overview by block, in the same order as in the file.

1. **Settings** – everything you change is at the top; nothing below needs touching.
   - `NACIN` and `SMJER` (mode and direction) – what we run and for which driving direction.
   - `KUTOVI = [30, 70, 110, 150]` (angles) – camera directions in degrees from “straight ahead”. For CCW the camera turns left; for CW the script turns the same numbers to the right, because the inside of the track is always on the turning side.
   - `PRAG_CRVENA`, `PRAG_ZELENA`, `PRAG_MAGENTA` (red, green, magenta threshold) – LAB colour thresholds. The values in the file are only starting values.
   - `MIN_PIKSELA = 15` (minimum pixels) – anything smaller than 15 px is treated as noise.
   - `SMIRIVANJE_MS = 800` (settling time) – wait after turning before capturing.
   - `ROI` – rows `(position, x, y, w, h)` for each direction and angle; empty until you calibrate. Each position is entered **only once**.
   - `OCEKIVANO` (expected) – the real layout as 24 characters `R`, `G` or `-`; you may write it in groups of 6 (one group = one section).
2. **Camera and button** – `init_kamera()` sets a 1280×800 colour image and waits 2 s for the exposure to settle. `zakljucaj()` (lock) then immediately locks the white balance **in all three modes**, so thresholds set in LIVE also apply in SKEN. `postavi_kut()` (set angle) prints an instruction and waits for the SW button; while it waits, the Frame Buffer shows a live image with that angle's ROIs. Blue LED = the camera is waiting for you, green = capturing, don't touch the mount.
3. **Measurement** – `nadji_mrlje()` (find blobs) searches for red, green and magenta **each separately**, so blobs of different colours are never merged into one. `mjeri_roi()` (measure ROI) sums the pixels per colour within one ROI: the colour with more pixels wins, confidence is its share of red + green (1.00 = clean), and magenta is only counted as a warning.
4. **Modes** – `nacin_live()`, `nacin_kalibracija()` and `nacin_sken()` from the table above. In SKEN, drawing happens only after measuring, so the drawn lines don't spoil the colours.
5. **Start** – runs the selected mode.

### Output in SKEN mode

Each measurement is one line starting with `CSV;`. Copy them into a spreadsheet (semicolon as separator):

<!-- ```csv
CSV;sken;mjesto;opis;kut;boja;px;w;h;sigurnost;magenta_px;ekspozicija
CSV;1;15;S2 L1 V;0;R;80;18;6;1.00;0;exp=8200us gain=6.0dB
``` -->


CSV-cro|sken|mjesto|opis|kut|boja|px|w|h|sigurnost|magenta_px|ekspozicija
--------|----|------|----|---|----|--|-|-|---------|----------|-----------
CSV-eng (translation)|scan|position|description|angle|color|px|w|h|reliability|magenta_px|exposure
CSV-val|1|15|S2 L1 V|0|R|80|18|6|1.00|0|exp=8200us gain=6.0dB


Columns: scan; position; description; angle; colour; pixels; width; height; confidence; magenta pixels; exposure. The `h` column is the pillar height in pixels – the key number of the test. After each scan comes a line like `SKEN  3: --R--- G----- -R--G- ----R-  pogresaka: 0` (*pogrešaka* = errors; four groups = four sections; `?` = a position without an ROI). At the end there is a summary per position (`mjesto 15 (S2 L1 V): RRRRRRRRRR  stabilno ocekivano R` = position, stable, expected), the overall accuracy and the smallest pillar height measured.

**Verified:** version 3 runs on the AE3 with OpenMV v4.8.1 (MicroPython 1.26) without the Frame Buffer freezing. KALIBRACIJA and SKEN were also checked in a simulation with a mock camera and button. If any call raises an error, copy the full terminal output and report it.

## Test procedure

The test has five phases and takes about 2–3 hours per direction. Do the whole procedure for CCW first, and CW only once CCW passes.

### Phase 1 – Setup and tilt (LIVE, \~20 min)

1. Place the base plate with the camera in the parking lot using the jig (front end towards the exit), then remove the jig.
2. Turn the camera by hand to **angle 0** (30° left for CCW). The script locks the white balance in this direction at start-up, so every run must start the same way.
3. In the script set `NACIN = "LIVE"`, `SMJER = "CCW"`, `LIVE_KUT_INDEKS = 0` and run it (green button at the bottom left).
4. Check the output `Rezolucija: 1280 x 800` (resolution). If it says something else, write it down.
5. Place pillars on all positions. For the hardest case, put the pillars in section S2 (opposite the start) on the **inner side** (positions 12, 14, 16).
6. Turn the camera by hand towards S2 and adjust the tilt until:
   - the pillars in S2 are in the upper quarter of the image, but not right at the edge;
   - the pillars in S0 at positions 0, 2 and 4 (camera turned towards them) are still fully in the image.
7. Write down the tilt (spirit level or app) and the lens height above the mat. From now on, don't touch them.

### Phase 2 – Colour thresholds (LIVE + Threshold Editor, \~30 min)

1. While `LIVE` is running, open **Tools → Machine Vision → Threshold Editor**, source **Frame Buffer**.
2. Move the L, A, B sliders until **only the red pillars** stay white in the binary image. Copy the six numbers into `PRAG_CRVENA`.
3. Do the same for the green pillars (`PRAG_ZELENA`) and for the magenta parking-lot elements (`PRAG_MAGENTA`).
4. Key point: the red threshold **must not** capture magenta. If they overlap, narrow the B range of red (magenta has a lower B value than red).
5. Run `LIVE` again. Every blob in the image must have exactly one colour. Write down `h` and `p` for the farthest pillar – that is the first real test result.
6. If you can, repeat the check with the ceiling light off and with daylight from the window. The competition hall will not have your lighting.

### Phase 3 – ROI calibration (KALIBRACIJA, \~40 min)

1. Place pillars of known colour on as many positions at once as possible. With 12 pillars, work in two passes: the first pass covers all **inner** positions (even numbers), the second all **outer** positions (odd numbers, except 1, 3 and 5 next to the parking lot).
2. Place the base plate with the jig and turn the camera to angle 0.
3. Set `NACIN = "KALIBRACIJA"` and run.
4. For each of the four angles: when the terminal prints e.g. `Kut 1: okreni kameru na 70 st. lijevo` (angle 1: turn the camera 70° left), turn the camera using the protractor, hold the mount with one hand and with the other **gently** press and release the SW button. The capture starts only when you release the button.
5. After the capture, look at the Frame Buffer: each blob has a label `#0`, `#1`… (left to right). The terminal has a line for each, like `(-1, 612, 248, 46, 32),  #3 G px=96 w=22 h=8`.
6. Using the printed diagram, work out which position each label `#k` belongs to. Write it on paper, then press SW for the next angle.
7. Copy the printed blocks into `ROI["CCW"]` and in every line replace `-1` with the position number (0–23). Delete lines that are not pillars. Magenta lines are already commented out.
8. If the same position is visible at two angles, keep it **only at the angle where its `h` is larger**. Each position may appear in `ROI` only once (SKEN checks this and stops with an error).
9. After the second pass, add its lines to the same `ROI["CCW"]`.
10. Check: `NACIN = "LIVE"`, `LIVE_KUT_INDEKS` set to 0, 1, 2, 3 in turn (turn the camera to the matching angle), and verify that the yellow ROIs sit on the pillars.

**Why a 12 px margin:** `ROI_RUB = 12` (ROI margin) adds space around the pillar, because the robot shifts by a millimetre or two each time it is placed. If in phase 4 a position occasionally comes out as `-`, enlarge that ROI's margin by hand. If it captures a neighbouring pillar (confidence below 0.90), shrink it.

### Phase 4 – Repeatable scan (SKEN, \~40 min)

1. Arrange a **new** pillar layout, different from the calibration one: at most two pillars per section, red and green, with empty positions.
2. Enter it in `OCEKIVANO` in groups of 6, e.g. `"--R--- G----- -R--G- ----R-"`: the first character is position 0, the last is position 23.
3. Set `NACIN = "SKEN"` and `BROJ_SKENOVA = 10` (number of scans). Place the base plate with the jig, turn the camera to angle 0, and run.
4. For each scan, go through the four angles with the SW button.
5. Between scans, when the terminal prints `izvadi nosac iz starta, vrati ga uz postavljac` (take the mount out of the start, put it back against the jig), **lift the base plate and place it again with the jig**, remove the jig and turn the camera back to angle 0. This tests exactly what you will do with the robot.
6. At the end, copy the whole terminal output into a file (right-click → Save).

### Phase 5 – Hard cases (\~30 min)

Repeat 3–5 scans for each of these layouts:

- a red pillar at the position next to the magenta parking-lot element,
- all pillars in the inner position in the section opposite the start,
- two pillars in the same section, one behind the other as seen from the camera,
- different lighting (light off, sunlight from the window).

After that, repeat the whole procedure with `SMJER = "CW"`.

## Recording and evaluating results

The decision comes from a single table: for each position, the smallest blob height and the number of errors in 10 scans. Take the values from the CSV lines and the summary.

### Test log (CCW direction)

Lens height: \_\_\_ cm · Tilt: \_\_\_° · Resolution: \_\_\_ · Lighting: \_\_\_

| Position | Description | Angle | Min h (px) | Min area (px) | Lowest confidence | Errors / 10 | Magenta? |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 0 | S0 L0 U |  |  |  |  |  |  |
| 1 | S0 L0 V (parking) |  |  |  |  |  |  |
| 2 | S0 L1 U |  |  |  |  |  |  |
| 3 | S0 L1 V (parking) |  |  |  |  |  |  |
| 4 | S0 L2 U |  |  |  |  |  |  |
| 5 | S0 L2 V (parking) |  |  |  |  |  |  |
| 6 | S1 L0 U |  |  |  |  |  |  |
| 7 | S1 L0 V |  |  |  |  |  |  |
| 8 | S1 L1 U |  |  |  |  |  |  |
| 9 | S1 L1 V |  |  |  |  |  |  |
| 10 | S1 L2 U |  |  |  |  |  |  |
| 11 | S1 L2 V |  |  |  |  |  |  |
| 12 | S2 L0 U |  |  |  |  |  |  |
| 13 | S2 L0 V |  |  |  |  |  |  |
| 14 | S2 L1 U |  |  |  |  |  |  |
| 15 | S2 L1 V |  |  |  |  |  |  |
| 16 | S2 L2 U |  |  |  |  |  |  |
| 17 | S2 L2 V |  |  |  |  |  |  |
| 18 | S3 L0 U |  |  |  |  |  |  |
| 19 | S3 L0 V |  |  |  |  |  |  |
| 20 | S3 L1 U |  |  |  |  |  |  |
| 21 | S3 L1 V |  |  |  |  |  |  |
| 22 | S3 L2 U |  |  |  |  |  |  |
| 23 | S3 L2 V |  |  |  |  |  |  |

### How to read the results

- **Confidence < 0.90** at a position means the ROI also captures another colour (a neighbouring pillar or the background). Narrow the ROI.
- **A blob appears, but h varies by more than 2 px** between scans: the mount shifts when it is placed. It will be the same on the robot, so the ROI needs a margin.
- **Position reads “-” although there is a pillar** – the pillar is outside the ROI (shift during placement) or fell below `MIN_PIKSELA`. Check the CSV for that scan.
- **Magenta\_px > 0 next to red** – the red threshold captures the parking-lot element. Go back to phase 2.
- **Exposure** in the CSV differs a lot between angles – one angle looks into the light. Try `ZAKLJUCAJ_EKSPOZICIJU = True` (lock exposure) and compare.

### Typical problems

| Symptom | Likely cause | Fix |
| --- | --- | --- |
| `AttributeError` at `csi.HD` or `rgb_gain_db` | Old firmware | Upgrade the firmware through OpenMV IDE |
| Warning “balans bijele se ne može zaključati” (white balance cannot be locked) | The sensor does not support manual white balance | Set `ZAKLJUCAJ_BALANS = False` and compare results |
| SW button does not respond | The pin has a different name in your firmware | Set `ODBROJAVANJE_S = 5` (5 s countdown) |
| Frame Buffer does not show ROIs in KALIBRACIJA | `img.flush()` not supported | Set `SPREMI_SLIKE = True` and look at the JPEGs on the camera's drive |
| `AttributeError: 'NoneType' object has no attribute 'width'` or `ImportError: PWM` | Old version of the script | Use version 3 of the script (verified on OpenMV 4.8.1) |
| Red and magenta keep getting mixed up | Thresholds too wide on the B axis | Narrow B; if that fails, keep the magenta part of the image out of the ROI |

### Decision after the test

- **All positions pass** the criteria from the first section → the start-scan strategy goes into the robot design, with these angles and this tilt.
- **1–2 positions borderline** → try a different tower angle for those positions or add a fifth angle before giving up.
- **The farthest positions fail** → the camera must go higher or closer to the centre of the track, or those positions must be checked while driving (hybrid strategy).

## Sources

- [WRO 2026 Future Engineers – General Rules](https://wro-association.org/wp-content/uploads/WRO-2026-Future-Engineers-Self-Driving-Cars-General-Rules.pdf)
- [WRO 2026 Future Engineers - Game Mat](https://wro-association.org/wp-content/uploads/WRO-2026_FutureEngineers_Playfield.pdf)
- [OpenMV AE3 – product page](https://openmv.io/products/openmv-ae3)
- [OpenMV AE3 – documentation](https://docs.openmv.io/openmvcam/quickref/openmv-ae3.html)
- [OpenMV `csi` module](https://docs.openmv.io/library/omv.csi.html)
- [Battlepillars/Wro26 GitHub page](https://github.com/Battlepillars/Wro26)
