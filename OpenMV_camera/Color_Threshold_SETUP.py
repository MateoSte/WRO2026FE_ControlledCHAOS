# ============================================================
# Color Threshold Tuning Helper
# ------------------------------------------------------------
# Two modes, switched by the TEST_MODE flag below:
#
# MODE 1 - Calibration (TEST_MODE = False):
#   Just streams a live camera feed. Use this together with
#   OpenMV IDE's built-in Threshold Editor to find your LAB values:
#     1. Run this script with Frame Buffer ENABLED (not disabled).
#     2. In the menu: Tools -> Machine Vision -> Threshold Editor.
#     3. Point the camera at your object under real lighting.
#     4. Drag the L/A/B sliders until ONLY your object shows white
#        in the preview (everything else black).
#     5. Copy the 6-number tuple it shows you into RED_THRESHOLD
#        or GREEN_THRESHOLD below.
#     6. Repeat for the other color.
#
# MODE 2 - Live test (TEST_MODE = True):
#   Runs actual blob detection using the thresholds below and
#   draws a box around whatever it finds, so you can visually
#   confirm detection works before wiring anything else back in.
# ============================================================

import sensor
import time

TEST_MODE = False  # False = just stream video for Threshold Editor
                    # True  = test detection live with thresholds below

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=2000)

# Paste your calibrated values here after using the Threshold Editor
RED_THRESHOLD = [(18, 40, 10, 30, 5, 24)]
GREEN_THRESHOLD = [(30, 100, -64, -8, -32, 32)]

def biggest(blobs):
    best = None
    for b in blobs:
        if best is None or b.pixels > best.pixels:
            best = b
    return best

while True:
    img = sensor.snapshot()

    if TEST_MODE:
        time.sleep_ms(50)
        red = biggest(img.find_blobs(RED_THRESHOLD, pixels_threshold=100, area_threshold=100))
        green = biggest(img.find_blobs(GREEN_THRESHOLD, pixels_threshold=100, area_threshold=100))

        if red is not None:
            img.draw_rectangle(red.rect, color=(255, 0, 0), thickness=2)
            print("Red detected - w:", red.w, "h:", red.h)
        elif green is not None:
            img.draw_rectangle(green.rect, color=(0, 255, 0), thickness=2)
            print("Green detected - w:", green.w, "h:", green.h)
        else:
            print("Nothing detected")
    # else: just streaming frames for the Threshold Editor, no processing
