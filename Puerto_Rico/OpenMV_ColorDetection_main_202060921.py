# ============================================================
# AE3 <-> ESP32-S3 Color + Distance Detector
# ------------------------------------------------------------
# Detects red/green blobs, estimates distance to the largest one,
# and exposes the result to an ESP32 (I2C master) over I2C via
# the Qwiic connector, using an I2CTarget "shared memory" buffer.
#
# result[0] = detected color: 'R', 'G', or 'N' (none)
# result[1] = estimated distance in cm, capped at 255 (0-255 range,
#             since it's stored as a single byte)
#
# ESP32 side reads this with Wire.requestFrom(0x12, 2).
# ============================================================

import sensor
import time
from machine import I2CTarget

# --- Camera setup ---
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=2000)  # let auto-exposure/white-balance settle

# --- I2C target (slave) setup ---
# This buffer is shared memory: the ESP32 (I2C master) can read it
# directly at any time. We just keep updating result[] in the loop.
result = bytearray(2)
result[0] = ord('N')
result[1] = 255
i2c_target = I2CTarget(1, addr=0x12, mem=result)  # bus 1 = Qwiic connector's I2C1

# --- Color thresholds (LAB colorspace) ---
# Tune these with Tools -> Machine Vision -> Threshold Editor in OpenMV IDE
# for your actual lighting conditions and object colors.
red_threshold = [(18, 40, 10, 30, 5, 24)]
green_threshold = [(30, 100, -64, -8, -32, 32)]

# --- Distance estimation constants ---
# REAL_WIDTH_CM: measure your actual target object's width with a ruler.
# FOCAL: calibrated once using: focal = (pixel_width_at_known_dist * known_dist) / real_width
REAL_WIDTH_CM = 5.0
FOCAL = 480  # replace with your own calibrated value

def get_distance(pixel_width):
    """Estimate distance (cm) from an object's pixel width, using the
    pinhole camera formula: distance = (real_width * focal) / pixel_width.
    Clamped to 255 since it's stored in a single byte."""
    d = REAL_WIDTH_CM * FOCAL / pixel_width
    return min(255, int(d))

def biggest(blobs):
    """Return the largest blob (by pixel count) from a list, or None
    if the list is empty. Used to ignore small/noisy detections."""
    best = None
    for b in blobs:
        if best is None or b.pixels > best.pixels:
            best = b
    return best

# NOTE: On this firmware version (OpenMV MicroPython 1.28 / IDE 5.0.0),
# Blob fields (rect, w, h, pixels, cx, cy, etc.) are plain ATTRIBUTES,
# not method calls. Use `blob.rect`, NOT `blob.rect()` -- calling them
# like a function raises "TypeError: 'tuple' object isn't callable".

while True:
    time.sleep_ms(50)  # throttle loop; also gives I2CTarget/camera time to coexist
    img = sensor.snapshot()

    red = biggest(img.find_blobs(red_threshold, pixels_threshold=100, area_threshold=100))
    green = biggest(img.find_blobs(green_threshold, pixels_threshold=100, area_threshold=100))

    if red is not None:
        img.draw_rectangle(red.rect, color=(255, 0, 0), thickness=2)
        result[1] = get_distance(red.w)
        result[0] = 1
        print("Red - w:", red.w, "dist:", result[1])
    elif green is not None:
        img.draw_rectangle(green.rect, color=(0, 255, 0), thickness=2)
        result[1] = get_distance(green.w)
        result[0] = 2
        print("Green - w:", green.w, "dist:", result[1])
    else:
        result[0] = 0
        result[1] = 255
        print("Nothing detected")
