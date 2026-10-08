import sensor
from machine import I2CTarget

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=2000)

# result[0] = color ('R', 'G', 'N'), result[1] = distance in cm (0-255)
result = bytearray(2)
result[0] = ord('N')
result[1] = 255
i2c_target = I2CTarget(1, addr=0x12, mem=result)

red_threshold = [(18, 40, 10, 30, 5, 24)]
green_threshold = [(30, 100, -64, -8, -32, 32)]

REAL_WIDTH_CM = 5.0   # measure your object with a ruler
FOCAL = 480           # replace with your calibrated value

def get_distance(pixel_width):
    d = REAL_WIDTH_CM * FOCAL / pixel_width
    return min(255, int(d))

def biggest(blobs):
    best = None
    for b in blobs:
        if best is None or b.pixels() > best.pixels():
            best = b
    return best   # None if the list is empty
while True:
    img = sensor.snapshot()
    red = biggest(img.find_blobs(red_threshold, pixels_threshold=100, area_threshold=100))
    green = biggest(img.find_blobs(green_threshold, pixels_threshold=100, area_threshold=100))

    if red is not None:
        img.draw_rectangle(red.rect(), color=(255, 0, 0), thickness=2)
        result[1] = get_distance(red.w())
        result[0] = ord('R')
        print("Red - w:", red.w(), "dist:", result[1])
    elif green is not None:
        img.draw_rectangle(green.rect(), color=(0, 255, 0), thickness=2)
        result[1] = get_distance(green.w())
        result[0] = ord('G')
        print("Green - w:", green.w(), "dist:", result[1])
    else:
        result[0] = ord('N')
        result[1] = 255
        print("Nothing detected")
