import sensor, time
from machine import I2CTarget

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=2000)

result = bytearray(1)
result[0] = ord('N')
i2c_target = I2CTarget(1, addr=0x12, mem=result)

red_threshold = [(18, 40, 10, 30, 5, 24)]
green_threshold = [(30, 100, -64, -8, -32, 32)]

last_check = time.ticks_ms()
check_interval = 1000  # milliseconds

while True:
    img = sensor.snapshot()  # runs every loop, keeps preview live

    if time.ticks_diff(time.ticks_ms(), last_check) >= check_interval:
        last_check = time.ticks_ms()

        red_blobs = img.find_blobs(red_threshold, pixels_threshold=100, area_threshold=100)
        green_blobs = img.find_blobs(green_threshold, pixels_threshold=100, area_threshold=100)

        if red_blobs:
            b = red_blobs[0]
            img.draw_rectangle(b.rect(), color=(255, 0, 0))
            print("Red detected - width:", b.w(), "height:", b.h())
            result[0] = ord('R')
        elif green_blobs:
            b = green_blobs[0]
            img.draw_rectangle(b.rect(), color=(0, 255, 0))
            print("Green detected - width:", b.w(), "height:", b.h())
            result[0] = ord('G')
        else:
            print("Nothing detected")
            result[0] = ord('N')
