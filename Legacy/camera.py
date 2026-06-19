from pybricks.hubs import PrimeHub
from pybricks.tools import AppData
from pybricks.tools import wait
from pybricks.parameters import Color
from ustruct import unpack

hub = PrimeHub()

# Only mode 0 exists for the color tracker
app = AppData([(0, 4)])

RED_HUE   = bytes([Color.RED.h // 2 if Color.RED.h > 0 else 1])
GREEN_HUE = bytes([Color.GREEN.h // 2])  # 60

red_data   = (0, 0, 0, 0)
green_data = (0, 0, 0, 0)

while True:
    # --- Read red ---
    app.configure(0, 0, RED_HUE)
    wait(500)  # give the phone time to switch and send a fresh frame
    red_data = unpack("BBBB", app.get_bytes(0))

    # --- Read green ---
    app.configure(0, 0, GREEN_HUE)
    wait(500)
    green_data = unpack("BBBB", app.get_bytes(0))

    print("Red:",   red_data)
    print("Green:", green_data)