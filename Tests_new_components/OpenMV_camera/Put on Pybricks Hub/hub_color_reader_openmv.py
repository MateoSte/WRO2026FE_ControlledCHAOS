from pybricks.parameters import Port
from pupremote_hub import PUPRemoteHub

pr = PUPRemoteHub(Port.B, max_packet_size=16)  # promijeni na port gdje je breakout spojen
pr.add_channel('color', to_hub_fmt="bhhhh")

COLOR_NAMES = {0: "NONE", 1: "RED", 2: "GREEN"}

def get_color():
    color_id, x, y, w, h = pr.call('color')
    return (COLOR_NAMES.get(color_id, "NONE"), x, y, w, h)

while True:
    print(get_color())