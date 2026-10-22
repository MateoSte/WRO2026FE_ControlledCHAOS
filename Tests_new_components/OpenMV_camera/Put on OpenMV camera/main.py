import csi
from pupremote import PUPRemoteSensor

cam = csi.CSI()
cam.reset()
cam.pixformat(csi.RGB565)
cam.framesize(csi.QVGA)  # 320x240
cam.snapshot(time=2000)  # neka se ekspozicija stabilizira (zamjena za skip_frames)

# LAB pragovi za crvenu i zelenu.
# Kalibriraj u OpenMV IDE: Tools > Machine Vision > Threshold Editor,
# pa zamijeni ove vrijednosti svojim.
RED = (30, 100, 15, 127, 15, 127)
GREEN = (30, 100, -64, -8, -32, 32)

# power=True ako koristis OpenMV RT seriju, False je obicno OK za H7/H7 Plus
p = PUPRemoteSensor(power=False, max_packet_size=16)
p.add_channel('color', to_hub_fmt="bhhhh")  # ID boje (byte) + x,y,w,h (4x signed short)

while True:
    img = cam.snapshot()
    color_id, x, y, w, h = 0, 0, 0, 0, 0

    reds = img.find_blobs([RED], pixels_threshold=200, area_threshold=200, merge=True)
    greens = img.find_blobs([GREEN], pixels_threshold=200, area_threshold=200, merge=True)

    if reds:
        b = max(reds, key=lambda blob: blob.pixels())
        color_id, x, y, w, h = 1, b.cx(), b.cy(), b.w(), b.h()
    elif greens:
        b = max(greens, key=lambda blob: blob.pixels())
        color_id, x, y, w, h = 2, b.cx(), b.cy(), b.w(), b.h()

    p.update_channel('color', color_id, x, y, w, h)
    p.process()
