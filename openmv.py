"""
openmv_ball_server.py

Runs ON THE OPENMV H7 PLUS (not the hub). Upload this to the
camera as main.py, replacing the standalone print-based script.

No RPC library needed on this side either — just streams ball
detections out over UART as plain ASCII lines, one per frame:

    "<found>,<x>,<y>,<area>\n"

    found = 1 or 0
    x, y  = blob center in pixels (0 if not found)
    area  = blob size in pixels (0 if not found)

The hub reads these continuously with rafael_camera.py.
"""

import sensor
import time

from machine import UART


# ============================================================
# CAMERA SETUP
# ============================================================

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=2000)

clock = time.clock()

orange_threshold = (10, 100, 20, 127, 0, 100)


# ============================================================
# UART SETUP
# ============================================================

# TODO: confirm which UART bus number is wired to the hub on
# your OpenMV H7 Plus (check your board's pinout for the pins
# you actually soldered/connected TX/RX to).
uart = UART(3, 115200)


# ============================================================
# MAIN LOOP
# ============================================================

while True:

    clock.tick()

    img = sensor.snapshot()

    blobs = img.find_blobs(
        [orange_threshold],
        pixels_threshold=100,
        area_threshold=10,
        merge=True
    )

    if blobs:

        biggest = max(blobs, key=lambda b: b.pixels)

        img.draw_rectangle(biggest.rect)
        img.draw_cross((biggest.cx, biggest.cy))

        found, x, y, area = 1, biggest.cx, biggest.cy, biggest.pixels

    else:
        found, x, y, area = 0, 0, 0, 0

    line = "{},{},{},{}\n".format(found, x, y, area)

    uart.write(line)
