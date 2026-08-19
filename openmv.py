"""
openmv_ball_server.py

Runs ON THE OPENMV H7 PLUS (not the hub). Upload this to the
camera as main.py, replacing the standalone print-based script.

Same blob detection as before, but exposed over UART via
Anton's Mindstorms uRemote library so the Pybricks hub can pull
detections with ur.call("ball").

uRemote can't send floats yet, so everything returned here is
already an int (cx, cy, pixels are ints from find_blobs anyway).
"""

import sensor
import time

from uremote import uRemote


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
# uRemote SERVER
# ============================================================

ur = uRemote()


def ball():
    """
    Handler for the "ball" command. Runs one frame, returns the
    biggest orange blob found (or a "not found" reply).

    Returns:
        (found, x, y, area)

        found = 1 or 0
        x, y  = blob center in pixels (0 if not found)
        area  = blob size in pixels (0 if not found)
    """

    clock.tick()

    img = sensor.snapshot()

    blobs = img.find_blobs(
        [orange_threshold],
        pixels_threshold=100,
        area_threshold=10,
        merge=True
    )

    if not blobs:
        return 0, 0, 0, 0

    biggest = max(blobs, key=lambda b: b.pixels)

    img.draw_rectangle(biggest.rect)
    img.draw_cross((biggest.cx, biggest.cy))

    return 1, biggest.cx, biggest.cy, biggest.pixels


# ============================================================
# SERVER LOOP
# ============================================================

while True:
    ur.process()
