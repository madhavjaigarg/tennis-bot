import sensor
import time

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=2000)
clock = time.clock()

orange_threshold = (10, 100, 20, 127, 0, 100)

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

        x = biggest.cx
        y = biggest.cy
        area = biggest.pixels
        width = biggest.w
        height = biggest.h

        # Firmware 5.0
        img.draw_rectangle(biggest.rect)
        img.draw_cross((x, y))

        if x < 106:
            position = "LEFT"
        elif x > 213:
            position = "RIGHT"
        else:
            position = "CENTER"

        print(
            "x:", x,
            "y:", y,
            "area:", area,
            position
        )

    else:
        print("NO BALL")

    print("FPS:", clock.fps())

