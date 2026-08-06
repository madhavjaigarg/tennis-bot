"""
Math helper functions.
"""

from math import atan2, cos, sin, radians, degrees, sqrt


def distance(x1, y1, x2, y2):
    return sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)


def angle_to(x1, y1, x2, y2):
    return degrees(atan2(y2 - y1, x2 - x1))


def normalize(angle):
    while angle > 180:
        angle -= 360

    while angle < -180:
        angle += 360

    return angle


def clamp(value, minimum, maximum):
    return max(minimum, min(value, maximum))


def polar_to_cartesian(distance_mm, angle_deg):

    x = distance_mm * cos(radians(angle_deg))
    y = distance_mm * sin(radians(angle_deg))

    return x, y
