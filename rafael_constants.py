"""
Robot and field constants
RoboSports Navigation Framework
"""

from math import pi

# ------------------------
# Robot Geometry
# ------------------------

WHEEL_DIAMETER = 85.00         # mm
WHEEL_RADIUS = WHEEL_DIAMETER / 2

WHEEL_CIRCUMFERENCE = pi * WHEEL_DIAMETER

TRACK_WIDTH = 142.0            # mm

# ------------------------
# Drive Motors
# ------------------------

LEFT_PORT = "E"
RIGHT_PORT = "A"

# ------------------------
# Sensors
# ------------------------

DISTANCE_PORT = "D"

# ------------------------
# Robot Limits
# ------------------------

MAX_SPEED = 700        # deg/s

CRUISE_SPEED = 550

TURN_SPEED = 350

MAX_ACCEL = 900

# ------------------------
# PID

DRIVE_KP = 0.07
DRIVE_KI = 0.00
DRIVE_KD = 0.03

TURN_KP = 1.0
TURN_KI = 0.0
TURN_KD = 0.5

# ------------------------
# Field

FIELD_WIDTH = 2362.0
FIELD_HEIGHT = 1143.0

# Goal coordinates
# (We'll fill these after we define the field orientation.)

GOAL_X = 0
GOAL_Y = 0
