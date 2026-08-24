"""
roger_geo_v2.py

New file — roger_geo.py is left untouched. Import from this one
(roger_odo_v2 / roger_motion_v2 / roger_robot_v2 already do) to
get the fixes below; keep using roger_geo.py + the old odo/motion/
robot files together if you want the original behavior.

Changes vs roger_geo.py:
  - added clamp() and drive-correction gains (DRIVE_KP/KI/KD),
    needed by roger_motion_v2.py's heading-hold correction.

NOT changed: MM_PER_MOTOR_DEGREE is still 0.915, same as the
original. For a direct-drive 81mm wheel with no gearing, the
geometric value would be WHEEL_CIRCUMFERENCE / 360 = 0.707 — about
30% lower. That's either a real gear ratio on this drivetrain, or
a stale/mis-measured constant. Left as-is rather than guessed at —
see the TESTING notes for how to check this on the mat.
"""

import math

WHEEL_DIAMETER = 81
AXLE_TRACK = 142

WHEEL_CIRCUMFERENCE = math.pi * WHEEL_DIAMETER
MM_PER_MOTOR_DEGREE = 0.915

# Straight-line heading-hold correction for Motion.drive_distance()
# in roger_motion_v2.py. Start conservative, tune on the mat.
DRIVE_KP = 0.07
DRIVE_KI = 0.00
DRIVE_KD = 0.03


def motor_degrees_to_mm(degrees):
    return degrees * MM_PER_MOTOR_DEGREE

def mm_to_motor_degrees(mm):
    return mm / MM_PER_MOTOR_DEGREE

def normalize_angle(angle):
    while angle > 180:
        angle -= 360

    while angle <= -180:
        angle += 360

    return angle

def clamp(value, minimum, maximum):
    return max(minimum, min(value, maximum))
