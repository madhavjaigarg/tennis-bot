"""
RoboSports Launcher

Controls the flicker mechanism (Port C) that chucks a collected
ball over the barrier. A single quick motion: swing out to the
flick angle, then return to rest.
"""

from pybricks.pupdevices import Motor
from pybricks.parameters import Port

flicker_motor = Motor(Port.C)


# ============================================================
# TUNING — confirm these match your actual mechanism
# ============================================================

FLICK_SPEED = 1000          # deg/s
FLICK_ANGLE = -75            # degrees to swing out for the flick
RETURN_SPEED = 500          # deg/s, slower on the way back


# ============================================================
# LAUNCH
# ============================================================

def flick():
    """
    Run one flick: swing out to FLICK_ANGLE, then return to the
    starting position. Blocks until the motion completes.
    """

    flicker_motor.run_target(FLICK_SPEED, FLICK_ANGLE)

    flicker_motor.run_target(RETURN_SPEED, 0)
