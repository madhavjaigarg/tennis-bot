"""
RoboSports Field Map

Defines this robot's operating area only: our half of the field,
from our starting position up to the barrier. We never cross
into the opponent's half, so nothing on the other side of the
barrier is tracked here.

Coordinate convention (same as odometry.py / navigation.py):

    heading 0   = +X
    heading -90 = +Y
    heading 180 = -X
    heading 90  = -Y

Origin:
    (0, 0) is THIS robot's starting position, set by
    rafael_calibration.py via odometry.reset_position(0, 0, ...).

    +X = forward, from our starting wall toward the barrier.
    +Y = sideways, across the width of our half.

Boundary:
    BARRIER_X marks the edge of our half. Touching the
    opponent's side is an instant match loss (rules 6.29 /
    6.32.2), so is_past_barrier() checks a safety margin rather
    than the bare coordinate.
"""

from rafael_constants import FIELD_WIDTH, FIELD_HEIGHT
import rafael_odometry


# ============================================================
# FIELD SIZE  (our half only)
# ============================================================

BARRIER_X = FIELD_WIDTH / 2.0      # 1181 mm — edge of our half

FIELD_X_MIN = 0.0
FIELD_X_MAX = BARRIER_X

FIELD_Y_MIN = 0.0
FIELD_Y_MAX = FIELD_HEIGHT         # 1143 mm — full width of our half


# How far back from the barrier we stay, so odometry drift
# doesn't cause an accidental cross into the opponent's half.
BARRIER_SAFETY_MARGIN_MM = 30.0


# ============================================================
# STARTING POSITION
# ============================================================

START_X = 0.0
START_Y = 0.0
START_HEADING = 0.0


# ============================================================
# CALIBRATION
# ============================================================

def calibrate():
    """
    Zero the odometry at our starting position.

    Call this once, right before the match starts, after the
    robot has been manually placed in its starting zone, front
    edge touching the starting wall.
    """

    rafael_odometry.reset_position(
        START_X,
        START_Y,
        START_HEADING
    )

    print("CALIBRATED")
    print_field_map()


# ============================================================
# BOUNDS CHECKING
# ============================================================

def is_within_field(x, y):
    """
    True if (x, y) is inside our half of the field.
    """

    return (
        FIELD_X_MIN <= x <= FIELD_X_MAX and
        FIELD_Y_MIN <= y <= FIELD_Y_MAX
    )


def is_past_barrier(x):
    """
    True if x has crossed, or is within the safety margin of,
    the barrier into the opponent's half.
    """

    return x >= (BARRIER_X - BARRIER_SAFETY_MARGIN_MM)


def clamp_to_field(x, y):
    """
    Clamp (x, y) so it lies within our half.
    """

    clamped_x = min(max(x, FIELD_X_MIN), FIELD_X_MAX)
    clamped_y = min(max(y, FIELD_Y_MIN), FIELD_Y_MAX)

    return clamped_x, clamped_y


# ============================================================
# DEBUGGING
# ============================================================

def print_field_map():
    """
    Print our field bounds and starting position.
    """

    print("OUR HALF")
    print("X:", FIELD_X_MIN, "to", FIELD_X_MAX, "(barrier at", BARRIER_X, ")")
    print("Y:", FIELD_Y_MIN, "to", FIELD_Y_MAX)
    print("START:", START_X, START_Y, START_HEADING)
