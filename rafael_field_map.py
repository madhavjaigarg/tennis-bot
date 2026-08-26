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

from pybricks.tools import StopWatch, wait

from rafael_constants import FIELD_WIDTH, FIELD_HEIGHT
from rafael_robot import touch_sensor
from rafael_motion import set_motor_speed, stop, MIN_DRIVE_SPEED, turn_to, MAX_SPEED, drive_distance
import rafael_odometry
import rafael_navigation as navigation


# ============================================================
# FIELD SIZE  (our half only)
# ============================================================

BARRIER_X = 950      # 950 mm — calbrated for our bot

FIELD_X_MIN = 0.0
FIELD_X_MAX = BARRIER_X

FIELD_Y_MIN = -330.0
FIELD_Y_MAX = 210.0         # calibrated for bot


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


def return_home_and_calibrate(max_time_ms=5000):
    """
    Navigate near the left wall, turn 180, and hug the wall 
    until the front sensor hits the bottom starting wall.
    """
    # 1. Drive to a safe Y-coordinate just off the left wall
    #current_x, current_y, _ = rafael_odometry.get_position()
    turn_to(-90)
    while not touch_sensor.pressed():
        set_motor_speed(MAX_SPEED, MAX_SPEED)
    drive_distance(-18)
    # 2. Turn to face the starting wall (-X direction)
    turn_to(180)
    
    # 3. Creep forward while pressing into the field's left wall
    contact_made = _creep_and_hug_wall(max_time_ms)
    
    # 4. Re-zero odometry at the known bottom-left corner
    # (X=0, Y=Left Wall, Heading=180)
    rafael_odometry.reset_position(0, FIELD_Y_MAX, 180)
    rafael_odometry.update()
    print("CALIBRATED AT BOTTOM-LEFT CORNER")
    
    return contact_made

def _creep_and_hug_wall(max_time_ms=5000):
    """
    Drive forward with a deliberate drift to stay pressed against 
    a wall, bypassing the standard check_wall_collision logic.
    """
    watch = StopWatch()
    
    # Because the robot faces 180, the +Y left wall is on its right.
    # Left motor gets a speed boost to veer right into the wall.
    set_motor_speed(MAX_SPEED, MAX_SPEED-75)
    
    while not touch_sensor.pressed():
        if watch.time() > max_time_ms:
            break
        wait(10)
        
    stop()
    return touch_sensor.pressed()


def _creep_forward_to_wall(max_time_ms=3000):
    """
    Creep slowly forward, front-first, until the touch sensor
    detects contact with the starting wall, or max_time_ms
    elapses (safety timeout in case contact never registers).

    Returns True if contact was made, False if it timed out.
    """

    watch = StopWatch()

    set_motor_speed(MIN_DRIVE_SPEED, MIN_DRIVE_SPEED)

    while not touch_sensor.pressed():

        if watch.time() > max_time_ms:
            break

        wait(10)

    stop()

    return touch_sensor.pressed()


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
