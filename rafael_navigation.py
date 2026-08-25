"""
RoboSports Cartesian Navigation
Pybricks

High-level Cartesian navigation.

Coordinate convention:

          +Y
           ^
           |
    -X <---+---> +X
           |
           v
          -Y

Heading:
    0°    = +X
    -90°   = +Y
    180°  = -X
    90°  = -Y

All distances are in millimetres.
"""

from math import atan2, degrees, sqrt

import rafael_odometry
import rafael_motion


# ============================================================
# HOME POSITION
# ============================================================

# Temporary values.
#
# These will eventually be replaced by the actual starting
# position from rafael_calibration.py / rafael_field_map.py.

HOME_X = 0.0
HOME_Y = 0.0


# ============================================================
# ANGLE HELPER
# ============================================================

def normalize_angle(angle):
    """
    Normalize an angle to:

        -180° to +180°
    """

    while angle > 180:
        angle -= 360

    while angle < -180:
        angle += 360

    return angle


# ============================================================
# GET CURRENT POSITION
# ============================================================

def get_position():
    """
    Return the robot's current:

        x
        y
        heading

    Returns:

        (x, y, heading)
    """

    return rafael_odometry.get_position()


# ============================================================
# DISTANCE TO POINT
# ============================================================

def distanceTo(target_x, target_y):
    """
    Calculate the straight-line distance from the robot's
    current position to a target point.

    Example:

        distanceTo(1000, 500)

    Returns:

        distance in mm
    """

    current_x, current_y, _ = (
        rafael_odometry.get_position()
    )

    dx = target_x - current_x
    dy = target_y - current_y

    return sqrt(
        dx * dx +
        dy * dy
    )


# ============================================================
# ANGLE TO POINT
# ============================================================

def angleTo(target_x, target_y):
    """
    Calculate the absolute mathematical heading required
    for the robot to face a target point.

    Coordinate convention:

        0°   = +X
        -90°  = +Y
        180° = -X
        90° = -Y

    Returns:

        absolute heading in degrees
    """

    current_x, current_y, _ = (
        rafael_odometry.get_position()
    )

    dx = target_x - current_x
    dy = target_y - current_y

    angle = degrees(
        atan2(-dy, dx)
    )

    #return normalize_angle(angle)
    return angle
    

# ============================================================
# FACE POINT
# ============================================================

def facePoint(target_x, target_y):
    """
    Turn the robot so that it faces a target point.

    Example:

        facePoint(1000, 500)
    """

    target_heading = angleTo(
        target_x,
        target_y
    )

    rafael_motion.turn_to(
        target_heading
    )


# ============================================================
# GO TO POINT
# ============================================================

def goTo(target_x, target_y, stop_condition=None):
    """
    Drive the robot to a Cartesian target.

    Example:

        goTo(1000, 500)

    The robot:

        1. Gets its current position.
        2. Calculates the target angle.
        3. Turns toward the target.
        4. Calculates the target distance.
        5. Drives that distance.
        6. Updates odometry.

    If stop_condition is supplied, it's checked continuously
    during both the turn and the drive. The instant it returns
    True, movement stops immediately (mid-turn or mid-drive) and
    this function returns True without reaching target_x/target_y.
    Otherwise it completes the full move and returns False.
    """

    # --------------------------------------------------------
    # Current position
    # --------------------------------------------------------

    current_x, current_y, current_heading = (
        rafael_odometry.get_position()
    )

    # --------------------------------------------------------
    # Calculate target vector
    # --------------------------------------------------------

    dx = target_x - current_x
    dy = target_y - current_y

    distance = sqrt(
        dx * dx +
        dy * dy
    )

    # --------------------------------------------------------
    # Already at target
    # --------------------------------------------------------

    if distance < 1.0:
        return False

    # --------------------------------------------------------
    # Calculate required heading
    # --------------------------------------------------------

    #target_heading = normalize_angle(
        #degrees(
        #  atan2(-dy, dx)
       #)
    #)
    target_heading = degrees(atan2(-dy,dx))
    

    # --------------------------------------------------------
    # Turn toward target
    # --------------------------------------------------------

    turn_interrupted = rafael_motion.turn_to(
        target_heading,
        stop_condition=stop_condition
    )

    if turn_interrupted:
        return True

    # --------------------------------------------------------
    # Drive toward target
    #
    # We explicitly give drive_distance the target heading
    # so the calibrated straight-drive controller maintains
    # the direction we just turned toward.
    # --------------------------------------------------------

    interrupted = rafael_motion.drive_distance(
        distance,
        heading=target_heading,
        stop_condition=stop_condition
    )

    # --------------------------------------------------------
    # Final odometry update
    # --------------------------------------------------------

    rafael_odometry.update()

    return interrupted


# ============================================================
# RETURN HOME
# ============================================================

def returnHome():
    """
    Drive back to the configured home position.

    The home coordinates are temporary until the field
    map and starting-position calibration are completed.
    """

    goTo(
        HOME_X,
        HOME_Y
    )


# ============================================================
# DEBUGGING
# ============================================================

def print_navigation_state():
    """
    Print the current Cartesian navigation state.
    """

    rafael_odometry.update()

    x, y, heading = (
        rafael_odometry.get_position()
    )

    print(
        "X:",
        round(x, 1),
        "Y:",
        round(y, 1),
        "Heading:",
        round(heading, 1)
    )


# ============================================================
# TEST FUNCTIONS
# ============================================================

def test_distance(target_x, target_y):
    """
    Print the distance from the robot to a target.
    """

    distance = distanceTo(
        target_x,
        target_y
    )

    print(
        "Distance:",
        round(distance, 1),
        "mm"
    )


def test_angle(target_x, target_y):
    """
    Print the angle from the robot to a target.
    """

    angle = angleTo(
        target_x,
        target_y
    )

    print(
        "Angle:",
        round(angle, 1),
        "degrees"
    )


def test_goTo(target_x, target_y):
    """
    Test Cartesian navigation to a target.

    Prints the state before and after movement.
    """

    print("NAVIGATION TEST")

    print("TARGET:")
    print(
        "X:",
        target_x,
        "Y:",
        target_y
    )

    print("BEFORE:")
    print_navigation_state()

    print(
        "Distance:",
        round(
            distanceTo(
                target_x,
                target_y
            ),
            1
        ),
        "mm"
    )

    print(
        "Target angle:",
        round(
            angleTo(
                target_x,
                target_y
            ),
            1
        ),
        "degrees"
    )

    goTo(
        target_x,
        target_y
    )

    print("AFTER:")
    print_navigation_state()
