"""
RoboSports Cartesian Odometry
Pybricks

Tracks:
    x       = position in mm
    y       = position in mm
    heading = mathematical heading in degrees

Coordinate convention:

              +Y
               ^
               |
               |
        -X <---+---> +X
               |
               |
               v
              -Y

heading = 0°  -> +X
heading = 90° -> +Y
heading = 180° -> -X
heading = -90° -> -Y

This file uses:
    - Left motor encoder
    - Right motor encoder
    - PrimeHub IMU heading

The motor encoders provide distance traveled.
The gyro provides the robot's heading.
"""

from math import pi, sin, cos, radians

from pybricks.tools import StopWatch, wait

from robot import left_motor, right_motor, gyro
from constants import WHEEL_DIAMETER


# ============================================================
# ROBOT GEOMETRY
# ============================================================

TRACK_WIDTH = 142.0          # mm
WHEEL_CIRCUMFERENCE = pi * WHEEL_DIAMETER


# ============================================================
# POSITION STATE
# ============================================================

x = 0.0
y = 0.0

# Mathematical heading.
#
# 0   = +X
# 90  = +Y
# 180 = -X
# -90 = -Y
heading = 0.0


# ============================================================
# ENCODER STATE
# ============================================================

_previous_left_angle = 0.0
_previous_right_angle = 0.0


# ============================================================
# ODOMETRY TIMER
# ============================================================

_watch = StopWatch()


# ============================================================
# HELPER FUNCTIONS
# ============================================================

def motor_degrees_to_mm(degrees):
    """
    Convert wheel rotation in degrees to linear distance.

    Example:
        360 degrees = one complete wheel revolution.
    """

    return (degrees / 360.0) * WHEEL_CIRCUMFERENCE


def get_gyro_heading():
    """
    Get the robot heading from the SPIKE Prime IMU.

    Pybricks reports positive heading as clockwise.

    Our coordinate system uses positive angles
    counter-clockwise.

    Therefore:

        mathematical_heading = -pybricks_heading
    """

    return -gyro.heading()


def normalize_heading(angle):
    """
    Convert any angle into the range:

        -180° to +180°
    """

    while angle > 180:
        angle -= 360

    while angle < -180:
        angle += 360

    return angle


# ============================================================
# RESET
# ============================================================

def reset_position(new_x=0.0, new_y=0.0, new_heading=0.0):
    """
    Completely reset odometry.

    This is what we will eventually call after
    calibration against the starting wall.

    Example:

        reset_position(0, 0, 0)
    """

    global x
    global y
    global heading

    global _previous_left_angle
    global _previous_right_angle

    x = new_x
    y = new_y

    heading = new_heading

    # Make the gyro agree with our chosen heading.
    gyro.reset_heading(-new_heading)

    # Reset encoder reference positions.
    _previous_left_angle = left_motor.angle()
    _previous_right_angle = right_motor.angle()

    _watch.reset()


# ============================================================
# READ POSITION
# ============================================================

def get_position():
    """
    Return the current Cartesian position.

    Returns:

        (x, y, heading)
    """

    return x, y, heading


# ============================================================
# UPDATE ODOMETRY
# ============================================================

def update():
    """
    Update the robot's Cartesian position.

    This should be called repeatedly while the robot moves.

    The algorithm:

        1. Read left encoder.
        2. Read right encoder.
        3. Calculate wheel movement.
        4. Calculate average forward movement.
        5. Read gyro heading.
        6. Project movement onto X and Y.
        7. Update x and y.
    """

    global x
    global y
    global heading

    global _previous_left_angle
    global _previous_right_angle

    # --------------------------------------------------------
    # Read motor encoders
    # --------------------------------------------------------

    current_left_angle = left_motor.angle()
    current_right_angle = right_motor.angle()

    # --------------------------------------------------------
    # Calculate change in wheel rotation
    # --------------------------------------------------------

    left_delta_angle = (
        current_left_angle -
        _previous_left_angle
    )

    right_delta_angle = (
        current_right_angle -
        _previous_right_angle
    )

    # --------------------------------------------------------
    # Save encoder positions
    # --------------------------------------------------------

    _previous_left_angle = current_left_angle
    _previous_right_angle = current_right_angle

    # --------------------------------------------------------
    # Convert wheel rotation to distance
    # --------------------------------------------------------

    left_distance = motor_degrees_to_mm(left_delta_angle)

    right_distance = motor_degrees_to_mm(right_delta_angle)

    # --------------------------------------------------------
    # Average distance traveled by robot
    # --------------------------------------------------------

    forward_distance = (
        left_distance + right_distance
    ) / 2.0

    # --------------------------------------------------------
    # Get gyro heading
    # --------------------------------------------------------

    heading = get_gyro_heading()

    # --------------------------------------------------------
    # Convert heading to radians
    # --------------------------------------------------------

    theta = radians(heading)

    # --------------------------------------------------------
    # Convert robot movement into field movement
    # --------------------------------------------------------

    dx = forward_distance * cos(theta)
    dy = forward_distance * sin(theta)

    # --------------------------------------------------------
    # Update position
    # --------------------------------------------------------

    x += dx
    y += dy


# ============================================================
# CONTINUOUS ODOMETRY LOOP
# ============================================================

def update_for(duration_ms):
    """
    Update odometry continuously for a specified time.

    This is mainly useful for testing.

    Example:

        update_for(5000)
    """

    start = _watch.time()

    while _watch.time() - start < duration_ms:

        update()

        wait(10)


# ============================================================
# DEBUGGING
# ============================================================

def print_position():
    """
    Print the current position to the Pybricks console.
    """

    print(
        "X:",
        round(x, 1),
        "Y:",
        round(y, 1),
        "Heading:",
        round(heading, 1)
    )


# ============================================================
# TEST ROUTINE
# ============================================================

def test_odometry():
    """
    Simple odometry test.

    The robot should be placed on the floor.

    It does NOT drive the motors.

    Move the robot manually and observe the
    reported encoder/gyro information.

    This is only a diagnostic function.
    """

    print("ODOMETRY TEST")

    reset_position(0, 0, 0)

    while True:

        update()

        print_position()

        wait(100)
