"""
RoboSports Motion Controller
Pybricks

This is the low-level movement system.

Provides:

    drive_distance(mm)
    drive_straight(mm)
    turn_to(angle)
    turn_by(angle)
    stop()

The system uses:
    - Left motor encoder
    - Right motor encoder
    - SPIKE Prime IMU
    - PID control

Coordinate convention:

    0°    = +X
    -90°   = +Y
    180°  = -X
    90°  = -Y

Important:
    odometry.py uses the same mathematical heading convention.
"""

from math import pi

from pybricks.tools import wait

from rafael_robot import left_motor, right_motor, gyro

from rafael_constants import (
    WHEEL_DIAMETER,
    CRUISE_SPEED,
    TURN_SPEED,
    MAX_SPEED,
    DRIVE_KP,
    DRIVE_KI,
    DRIVE_KD,
    TURN_KP,
    TURN_KI,
    TURN_KD,
)

import rafael_odometry


# ============================================================
# CONSTANTS
# ============================================================

WHEEL_CIRCUMFERENCE = pi * WHEEL_DIAMETER


# ============================================================
# MOTOR DIRECTION
# ============================================================

# IMPORTANT:
#
# Test this before competition.
#
# If both motors physically drive the robot forward when
# commanded with positive speed, leave these as they are.
#
# If one motor is mounted in the opposite orientation,
# change its sign to -1.
#
# We will verify this during testing.

LEFT_DIRECTION = -1
RIGHT_DIRECTION = 1


# ============================================================
# PID SETTINGS
# ============================================================

# Straight driving correction.
#
# This compares the desired gyro heading against the
# current gyro heading.

STRAIGHT_KP = DRIVE_KP
STRAIGHT_KI = DRIVE_KI
STRAIGHT_KD = DRIVE_KD


# Turning PID.

TURNING_KP = TURN_KP
TURNING_KI = TURN_KI
TURNING_KD = TURN_KD


# ============================================================
# LIMITS
# ============================================================

MIN_DRIVE_SPEED = 80

MIN_TURN_SPEED = 60

MAX_TURN_SPEED = TURN_SPEED


# ============================================================
# HELPER FUNCTIONS
# ============================================================

def clamp(value, minimum, maximum):
    """
    Restrict value to a range.
    """

    if value < minimum:
        return minimum

    if value > maximum:
        return maximum

    return value


def normalize_angle(angle):
    """
    Convert angle to:

        -180° to +180°
    """

    while angle > 180:
        angle -= 360

    while angle < -180:
        angle += 360

    return angle


def get_heading():
    """
    Return our mathematical heading.

    Pybricks:
        clockwise positive

    Our system:
        clockwise positive
    """

    return gyro.heading()


# ============================================================
# MOTOR CONTROL
# ============================================================

def set_motor_speed(left_speed, right_speed):
    """
    Set both motor speeds in degrees/second.

    Positive = forward
    Negative = reverse
    """

    left_motor.run(
        LEFT_DIRECTION * left_speed
    )

    right_motor.run(
        RIGHT_DIRECTION * right_speed
    )


def stop():
    """
    Stop both drive motors.
    """

    left_motor.stop()
    right_motor.stop()


# ============================================================
# DRIVE DISTANCE
# ============================================================

def degrees_for_distance(distance_mm):
    """
    Convert linear distance in mm into wheel rotation.
    """

    return (
        distance_mm /
        WHEEL_CIRCUMFERENCE
    ) * 360.0


def drive_distance(
    distance_mm,
    speed=CRUISE_SPEED,
    heading=None
):
    """
    Drive a specified distance.

    Example:

        drive_distance(500)

    drives forward 500 mm.

    Example:

        drive_distance(-300)

    drives backward 300 mm.

    If heading is supplied, gyro correction keeps
    the robot travelling in that direction.
    """

    # --------------------------------------------------------
    # Determine direction
    # --------------------------------------------------------

    direction = 1

    if distance_mm < 0:
        direction = -1

    target_degrees = abs(
        degrees_for_distance(distance_mm)
    )

    # --------------------------------------------------------
    # Reset motor encoders
    # --------------------------------------------------------

    left_motor.reset_angle(0)
    right_motor.reset_angle(0)

    # --------------------------------------------------------
    # Heading target
    # --------------------------------------------------------

    if heading is None:
        heading = get_heading()

    # --------------------------------------------------------
    # PID variables
    # --------------------------------------------------------

    integral = 0
    previous_error = 0

    # --------------------------------------------------------
    # Motion loop
    # --------------------------------------------------------

    while True:

        # ----------------------------------------------------
        # Encoder distance
        # ----------------------------------------------------

        left_angle = abs(left_motor.angle())
        right_angle = abs(right_motor.angle())

        average_angle = (
            left_angle + right_angle
        ) / 2.0

        # ----------------------------------------------------
        # Remaining distance
        # ----------------------------------------------------

        remaining = (
            target_degrees -
            average_angle
        )

        # ----------------------------------------------------
        # Finished?
        # ----------------------------------------------------

        if remaining <= 2:

            break

        # ----------------------------------------------------
        # Calculate base speed
        # ----------------------------------------------------

        # Slow down near target.

        if remaining < 150:

            base_speed = max(
                MIN_DRIVE_SPEED,
                speed * remaining / 150
            )

        else:

            base_speed = speed

        base_speed = clamp(
            base_speed,
            MIN_DRIVE_SPEED,
            MAX_SPEED
        )

        # ----------------------------------------------------
        # Gyro correction
        # ----------------------------------------------------

        current_heading = get_heading()

        error = normalize_angle(
            heading - current_heading
        )

        integral += error

        derivative = (
            error -
            previous_error
        )

        previous_error = error

        correction = (
            STRAIGHT_KP * error +
            STRAIGHT_KI * integral +
            STRAIGHT_KD * derivative
        )

        # ----------------------------------------------------
        # Motor speeds
        # ----------------------------------------------------

        left_speed = (
            direction *
            (base_speed + correction)
        )

        right_speed = (
            direction *
            (base_speed - correction)
        )

        left_speed = clamp(
            left_speed,
            -MAX_SPEED,
            MAX_SPEED
        )

        right_speed = clamp(
            right_speed,
            -MAX_SPEED,
            MAX_SPEED
        )

        set_motor_speed(
            left_speed,
            right_speed
        )

        # ----------------------------------------------------
        # Update odometry
        # ----------------------------------------------------

        rafael_odometry.update()

        wait(10)

    # --------------------------------------------------------
    # Stop
    # --------------------------------------------------------

    stop()

    # Give motors time to settle.
    wait(50)

    # Final odometry update.
    rafael_odometry.update()


# ============================================================
# DRIVE STRAIGHT
# ============================================================

def drive_straight(distance_mm, speed=CRUISE_SPEED):
    """
    Drive straight relative to the robot's current heading.

    Example:

        drive_straight(500)

    means:

        drive 500 mm in the direction currently faced.
    """

    heading = get_heading()

    drive_distance(
        distance_mm,
        speed,
        heading
    )


# ============================================================
# TURN TO ABSOLUTE HEADING
# ============================================================

def turn_to(
    target_heading,
    speed=TURN_SPEED
):
    """
    Turn to an absolute mathematical heading.

    Example:

        turn_to(90)

    turns until the robot faces +Y.
    """

    integral = 0
    previous_error = 0

    while True:

        current_heading = get_heading()

        error = -normalize_angle(
            target_heading -
            current_heading
        )

        # ----------------------------------------------------
        # Finished?
        # ----------------------------------------------------

        if abs(error) < 1.0:

            break

        # ----------------------------------------------------
        # PID
        # ----------------------------------------------------

        integral += error

        derivative = (
            error -
            previous_error
        )

        previous_error = error

        turn_power = (
            TURNING_KP * error +
            TURNING_KI * integral +
            TURNING_KD * derivative
        )

        # ----------------------------------------------------
        # Limit speed
        # ----------------------------------------------------

        turn_power = clamp(
            turn_power,
            -MAX_TURN_SPEED,
            MAX_TURN_SPEED
        )

        # Prevent extremely slow turning.
        if 0 < abs(turn_power) < MIN_TURN_SPEED:

            turn_power = (
                MIN_TURN_SPEED
                if turn_power > 0
                else -MIN_TURN_SPEED
            )

        # ----------------------------------------------------
        # Tank turn
        # ----------------------------------------------------

        set_motor_speed(
            -turn_power,
            turn_power
        )

        rafael_odometry.update()

        wait(10)

    stop()

    wait(75)

    rafael_odometry.update()


# ============================================================
# RELATIVE TURN
# ============================================================

def turn_by(angle):
    """
    Turn relative to the robot's current heading.

    Example:

        turn_by(90)

    rotates 90° clockwise.

    Example:

        turn_by(-90)

    rotates 90° counter-clockwise.
    """

    current = get_heading()

    target = normalize_angle(
        current + angle
    )

    turn_to(target)


# ============================================================
# FACE DIRECTION
# ============================================================

def face_direction(angle):
    """
    Alias for turn_to().
    """

    turn_to(angle)


# ============================================================
# TEST FUNCTIONS
# ============================================================

def test_forward():
    """
    Basic forward test.

    Robot should drive approximately 500 mm.
    """

    print("FORWARD TEST")

    drive_straight(500)

    print("Position:")
    rafael_odometry.print_position()


def test_turn():
    """
    Basic 90° turn test.
    """

    print("TURN TEST")

    turn_by(90)

    print("Position:")
    rafael_odometry.print_position()


def test_motion():
    """
    Complete basic movement test.

    Sequence:

        Forward 500 mm
        Turn 90°
        Forward 500 mm
    """

    print("MOTION TEST")

    drive_straight(500)

    turn_by(90)

    drive_straight(500)

    print("FINAL POSITION")

    rafael_odometry.print_position()
