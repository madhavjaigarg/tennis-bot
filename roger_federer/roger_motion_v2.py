"""
roger_motion_v2.py

New file — roger_motion.py is left untouched.

Fixes vs roger_motion.py:
  - roger_motion.py did `from geometry import ...` — same missing-
    module problem as roger_odo.py. Imports from roger_geo_v2
    instead.
  - drive_distance() ran both motors at one fixed speed for the
    whole move with no feedback at all. Any real-world asymmetry
    (friction, a slightly-off encoder, getting bumped by another
    robot) makes it curve, and nothing corrects it. This version
    adds gyro heading-hold correction each loop tick (DRIVE_KP/KI/
    KD from roger_geo_v2.py), the same approach rafael_motion.py
    uses.
  - takes roger_odo_v2's Odometry (needs get_heading(), matches
    the reset()/update() signatures above).
"""

from pybricks.tools import wait
from roger_geo_v2 import (
    mm_to_motor_degrees,
    normalize_angle,
    clamp,
    DRIVE_KP,
    DRIVE_KI,
    DRIVE_KD,
)


class Motion:
    def __init__(self, left_motor, right_motor, hub, odometry):
        self.left_motor = left_motor
        self.right_motor = right_motor
        self.hub = hub
        self.odometry = odometry

    def stop(self):
        self.left_motor.stop()
        self.right_motor.stop()

    def drive_distance(self, distance, speed=200, heading=None):
        """
        Drive `distance` mm, holding a straight heading via gyro
        correction. If heading is None, holds whatever heading the
        robot is facing when the call starts.
        """

        start_left = self.left_motor.angle()
        start_right = self.right_motor.angle()

        target_degrees = abs(mm_to_motor_degrees(distance))

        direction = 1 if distance >= 0 else -1

        if heading is None:
            heading = self.odometry.get_heading()

        integral = 0
        previous_error = 0

        while True:
            left_delta = abs(self.left_motor.angle() - start_left)
            right_delta = abs(self.right_motor.angle() - start_right)

            average_delta = (left_delta + right_delta) / 2

            if average_delta >= target_degrees:
                break

            current_heading = self.odometry.get_heading()
            error = normalize_angle(heading - current_heading)

            integral += error
            derivative = error - previous_error
            previous_error = error

            correction = (
                DRIVE_KP * error +
                DRIVE_KI * integral +
                DRIVE_KD * derivative
            )

            self.left_motor.run(direction * (speed + correction))
            self.right_motor.run(direction * (speed - correction))

            self.odometry.update()
            wait(5)

        self.stop()
        self.odometry.update()

    def turn_by(self, angle, speed=150):
        start_heading = self.odometry.get_heading()
        target_heading = normalize_angle(start_heading + angle)

        direction = 1 if angle > 0 else -1

        while True:
            current_heading = self.odometry.get_heading()
            error = normalize_angle(target_heading - current_heading)

            if abs(error) <= 1:
                break

            if direction > 0:
                self.left_motor.run(speed)
                self.right_motor.run(-speed)
            else:
                self.left_motor.run(-speed)
                self.right_motor.run(speed)

            self.odometry.update()
            wait(5)

        self.stop()
        self.odometry.update()

    def turn_to(self, target_heading, speed=150):
        target_heading = normalize_angle(target_heading)

        while True:
            current_heading = self.odometry.get_heading()
            error = normalize_angle(target_heading - current_heading)

            if abs(error) <= 1:
                break

            if error > 0:
                self.left_motor.run(speed)
                self.right_motor.run(-speed)
            else:
                self.left_motor.run(-speed)
                self.right_motor.run(speed)

            self.odometry.update()
            wait(5)

        self.stop()
        self.odometry.update()
