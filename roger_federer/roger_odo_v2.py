"""
roger_odo_v2.py

New file — roger_odo.py is left untouched.

Fixes vs roger_odo.py:
  - roger_odo.py did `from geometry import normalize_angle`, but
    there is no geometry.py in this repo (the module is
    roger_geo.py) — that import fails immediately on the hub.
    This imports from roger_geo_v2 instead.
  - update() in roger_odo.py set self.x directly to the raw
    average ENCODER DEGREES — never converted to mm, never split
    into x/y by heading — and never touched self.y at all. That
    only looks right if the robot drives in a single straight
    line and never turns. This version converts to mm and
    projects into x/y using the current heading, the same way
    rafael_odometry.py does for the other robot.
  - left_start/right_start were only ever set once, in reset(),
    so the encoder deltas kept growing for the whole match instead
    of representing "since the last update()". Fixed by re-basing
    them every update() call.
"""

from math import radians, cos, sin

from roger_geo_v2 import normalize_angle, motor_degrees_to_mm


class Odometry:
    def __init__(self, left_motor, right_motor, hub):
        self.left_motor = left_motor
        self.right_motor = right_motor
        self.hub = hub

        self.x = 0.0
        self.y = 0.0
        self.heading = 0.0

        self.left_start = 0
        self.right_start = 0

    def reset(self, x=0.0, y=0.0, heading=0.0):
        """
        Zero odometry at a known position/heading. Call once at
        the start of the match, robot placed in its starting spot.
        """

        self.x = x
        self.y = y
        self.heading = heading

        self.hub.imu.reset_heading(-heading)

        self.left_start = self.left_motor.angle()
        self.right_start = self.right_motor.angle()

    def update(self):
        """Call every loop tick while the robot moves."""

        self.heading = self.get_heading()

        left_now = self.left_motor.angle()
        right_now = self.right_motor.angle()

        left_delta_deg = left_now - self.left_start
        right_delta_deg = right_now - self.right_start

        self.left_start = left_now
        self.right_start = right_now

        left_mm = motor_degrees_to_mm(left_delta_deg)
        right_mm = motor_degrees_to_mm(right_delta_deg)

        forward_mm = (left_mm + right_mm) / 2.0

        theta = radians(self.heading)

        self.x += forward_mm * cos(theta)
        self.y += -forward_mm * sin(theta)

    def get_heading(self):
        return normalize_angle(self.hub.imu.heading())

    def get_position(self):
        return self.x, self.y, self.heading
