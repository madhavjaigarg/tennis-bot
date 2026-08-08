from geometry import normalize_angle

class Odometry:
    def __init__(self, left_motor, right_motor, hub):
        self.left_motor = left_motor
        self.right_motor = right_motor
        self.hub = hub

        self.x = 0
        self.y = 0
        self.heading = 0

        self.left_start = 0
        self.right_start = 0

    def reset(self):
        self.x = 0
        self.y = 0
        self.heading = self.hub.imu.heading()

        self.left_start = self.left_motor.angle()
        self.right_start = self.right_motor.angle()

    def update(self):
        self.heading = self.hub.imu.heading()

        left_delta = self.left_motor.angle() - self.left_start
        right_delta = self.right_motor.angle() - self.right_start

        average_delta = (left_delta + right_delta) / 2

        self.x = average_delta

    def get_heading(self):
        return normalize_angle(self.heading)

    def get_position(self):
        return self.x, self.y

