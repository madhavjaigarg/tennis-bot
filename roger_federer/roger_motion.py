from pybricks.tools import wait
from geometry import mm_to_motor_degrees, normalize_angle

class Motion:
    def __init__(self, left_motor, right_motor, hub, odometry):
        self.left_motor = left_motor
        self.right_motor = right_motor
        self.hub = hub
        self.odometry = odometry

    def stop(self):
        self.left_motor.stop()
        self.right_motor.stop()

    def drive_distance(self, distance, speed=200):
        start_left = self.left_motor.angle()
        start_right = self.right_motor.angle()

        target_degrees = abs(mm_to_motor_degrees(distance))

        direction = 1 if distance >= 0 else -1

        while True:
            left_delta = abs(self.left_motor.angle() - start_left)
            right_delta = abs(self.right_motor.angle() - start_right)

            average_delta = (left_delta + right_delta) / 2

            if average_delta >= target_degrees:
                break

            self.left_motor.run(direction * speed)
            self.right_motor.run(direction * speed)

            self.odometry.update()
            wait(5)

        self.stop()
        self.odometry.update()

    def turn_by(self, angle, speed=150):
        start_heading = self.hub.imu.heading()
        target_heading = normalize_angle(start_heading + angle)

        direction = 1 if angle > 0 else -1

        while True:
            current_heading = self.hub.imu.heading()
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
            current_heading = self.hub.imu.heading()
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
