
from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor, ForceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch
from pybricks.iodevices import UARTDevice

from math import atan2, degrees, radians, tan, cos, sin, sqrt

from roger_robot_v2 import hub, left_motor, right_motor
from roger_motion_v2 import Motion

import rafael_odometry
import rafael_navigation
import rafael_math_utils
import rafael_field_map


rafael_odometry.left_motor = left_motor
rafael_odometry.right_motor = right_motor
rafael_odometry.gyro = hub.imu


class RogerMotionAdapter:
    def __init__(self, motion):
        self.motion = motion

    def stop(self):
        self.motion.stop()

    def drive_distance(self, distance, speed=550, heading=None):
        self.motion.drive_distance(
            distance,
            speed=speed,
            heading=heading
        )

    def turn_to(self, heading, speed=150):
        self.motion.turn_to(
            heading,
            speed=speed
        )

    def turn_by(self, angle, speed=150):
        self.motion.turn_by(
            angle,
            speed=speed
        )


roger_motion = Motion(
    left_motor,
    right_motor,
    hub,
    rafael_odometry
)

rafael_navigation.rafael_motion = RogerMotionAdapter(
    roger_motion
)


uart = UARTDevice(
    Port.F,
    baudrate=115200,
    timeout=1000,
    power_pin=2
)

flywheel = Motor(
    Port.C,
    positive_direction=Direction.COUNTERCLOCKWISE
)


image_width = 320
camera_fov = 60.0
ball_diameter_mm = 70.0

focal_length = (
    (image_width / 2.0) /
    tan(radians(camera_fov / 2.0))
)

stop_distance = 180.0

flywheel_speed = 1000
flywheel_park_angle = 215
flywheel_park_speed = 300
shoot_time = 1500

search_speed = 400

search_points = [
    (150, -150),
    (500, -150),
    (850, -150),
    (1100, -150),

    (1100, 50),
    (850, 50),
    (500, 50),
    (150, 50),

    (150, 300),
    (500, 300),
    (850, 300),
    (1100, 300)
]

search_index = 0

ball_locked = False
shooting = False
returning_to_search = False

buffer = b""


def normalize_angle(angle):
    return rafael_math_utils.normalize(angle)


def distance_to(x, y):
    return rafael_navigation.distanceTo(
        x,
        y
    )


def angle_to(x, y):
    return rafael_navigation.angleTo(
        x,
        y
    )


def park_flywheel():
    flywheel.run_target(
        flywheel_park_speed,
        flywheel_park_angle,
        then=Stop.HOLD
    )


def get_ball_position(camera_x, ball_width):
    if ball_width <= 0:
        return None

    distance = (
        focal_length *
        ball_diameter_mm /
        ball_width
    )

    bearing = degrees(
        atan2(
            camera_x - image_width / 2.0,
            focal_length
        )
    )

    robot_x, robot_y, robot_heading = (
        rafael_odometry.get_position()
    )

    ball_heading = robot_heading + bearing

    heading_rad = radians(
        ball_heading
    )

    ball_x = (
        robot_x +
        distance *
        cos(heading_rad)
    )

    ball_y = (
        robot_y -
        distance *
        sin(heading_rad)
    )

    return (
        ball_x,
        ball_y,
        distance
    )


def valid_search_point(x, y):
    if not rafael_field_map.is_within_field(
        x,
        y
    ):
        return False

    if rafael_field_map.is_past_barrier(
        x
    ):
        return False

    return True


def go_to_search_point():
    global search_index

    target_x, target_y = (
        search_points[search_index]
    )

    if not valid_search_point(
        target_x,
        target_y
    ):
        search_index += 1

        if search_index >= len(
            search_points
        ):
            search_index = 0

        return True

    distance = distance_to(
        target_x,
        target_y
    )

    if distance < 60:
        return True

    rafael_navigation.goTo(
        target_x,
        target_y
    )

    rafael_odometry.update()

    return True


def approach_ball(
    ball_x,
    ball_y,
    distance
):
    if distance <= stop_distance:
        roger_motion.stop()
        return True

    target_heading = angle_to(
        ball_x,
        ball_y
    )

    roger_motion.turn_to(
        target_heading,
        speed=100
    )

    rafael_odometry.update()

    distance = distance_to(
        ball_x,
        ball_y
    )

    if distance <= stop_distance:
        roger_motion.stop()
        return True

    drive_distance = (
        distance -
        stop_distance
    )

    roger_motion.drive_distance(
        drive_distance,
        speed=300,
        heading=target_heading
    )

    rafael_odometry.update()

    roger_motion.stop()

    return True


def face_ramp():
    target_heading = angle_to(
        0,
        0
    )

    print(
        "ramp heading:",
        round(target_heading, 1)
    )

    roger_motion.turn_to(
        target_heading,
        speed=100
    )

    rafael_odometry.update()


def shoot():
    global shooting
    global ball_locked
    global returning_to_search

    shooting = True

    roger_motion.stop()

    wait(250)

    face_ramp()

    wait(300)

    print("aligned")

    flywheel.run(
        flywheel_speed
    )

    wait(
        shoot_time
    )

    flywheel.stop()

    park_flywheel()

    wait(200)

    ball_locked = False
    shooting = False
    returning_to_search = True

    print("shot complete")
    print("returning to search")


def get_latest_message():
    global buffer

    data = uart.read_all()

    if data:
        buffer += data

    if b"\n" not in buffer:
        return None

    lines = buffer.split(b"\n")

    buffer = lines[-1]

    messages = [
        line.strip()
        for line in lines[:-1]
        if line.strip()
    ]

    latest_ball = None
    saw_no_ball = False

    for message in messages:
        if message.startswith(b"ball,"):
            latest_ball = message
        elif message == b"no ball":
            saw_no_ball = True

    if latest_ball is not None:
        return latest_ball

    if saw_no_ball:
        return b"no ball"

    return None


rafael_field_map.calibrate()

park_flywheel()

print("main ready")
print("flywheel parked at", flywheel.angle())

while True:

    rafael_odometry.update()

    message = get_latest_message()

    if shooting:
        wait(10)
        continue

    if returning_to_search:

        target_x, target_y = (
            search_points[search_index]
        )

        print(
            "returning to:",
            search_index,
            target_x,
            target_y
        )

        go_to_search_point()

        returning_to_search = False

        search_index += 1

        if search_index >= len(
            search_points
        ):
            search_index = 0

        wait(10)
        continue

    if message is None:
        wait(10)
        continue

    print(
        "received:",
        message
    )

    if message == b"no ball":

        if not ball_locked:

            target_x, target_y = (
                search_points[search_index]
            )

            print(
                "search:",
                search_index,
                target_x,
                target_y
            )

            reached = go_to_search_point()

            if reached:

                print(
                    "reached:",
                    search_index
                )

                search_index += 1

                if search_index >= len(
                    search_points
                ):
                    search_index = 0

    elif message.startswith(b"ball,"):

        if ball_locked:
            wait(10)
            continue

        parts = message.split(b",")

        if len(parts) < 7:
            wait(10)
            continue

        try:
            camera_x = int(
                parts[1]
            )

            ball_width = int(
                parts[4]
            )

            result = get_ball_position(
                camera_x,
                ball_width
            )

            if result is None:
                wait(10)
                continue

            ball_x, ball_y, distance = result

            print(
                "ball:",
                round(ball_x, 1),
                round(ball_y, 1),
                "distance:",
                round(distance, 1)
            )

            ball_locked = True

            roger_motion.stop()

            reached = approach_ball(
                ball_x,
                ball_y,
                distance
            )

            if reached:

                roger_motion.stop()

                print("ball locked")

                shoot()

        except Exception as e:
            print(
                "error:",
                e
            )

    wait(10)

