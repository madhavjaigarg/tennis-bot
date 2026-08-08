from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor, ForceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch

hub = PrimeHub()

left_motor = Motor(
    Port.A,
    positive_direction=Direction.COUNTERCLOCKWISE
)

right_motor = Motor(
    Port.E,
    positive_direction=Direction.CLOCKWISE
)

distance_sens = UltrasonicSensor(Port.F)

drive_base = DriveBase(
    left_motor,
    right_motor,
    wheel_diameter=85,
    axle_track=128
)

target_distance = 80
hub.imu.reset_heading(0)

def get_direction():
    heading = hub.imu.heading()

    if heading < 45 or heading >= 315:
        return 0 
    elif heading < 135:
        return 90
    elif heading < 225:
        return 180
    else:
        return 270

directions = []


while True:

    d = distance_sens.distance()
    heading = hub.imu.heading()

    print("Distance:", d, "Heading:", heading)

    if d <= target_distance:
        drive_base.stop()

        while distance_sens.distance() <= target_distance:
            drive_base.drive(0, 100)
            wait(10)

        drive_base.stop()
        direction = get_direction()
        directions.append(direction)

        print("New direction:", direction)
        print("Directions:", directions)
        

    else:
        drive_base.drive(200, 0)

    wait(20) 

