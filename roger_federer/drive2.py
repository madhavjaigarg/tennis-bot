from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, UltrasonicSensor
from pybricks.parameters import Direction, Port
from pybricks.robotics import DriveBase
from pybricks.tools import wait

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

target_distance = 80  # mm

while True:

    d = distance_sens.distance()

    print("Distance:", d)

    if d <= target_distance:
        drive_base.stop()
        print("OBJECT DETECTED - STOPPED")
        
    else:
        drive_base.drive(200, 0)

    wait(50)

