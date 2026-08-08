from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor, ForceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch

hub = PrimeHub()
left_motor = Motor(Port.A, positive_direction=Direction.COUNTERCLOCKWISE)
right_motor = Motor(Port.E, positive_direction=Direction.CLOCKWISE)
distance_sens = UltrasonicSensor(Port.F)

drive_base = DriveBase(left_motor,right_motor,wheel_diameter=85,axle_track=128)
target_distance = 80

while True:
    d = distance_sens.distance()
    print(d)

    if d <= target_distance:
        drive_base.stop()
        
        start_heading = hub.imu.heading()
        target_heading = start_heading + 180
        
        while hub.imu.heading() < target_heading:
            drive_base.drive(0, 100)
            wait(10)

        drive_base.stop()

    else:
        drive_base.drive(200, 0)

    wait(20)
    

        
