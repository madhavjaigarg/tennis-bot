from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor, ForceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch

hub = PrimeHub()
touch_sensor = ForceSensor(Port.B)
left_motor = Motor(Port.A, positive_direction=Direction.COUNTERCLOCKWISE)
right_motor = Motor(Port.E, positive_direction=Direction.CLOCKWISE)
drive_base = DriveBase(left_motor,right_motor,wheel_diameter=85,axle_track=128)

while True:
    if touch_sensor.pressed():
        drive_base.brake()
        hub.speaker.beep()
        print("touch pressed")
    

    else:
        drive_base.drive(200, 0)
    wait(50)