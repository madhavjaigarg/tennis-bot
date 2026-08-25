"""
Robot hardware.
"""

from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, UltrasonicSensor, ForceSensor, ColorSensor
from pybricks.parameters import Port

hub = PrimeHub()

left_motor = Motor(Port.A)

right_motor = Motor(Port.E)

#color_sensor = ColorSensor(Port.B)

touch_sensor = ForceSensor(Port.D)

gyro = hub.imu
