"""
Robot hardware.
"""

from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, UltrasonicSensor, ForceSensor
from pybricks.parameters import Port

hub = PrimeHub()

left_motor = Motor(Port.A)

right_motor = Motor(Port.E)

distance_sensor = UltrasonicSensor(Port.B)

touch_sensor = ForceSensor(Port.D)

gyro = hub.imu
