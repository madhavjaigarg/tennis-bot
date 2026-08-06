"""
Robot hardware.
"""

from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, UltrasonicSensor
from pybricks.parameters import Port

hub = PrimeHub()

left_motor = Motor(Port.A)

right_motor = Motor(Port.E)

distance_sensor = UltrasonicSensor(Port.D)

gyro = hub.imu
