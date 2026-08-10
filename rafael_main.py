from rafael_robot import left_motor, right_motor
from rafael_motion import set_motor_speed, stop, drive_straight
from pybricks.tools import wait
from rafael_odometry import print_position

drive_straight(500)

print_position()
