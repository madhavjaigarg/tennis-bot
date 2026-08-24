import math

WHEEL_DIAMETER = 81
AXLE_TRACK = 142

WHEEL_CIRCUMFERENCE = math.pi * WHEEL_DIAMETER
MM_PER_MOTOR_DEGREE = 0.915

def motor_degrees_to_mm(degrees):
    return degrees * MM_PER_MOTOR_DEGREE

def mm_to_motor_degrees(mm):
    return mm / MM_PER_MOTOR_DEGREE

def normalize_angle(angle):
    while angle > 180:
        angle -= 360

    while angle <= -180:
        angle += 360

    return angle



