"""
RoboSports Ball Tracking

Converts raw camera detections (pixel x, y, area) into a field
(x, y) position, using the robot's current odometry position as
the reference point.

Only handles ORANGE ball detections right now — that's the only
color openmv_ball_server.py thresholds for. If purple-ball
tracking is needed later, the camera server needs a second
threshold added; this file doesn't need to change to support it.

============================================================
CALIBRATION — ONE MEASUREMENT REQUIRED
============================================================

Distance is estimated from blob area using a pinhole-camera
assumption: apparent area shrinks with the SQUARE of distance
(2x farther away = 1/4 the pixel area).

    distance = CALIBRATION_DISTANCE_MM * sqrt(
        CALIBRATION_AREA_PX / current_area_px
    )

To calibrate: place the ball at a known distance from the
camera lens, run rafael_camera.print_ball(), and set the two
constants below from what it reports.
"""

from math import sqrt, cos, sin, radians

from rafael_camera import read_ball
import rafael_odometry
from rafael_math_utils import normalize


# ============================================================
# CAMERA FRAME  (QVGA, matches openmv_ball_server.py)
# ============================================================

FRAME_WIDTH_PX = 320
FRAME_HEIGHT_PX = 240

# TODO_VERIFY: standard OpenMV H7 Plus lens is ~70 degrees FOV
# per OpenMV's own spec sheet. Confirm this matches your actual
# lens (different if you've swapped it for a wide/telephoto one).
HORIZONTAL_FOV_DEG = 70.0


# ============================================================
# DISTANCE CALIBRATION 
# ============================================================

CALIBRATION_DISTANCE_MM = 300.0
CALIBRATION_AREA_PX = 850.0          # placeholder — replace with your measurement


# ============================================================
# CONVERSION
# ============================================================

def estimate_distance_mm(area_px):
    """
    Estimate distance to a ball from its blob area, using the
    one calibration measurement above.

    Returns None if area_px is 0 (nothing to estimate from).
    """

    if area_px <= 0:
        return None

    return CALIBRATION_DISTANCE_MM * sqrt(CALIBRATION_AREA_PX / area_px)


def estimate_angle_deg(x_px):
    """
    Estimate the ball's angle relative to the robot's current
    heading, from its pixel x position.

    Positive = ball is to the robot's right (matches the
    clockwise-positive heading convention used in odometry.py —
    turning right increases heading).
    """

    frame_center = FRAME_WIDTH_PX / 2.0

    fraction = (x_px - frame_center) / frame_center

    return fraction * (HORIZONTAL_FOV_DEG / 2.0)


def get_ball_field_position():
    """
    Return the ball's estimated (x, y) field position, combining
    the current camera reading with the robot's current odometry
    position.

    The result is smoothed (see SMOOTHING section below) since
    raw camera readings jump around frame to frame.

    Returns None if no ball is currently detected.
    """

    global _smoothed_position

    ball = read_ball()

    if not ball["found"]:
        _smoothed_position = None      # reset once the ball is lost
        return None

    distance_mm = estimate_distance_mm(ball["area"])

    if distance_mm is None:
        _smoothed_position = None
        return None

    relative_angle_deg = estimate_angle_deg(ball["x"])

    robot_x, robot_y, robot_heading = rafael_odometry.get_position()

    target_heading = normalize(robot_heading + relative_angle_deg)

    theta = radians(target_heading)

    raw_x = robot_x + distance_mm * cos(theta)
    raw_y = robot_y - distance_mm * sin(theta)     # same sign flip as odometry.update()

    return _smooth(raw_x, -(raw_y))


# ============================================================
# SMOOTHING
# ============================================================

# Raw camera readings are noisy frame to frame (blob area and
# pixel position both jitter), which makes the computed field
# position jump around too. Smooth it with an exponential moving
# average instead of trusting each frame's reading directly.

SMOOTHING_ALPHA = 0.3   # 0 = ignore new readings, 1 = no smoothing at all

_smoothed_position = None


def _smooth(raw_x, raw_y):
    """
    Blend a new raw reading into the running smoothed position.
    Resets automatically (see get_ball_field_position) whenever
    the ball is lost, so an old smoothed position never lingers
    once tracking picks up a ball again.
    """

    global _smoothed_position

    if _smoothed_position is None:
        _smoothed_position = (raw_x, raw_y)
        return _smoothed_position

    prev_x, prev_y = _smoothed_position

    smoothed_x = SMOOTHING_ALPHA * raw_x + (1 - SMOOTHING_ALPHA) * prev_x
    smoothed_y = SMOOTHING_ALPHA * raw_y + (1 - SMOOTHING_ALPHA) * prev_y

    _smoothed_position = (smoothed_x, smoothed_y)

    return _smoothed_position


# ============================================================
# DEBUGGING
# ============================================================

def print_ball_field_position():
    """
    Print the ball's estimated field position, for testing.
    """

    print(get_ball_field_position())
