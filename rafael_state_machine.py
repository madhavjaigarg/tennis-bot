"""
RoboSports State Machine

Ties everything together into the match loop:

    find ball -> go to ball -> collect -> go to barrier -> flick
    -> repeat

Every mechanism is real now — no stubs left. Ball collection is
passive (driving into the ball pushes it into the hollow), so
there's no intake motor and no rafael_intake.py. The flicker
(Port C, rafael_launcher.py) and touch sensor (Port D) are both
wired in for real.
"""

from pybricks.tools import wait, StopWatch

import rafael_odometry
import rafael_field_map as field_map
import rafael_navigation as navigation
from rafael_target_manager import get_target
from rafael_robot import touch_sensor
from rafael_motion import set_motor_speed, stop, MIN_DRIVE_SPEED, drive_straight
from rafael_launcher import flick


# ============================================================
# STATES
# ============================================================

STATE_FIND_BALL = "FIND_BALL"
STATE_GO_TO_BALL = "GO_TO_BALL"
STATE_COLLECT = "COLLECT"
STATE_GO_TO_BARRIER = "GO_TO_BARRIER"
STATE_FLICK = "FLICK"


# ============================================================
# MECHANISM CONTROL
# ============================================================

def _collect_ball():
    """
    Collection is passive — driving into the ball pushes it into
    the hollow, no motor action needed. Just a brief pause to
    let it settle before moving on.
    """

    wait(200)


def _flick_ball():
    """
    Run the flicker to chuck the ball over the barrier.
    """

    flick()


def _touching_barrier():
    """
    True if the front touch sensor is currently pressed.
    """

    return touch_sensor.pressed()


def _creep_to_barrier(max_time_ms=3000):
    """
    Creep slowly forward until the front touch sensor detects
    contact with the barrier, or max_time_ms elapses (safety
    timeout in case contact never registers — e.g. the robot
    stalled or odometry put it somewhere unexpected).

    Returns True if contact was made, False if it timed out.
    """

    watch = StopWatch()

    set_motor_speed(MIN_DRIVE_SPEED, MIN_DRIVE_SPEED)

    while not touch_sensor.pressed():

        if watch.time() > max_time_ms:
            break

        wait(10)

    stop()

    return touch_sensor.pressed()


# ============================================================
# MAIN LOOP
# ============================================================

def run():
    """
    Run the match strategy loop. Call once, after
    rafael_field_map.calibrate().
    """

    state = STATE_FIND_BALL

    while True:

        if state == STATE_FIND_BALL:

            if get_target() is not None:
                state = STATE_GO_TO_BALL
            else:
                wait(50)

        elif state == STATE_GO_TO_BALL:

            target = get_target()

            if target is None:
                state = STATE_FIND_BALL
            else:
                x, y = target
                navigation.goTo(x, y)
                state = STATE_COLLECT

        elif state == STATE_COLLECT:

            _collect_ball()
            state = STATE_GO_TO_BARRIER

        elif state == STATE_GO_TO_BARRIER:

            _, current_y, _ = rafael_odometry.get_position()

            barrier_x = (
                field_map.BARRIER_X -
                field_map.BARRIER_SAFETY_MARGIN_MM
            )

            navigation.goTo(barrier_x, current_y)

            if _creep_to_barrier():
                drive_straight(-30)
                state = STATE_FLICK
            else:
                # Didn't make contact within the timeout —
                # abandon this attempt rather than push forever.
                state = STATE_FIND_BALL

        elif state == STATE_FLICK:

            _flick_ball()
            state = STATE_FIND_BALL
