"""
RoboSports Target Manager

Decides whether the currently-detected ball is worth going
after. The camera only reports the single biggest blob at a
time (see openmv_ball_server.py), so there's no multi-ball
selection here — just validation of the one detection we get.

If multi-ball targeting is needed later, the camera server needs
to report a list of blobs instead of just the biggest one — this
file would then need real selection logic added.
"""

from rafael_ball_tracking import get_ball_field_position
from rafael_field_map import is_within_field, is_past_barrier


def get_target():
    """
    Return the current ball's field (x, y) position if it's a
    valid target, otherwise None.

    A detection is treated as invalid if:
        - no ball is currently seen
        - the estimated position falls outside our half
        - the estimated position is past the barrier (a bad
          distance estimate, or a ball on the opponent's side
          that we can't touch anyway)
    """

    position = get_ball_field_position()

    if position is None:
        return None

    x, y = position

    if not is_within_field(x, y):
        return None

    if is_past_barrier(x):
        return None

    return position


def has_target():
    """
    True if there's currently a valid ball to go after.
    """

    return get_target() is not None
