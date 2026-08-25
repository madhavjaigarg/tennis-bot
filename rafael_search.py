"""
RoboSports Search Patrol

When no ball is visible, the robot sweeps our half of the field
along a fixed rectangular route instead of sitting still and
waiting for one to drift into view.

The route is driven one leg at a time via navigation.goTo(), with
has_target() passed in as a stop_condition — so a leg is abandoned
immediately, mid-turn or mid-drive, the instant a ball becomes
visible. The robot does not need to reach a waypoint to react to
a ball.
"""

import rafael_navigation as navigation
import rafael_field_map as field_map
from rafael_target_manager import has_target


# ============================================================
# ROUTE
# ============================================================

# A single rectangular loop around our half of the field.
SEARCH_PATH = [
    (0, 0),
    (0, -200),
    (800, -200),
    (800, 0),
]


# ============================================================
# PATROL STATE
# ============================================================

_next_waypoint_index = 0

# False until the patrol has driven at least one real leg. The
# route starts AT (0, 0) — the robot is already sitting there,
# already calibrated by rafael_field_map.calibrate() at match
# start — so the very first patrol_step() call would otherwise
# match the (0, 0) check before the robot has gone anywhere and
# spin it around for no reason. Only a genuine RETURN to (0, 0),
# after the route has looped back around, should trigger the wall
# recalibration.
_started = False


def reset():
    """
    Restart the patrol from its first waypoint.

    Call this whenever the robot leaves the search state (e.g.
    it spotted a ball and is going to collect it), so the next
    time searching resumes it sweeps from the start of the route
    again instead of picking up wherever it last left off.
    """

    global _next_waypoint_index, _started

    _next_waypoint_index = 0
    _started = False


def patrol_step():
    """
    Drive one leg of the search route toward the next waypoint.

    Normally that's a plain goTo(), stopping immediately (mid-turn
    or mid-drive) if a ball becomes visible along the way. But if
    the next waypoint is the starting corner (0, 0) AND the patrol
    has already driven at least one real leg (i.e. this is a
    genuine return, not the very first tick), this instead calls
    field_map.return_home_and_calibrate() — driving into the wall
    and re-zeroing odometry there — since that's the robot's one
    chance mid-match to correct drift against a known, fixed
    reference point.

    Returns True if a ball was spotted — the caller should stop
    searching and go collect it. Returns False if this leg
    finished (waypoint reached, or the wall recalibration ran)
    with nothing found; call this again to continue on to the next
    waypoint. The route loops back to its first waypoint after the
    last one is reached.
    """

    global _next_waypoint_index, _started

    if _next_waypoint_index >= len(SEARCH_PATH):
        _next_waypoint_index = 0

    target_x, target_y = SEARCH_PATH[_next_waypoint_index]

    if (target_x, target_y) == (0, 0) and _started:

        field_map.return_home_and_calibrate()

        _next_waypoint_index += 1

        return False

    _started = True

    spotted = navigation.goTo(
        target_x,
        target_y,
        stop_condition=has_target
    )

    if spotted:
        return True

    _next_waypoint_index += 1

    return False


# ============================================================
# DEBUGGING
# ============================================================

def print_search_state():
    """
    Print which waypoint the patrol will head to next.
    """

    index = _next_waypoint_index % len(SEARCH_PATH)

    print("NEXT WAYPOINT:", SEARCH_PATH[index])
