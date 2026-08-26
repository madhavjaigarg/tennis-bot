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

The patrol index is never reset — after a leg is interrupted by a
spotted ball (or completed normally), the next call to
patrol_step() always continues on to the next waypoint in the
route, whether that's right after collecting/launching a ball or
just the next tick of an uninterrupted sweep. There is no special
handling of the (0, 0) waypoint; it's driven through like any
other point on the route.
"""

import rafael_navigation as navigation
from rafael_motion import STATUS_STOP_CONDITION
from rafael_target_manager import has_target


# ============================================================
# ROUTE
# ============================================================

# A single rectangular loop around our half of the field.
SEARCH_PATH = [
    (0, 0),
    (0, -200),
    (800, -200),
    (800, 10),
]


# ============================================================
# PATROL STATE
# ============================================================

_next_waypoint_index = 0


def patrol_step():
    """
    Drive one leg of the search route toward the next waypoint,
    stopping immediately (mid-turn or mid-drive) if a ball becomes
    visible along the way.

    Returns True if a ball was actually spotted (has_target fired)
    — the caller should stop searching and go collect it. Returns
    False if this leg finished normally (waypoint reached) OR was
    cut short by an unrelated collision — either way there's no
    ball to chase, so we just move on to the next waypoint. The
    route loops back to its first waypoint after the last one is
    reached.

    Note: goTo() can also stop early because of an unexpected bump
    (STATUS_COLLISION), which is NOT a spotted ball. We only treat
    STATUS_STOP_CONDITION as "spotted" — collapsing both into a
    single True/False used to make an unrelated bump during patrol
    look like a found ball.
    """

    global _next_waypoint_index

    if _next_waypoint_index >= len(SEARCH_PATH):
        _next_waypoint_index = 0

    target_x, target_y = SEARCH_PATH[_next_waypoint_index]

    status = navigation.goTo(
        target_x,
        target_y,
        stop_condition=has_target
    )

    if status == STATUS_STOP_CONDITION:
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
    
