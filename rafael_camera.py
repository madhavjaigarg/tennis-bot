"""
RoboSports Camera Interface
Pybricks hub side — talks to the OpenMV H7 Plus over UART via
Anton's Mindstorms uRemote library.

Only job: ask the camera for the biggest detected ball and hand
back plain numbers. No Cartesian conversion, no ball tracking
logic, no target selection — that's rafael_ball_tracking.py and
rafael_target_manager.py.

Requires:
    - uremote.py copied onto the hub project.
    - Pybricks firmware patched with UARTDevice support (uRemote
      repo has the firmware build — standard Pybricks firmware
      does not include this).
    - The OpenMV side running the matching server script (see
      openmv_ball_server.py) with a "ball" command defined.

Wiring: cross TX/RX between hub and OpenMV, common GND, hub uses
an input port (this file assumes Port.C — confirm/change to
match how you actually wired it).
"""

from pybricks.parameters import Port
from uremote import uRemote, uRemoteError


# ============================================================
# CONNECTION
# ============================================================

# TODO: confirm this matches the port the OpenMV is actually
# wired to.
CAMERA_PORT = Port.C

ur = uRemote(CAMERA_PORT)


# ============================================================
# READ DETECTIONS
# ============================================================

def read_ball():
    """
    Ask the camera for the biggest currently-detected ball.

    Returns a dict:

        {
            "found": bool,
            "x": int,      # raw pixel x, 0 = left edge of frame
            "y": int,      # raw pixel y, 0 = top edge of frame
            "area": int,   # blob size in pixels
        }

    x/y are camera-frame pixel coordinates, NOT field
    coordinates — converting these into Cartesian positions
    happens in rafael_ball_tracking.py.

    If the camera doesn't respond (unplugged, crashed, still
    booting), found is False and x/y/area are 0 rather than
    raising — callers don't need a try/except for every read.
    """

    try:
        found, x, y, area = ur.call("ball")

    except uRemoteError:

        return {
            "found": False,
            "x": 0,
            "y": 0,
            "area": 0,
        }

    return {
        "found": bool(found),
        "x": x,
        "y": y,
        "area": area,
    }


# ============================================================
# DEBUGGING
# ============================================================

def print_ball():
    """
    Print the current ball reading. For testing the UART link.
    """

    print(read_ball())
