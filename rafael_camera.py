"""
RoboSports Camera Interface
Pybricks hub side — reads ball detections streamed continuously
from the OpenMV H7 Plus over a plain UART link.

No third-party library — uses Pybricks' own
pybricks.iodevices.UARTDevice directly.

*** REQUIRES Pybricks firmware v4.0 or later on the hub ***
UARTDevice support for SPIKE-style hubs (Prime Hub, Robot
Inventor, Technic Hub) was added in Pybricks v4. On older
firmware this class does not exist on these hubs and the import
below will fail.

Protocol (one-way — camera talks continuously, hub just reads
whatever's newest):

    OpenMV writes one ASCII line per frame:

        "<found>,<x>,<y>,<area>\n"

    e.g. "1,145,88,512\n" or "0,0,0,0\n" when no ball is seen.
    See openmv_ball_server.py for the camera-side script.

Only job of this file: get the latest line and hand back plain
numbers. No Cartesian conversion, no ball tracking logic — that
lives in rafael_ball_tracking.py.
"""

from pybricks.iodevices import UARTDevice
from pybricks.parameters import Port


# ============================================================
# CONNECTION
# ============================================================

# TODO: confirm this matches the port the OpenMV is actually
# wired to.
CAMERA_PORT = Port.F
CAMERA_BAUDRATE = 115200

_uart = UARTDevice(CAMERA_PORT, baudrate=CAMERA_BAUDRATE,power_pin=2)


# ============================================================
# READ DETECTIONS
# ============================================================

_last_ball = {"found": False, "x": 0, "y": 0, "area": 0}


def _parse_line(line):
    """
    Parse one "found,x,y,area" line (as BYTES, not str — Pybricks
    MicroPython doesn't include bytes.decode(), so we never
    convert to str at all and just work with bytes directly).

    Returns None if the line is malformed (torn packet, partial
    read, garbage) instead of raising, since UART data can
    legitimately arrive split across reads.
    """

    parts = line.split(b",")

    if len(parts) != 4:
        return None

    try:
        found, x, y, area = (int(p) for p in parts)
    except ValueError:
        return None

    return {
        "found": bool(found),
        "x": x,
        "y": y,
        "area": area,
    }


def read_ball():
    """
    Return the most recent ball detection from the camera.

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

    Non-blocking: if no new complete line has arrived since the
    last call, returns the last known reading. If the camera has
    never sent a valid line yet, returns "not found".
    """

    global _last_ball

    raw = _uart.read_all()

    if not raw:
        return _last_ball

    lines = raw.split(b"\n")

    # Walk backwards so we use the newest complete line and
    # discard any backlog (and any trailing partial line) —
    # for real-time control we only want the latest state.
    for line in reversed(lines):

        line = line.strip()

        if not line:
            continue

        parsed = _parse_line(line)

        if parsed is not None:
            _last_ball = parsed
            break

    return _last_ball


# ============================================================
# DEBUGGING
# ============================================================

def print_ball():
    """
    Print the current ball reading. For testing the UART link.
    """

    print(read_ball())
