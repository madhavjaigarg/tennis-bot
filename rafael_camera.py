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

        "<found>,<x>,<y>,<area>,<marker>\n"

    e.g. "1,145,88,512,END\n" or "0,0,0,0,END\n" when no ball is
    seen. "END" is a fixed disposable trailer — see
    openmv_ball_server.py for why.

Only job of this file: get the latest line and hand back plain
numbers. No Cartesian conversion, no ball tracking logic — that
lives in rafael_ball_tracking.py.
"""

from pybricks.iodevices import UARTDevice
from pybricks.parameters import Port


# ============================================================
# CONNECTION
# ============================================================

CAMERA_PORT = Port.F
CAMERA_BAUDRATE = 115200

_uart = UARTDevice(CAMERA_PORT, baudrate=CAMERA_BAUDRATE, power_pin=2, timeout=1000)

# ============================================================
# READ DETECTIONS
# ============================================================

_last_ball = {"found": False, "x": 0, "y": 0, "area": 0}


def _parse_line(line):
    """
    Parse one "found,x,y,area,END" line (as BYTES, not str —
    Pybricks MicroPython doesn't include bytes.decode(), so we
    never convert to str at all and just work with bytes
    directly).

    Returns None if the line is malformed (torn packet, partial
    read, garbage), or if the trailing "END" marker is missing
    or cut short — a torn tail read clips this disposable marker
    first, before it can clip real data.
    """

    parts = line.split(b",")

    if len(parts) != 5:
        return None

    if parts[4] != b"END":
        return None

    try:
        found, x, y, area = (int(p) for p in parts[:4])
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

    # split(b"\n") always leaves the LAST element as either an
    # empty string (raw ended cleanly on a newline) or a torn,
    # still-being-written line (raw was read mid-transmission —
    # this is what was truncating "area", the last field before
    # the newline). Either way, drop it before parsing anything.
    lines = raw.split(b"\n")[:-1]

    # Walk backwards so we use the newest complete line and
    # discard any backlog — for real-time control we only want
    # the latest state.
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
