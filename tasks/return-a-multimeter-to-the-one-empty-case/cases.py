"""Return a multimeter to the one empty case on a shelf of three identical hard cases.

On the bench:
- three identical hard cases (ABS, 14.5 x 20 x 5.4 cm, black, foam-lined floor) side by side, each with a sliding lid
  (14.5 x 20 cm, 8 mm) that has a finger knob at its front edge and slides straight back 15.5 cm to open
- two of the cases hold a multimeter (15 x 7.5 x 3.6 cm, yellow holster, display, dial, three input jacks); the third
  is empty. The lids are opaque: the robot has to open cases to see which one is empty
- the returned multimeter (long side across the bench) lies on the bench in front of the cases

VARIANT (0, 1 or 2) picks which case is empty: 0 the middle one, 1 the right-hand one, 2 the left-hand one.
Set by the CASES_VARIANT environment variable (default 0, the layout in the screenshot). An evaluation runs one trial per
layout and averages the three scores, so the empty case can't be memorised.
"""

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
import mech  # noqa: E402

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
CONTROL_HZ = 20.0
DEBUG = False
VARIANT = int(__import__('os').environ.get('CASES_VARIANT', '0'))  # which case is empty: 0 middle, 1 right-hand, 2 left-hand
EMPTY = (1, 2, 0)[VARIANT]  # index into CASE_Y (0 left, 1 middle, 2 right)

# -- the cases ---------------------------------------------------------------------------------
CX = 0.30  # the cases' centre, forward
CASE_Y = (0.23, 0.0, -0.23)  # left, middle, right
CASE = (0.145, 0.20)  # outer x (depth), y (width)
FLOOR_T, WALL_H, WALL_T = 0.010, 0.044, 0.015
REC = (0.115, 0.17)  # the foam recess's inside: x, y
ZF = TOP + FLOOR_T  # foam floor top
WALL_TOP = ZF + WALL_H
LID_T, LID_GAP = 0.008, 0.0005
LID_Z = WALL_TOP + LID_GAP + LID_T / 2
KNOB = (0.03, 0.04, 0.026)  # x, y, z: the finger knob on the lid's front edge
KNOB_DX = -CASE[0] / 2 + KNOB[0] / 2
SLIDE = 0.155  # the lid slides this far back to open
LID_KG = 0.12
DARK, FOAM, LIDC, KNOBC = (0.10, 0.10, 0.12), (0.20, 0.22, 0.27), (0.17, 0.18, 0.21), (0.72, 0.73, 0.76)

# -- the multimeter (long side along y) ---------------------------------------------------------
M_SIZE = (0.075, 0.15, 0.036)
M_KG = 0.20
M_START = (0.12, 0.0)  # the returned meter on the bench
YEL, SCREEN, BLACK, RED = (0.95, 0.76, 0.08), (0.55, 0.63, 0.55), (0.08, 0.08, 0.09), (0.85, 0.08, 0.06)
METER_PARTS = [
    ((0.045, 0.05, 0.002), (0.0, 0.035, 0.019), SCREEN),  # display window
    ((0.03, 0.03, 0.004), (0.0, -0.025, 0.020), BLACK, (0.0, 0.0, 45.0)),  # range dial
    ((0.010, 0.010, 0.004), (-0.022, -0.062, 0.020), BLACK),  # input jacks
    ((0.010, 0.010, 0.004), (0.0, -0.062, 0.020), BLACK),
    ((0.010, 0.010, 0.004), (0.022, -0.062, 0.020), RED),
]

# -- the rule and thresholds ----------------------------------------------------------------------
LID_OPEN_M = 0.005  # a lid is closed when within 5 mm of its closed position
DISTURB_M = 0.02  # a stored meter moved this far (cm) counts as disturbed
IN_CASE_Z = TOP + 0.072  # a meter whose centre is lower than this, over a case's recess, is in it (or on a meter in it)
EMPTY_Z = TOP + 0.040
TILT_COS = math.cos(math.radians(20.0))
STILL_S, STILL_M = 1.0, 0.003
LET_GO_M = 0.07
VIOL_S = 0.5


def _layout():
    empties = EMPTY
    return [i for i in range(3) if i != empties]


STORED = dict(zip(("stored_a", "stored_b"), _layout()))  # object name -> case index

OBJECTS = []
for _i, _y in enumerate(CASE_Y):
    OBJECTS.append(Box(f"case{_i}", size=(CASE[0], CASE[1], FLOOR_T), pos=(CX, _y, TOP + FLOOR_T / 2), color=FOAM))
    OBJECTS.append(Box(f"lid{_i}", size=(CASE[0], CASE[1], LID_T), pos=(CX, _y, LID_Z), color=LIDC, mass=LID_KG))
OBJECTS.append(Box("meter", size=M_SIZE, pos=(M_START[0], M_START[1], TOP + M_SIZE[2] / 2 + 0.0005), color=YEL, mass=M_KG))
for _n, _i in STORED.items():
    OBJECTS.append(Box(_n, size=M_SIZE, pos=(CX, CASE_Y[_i], ZF + M_SIZE[2] / 2 + 0.0005), color=YEL, mass=M_KG))

for _i, _y in enumerate(CASE_Y):
    _wz = FLOOR_T / 2 + WALL_H / 2
    mech.compound(f"case{_i}", [
        ((WALL_T, CASE[1], WALL_H), (CASE[0] / 2 - WALL_T / 2, 0.0, _wz), DARK),
        ((WALL_T, CASE[1], WALL_H), (-CASE[0] / 2 + WALL_T / 2, 0.0, _wz), DARK),
        ((REC[0], WALL_T, WALL_H), (0.0, REC[1] / 2 + WALL_T / 2, _wz), DARK),
        ((REC[0], WALL_T, WALL_H), (0.0, -REC[1] / 2 - WALL_T / 2, _wz), DARK),
    ])
    mech.compound(f"lid{_i}", [(KNOB, (KNOB_DX, 0.0, LID_T / 2 + KNOB[2] / 2), KNOBC)])
    mech.prismatic(f"lid{_i}", "X", (CX, _y, LID_Z), lower=0.0, upper=SLIDE, damping=5.0)
for _n in ("meter", *STORED):
    mech.compound(_n, METER_PARTS)


@mech.on_tick
def _reset(sim):
    if sim.t <= 1.5 / CONTROL_HZ:
        _state.clear()
        _state.update(_fresh())


# -- measurements ---------------------------------------------------------------------------------


def _rot(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def over_recess(p, i) -> bool:
    return abs(p[0] - CX) <= REC[0] / 2 and abs(p[1] - CASE_Y[i]) <= REC[1] / 2


def lid_open(objects, i) -> bool:
    """The lid has slid back more than 5 mm from its closed position."""
    return float(objects[f"lid{i}"].pos[0]) - CX > LID_OPEN_M


def n_open(objects) -> int:
    return sum(lid_open(objects, i) for i in range(3))


def meter_in_empty(objects) -> bool:
    m = objects["meter"]
    p = np.asarray(m.pos, float)
    flat = float((_rot(m.quat_xyzw) @ np.array([0.0, 0.0, 1.0]))[2]) >= TILT_COS
    return over_recess(p, EMPTY) and p[2] <= EMPTY_Z and flat


def meter_in_occupied(objects) -> bool:
    p = np.asarray(objects["meter"].pos, float)
    return any(over_recess(p, i) and p[2] <= IN_CASE_Z for i in STORED.values())


def stored_disturbed(objects) -> bool:
    return any(float(np.linalg.norm(np.asarray(objects[n].pos, float) - np.asarray(st0)))
               >= DISTURB_M for n, st0 in _state["stored0"].items())


def let_go(objects) -> bool:
    m = np.asarray(objects["meter"].pos, float)
    return all(np.linalg.norm(np.asarray(objects[a].pos, float) - m) >= LET_GO_M for a in ("left_tcp", "right_tcp"))


# -- scoring -------------------------------------------------------------------------------------


def _fresh():
    return dict(n=0, hist=[], bad_steps=0, violated=False, stored0={})


_state = _fresh()
_WINDOW = int(STILL_S * CONTROL_HZ)


def still(st) -> bool:
    h = st["hist"]
    if len(h) < _WINDOW:
        return False
    a = np.asarray(h[-_WINDOW:])
    return bool(np.all(np.linalg.norm(a - a[-1], axis=1) < STILL_M))


def score(objects):
    """0: never put in the empty case (doing nothing included); the meter put into an occupied case or on a meter
    in it (rests there 0.5 s: violation, latched); not still 1 s and let go. Otherwise, with the meter lying flat in
    the empty case, still for 1 s, both grippers at least 7 cm from it: 1.0 with all three lids closed (within 5 mm
    of their closed position), 0.5 with one lid open, 0.25 with two or three open. A stored meter moved 2 cm or more
    caps the score at 0.25."""
    st = _state
    st["n"] += 1
    if not st["stored0"]:
        st["stored0"] = {n: np.asarray(objects[n].pos, float).copy() for n in STORED}
    st["hist"] = (st["hist"] + [np.asarray(objects["meter"].pos, float)])[-(_WINDOW + 5):]
    if meter_in_occupied(objects):
        st["bad_steps"] += 1
        if st["bad_steps"] >= VIOL_S * CONTROL_HZ:
            st["violated"] = True
    else:
        st["bad_steps"] = 0
    if DEBUG and st["n"] % 20 == 0:
        m = np.asarray(objects["meter"].pos, float)
        lids = [round(float(objects[f"lid{i}"].pos[0]) - CX, 3) for i in range(3)]
        print(f"DBG t={st['n'] / CONTROL_HZ:.0f} meter=({m[0]:.3f},{m[1]:.3f},{m[2] - TOP:.3f}) lids={lids} "
              f"in_empty={meter_in_empty(objects)} still={still(st)} let_go={let_go(objects)} viol={st['violated']} "
              f"disturbed={stored_disturbed(objects)}", flush=True)
    if st["violated"] or not meter_in_empty(objects) or not (still(st) and let_go(objects)):
        return 0.0
    s = {0: 1.0, 1: 0.5}.get(n_open(objects), 0.25)
    return min(s, 0.25) if stored_disturbed(objects) else s


# -- scripted demo ------------------------------------------------------------------------------------
# Opens the cases from the left until it reaches the empty one (occupied ones are shut again), puts the meter in, shuts it.

PARK = {"left": (0.18, 0.27, TOP + 0.20), "right": (0.18, -0.27, TOP + 0.20)}
ARM_OF = ("left", "right", "right")
KX0 = CX + KNOB_DX  # the knob's x when closed
KZ = LID_Z + LID_T / 2 + KNOB[2] / 2 + 0.005  # TCP height on the knob
O, C = FINGER_OPEN, FINGER_CLOSED
Y = math.pi / 2  # fingers close along x: across the knob (the lid slides along x) and across the meter's 7.5 cm width


def _side(i):
    return 0.06 if i == 0 else -0.06


def _lid_open_wps(i):
    y = CASE_Y[i]
    return [
        Waypoint(2.5, (KX0, y, KZ + 0.03), O, f"over case {i}'s lid knob", Y),
        Waypoint(1.0, (KX0, y, KZ), O, "down to the knob", Y),
        Waypoint(0.6, (KX0, y, KZ), C, "pinch the knob", Y),
        Waypoint(3.5, (KX0 + SLIDE, y, KZ), C, "slide the lid back", Y),
        Waypoint(0.6, (KX0 + SLIDE, y, KZ), O, "let go of the knob", Y),
        Waypoint(1.0, (KX0 + SLIDE, y + _side(i), KZ), O, "slide sideways off the knob", Y),
        Waypoint(2.0, PARK[ARM_OF[i]], O, "park"),
    ]


def _lid_close_wps(i):
    y = CASE_Y[i]
    return [
        Waypoint(2.5, (KX0 + SLIDE, y, KZ + 0.03), O, f"over case {i}'s lid knob", Y),
        Waypoint(1.0, (KX0 + SLIDE, y, KZ), O, "down to the knob", Y),
        Waypoint(0.6, (KX0 + SLIDE, y, KZ), C, "pinch the knob", Y),
        Waypoint(3.5, (KX0, y, KZ), C, "slide the lid shut", Y),
        Waypoint(0.6, (KX0, y, KZ), O, "let go of the knob", Y),
        Waypoint(1.0, (KX0, y + _side(i), KZ), O, "slide sideways off the knob", Y),
        Waypoint(2.0, PARK[ARM_OF[i]], O, "park"),
    ]


def _meter_wps(i):
    YM = -Y if ARM_OF[i] == "left" else Y  # same grip, turned the short way round
    mx, my = M_START
    y = CASE_Y[i]
    gz = TOP + M_SIZE[2] / 2 + 0.008  # grip a little above the meter's middle
    hi = TOP + 0.095
    drop = ZF + M_SIZE[2] / 2 + 0.008 + 0.004
    return [
        Waypoint(2.5, (mx, my, TOP + 0.075), O, "over the meter", YM),
        Waypoint(1.2, (mx, my, gz), O, "down beside the meter", YM),
        Waypoint(0.6, (mx, my, gz), C, "grip it", YM),
        Waypoint(1.5, (mx, my, hi), C, "lift it", YM),
        Waypoint(2.0, (mx, y, hi), C, "over to the empty case's lane", YM),
        Waypoint(2.0, (CX, y, hi), C, "over the empty case", YM),
        Waypoint(1.5, (CX, y, drop), C, "down into the recess", YM),
        Waypoint(0.6, (CX, y, drop), O, "let go", YM),
        Waypoint(1.0, (CX, y, hi), O, "up", YM),
        Waypoint(2.0, PARK[ARM_OF[i]], O, "park"),
    ]


def demo():
    plans = {"left": [], "right": []}

    def add(arm, wps):
        d = sum(w.duration for w in wps)
        other = "right" if arm == "left" else "left"
        plans[arm] += wps
        plans[other].append(Waypoint(d, PARK[other], O, "stay parked"))

    for i in range(EMPTY + 1):
        add(ARM_OF[i], _lid_open_wps(i))
        if i < EMPTY:
            add(ARM_OF[i], _lid_close_wps(i))
    add(ARM_OF[EMPTY], _meter_wps(EMPTY))
    add(ARM_OF[EMPTY], _lid_close_wps(EMPTY))
    plans["left"].append(Waypoint(2.0, PARK["left"], O, "still"))
    plans["right"].append(Waypoint(2.0, PARK["right"], O, "still"))
    return plans
