"""A finished reaction: take the round-bottom flask off the heating mantle and put it down.

On the bench:
- a heating mantle for a 250 mL flask (aluminium housing 15.5 cm across, 8.6 cm tall, with a tan fabric-lined bowl), the
  flask sitting in its bowl with its neck up (condenser and clamp already removed; contents not modelled)
- a 250 mL round-bottom flask (borosilicate glass: an 85 mm sphere, a 24/29 neck 25 mm across and 50 mm above the
  sphere, 110 g)
- a natural cork ring (90 mm outside, 60 mm inside, 20 mm tall) on the bare bench: the stand a round-bottom flask is set
  in, because on the flat bench it rolls
- a 250 mL beaker, an ordinary bench item

The instruction gives only the goal. A round-bottom flask set on a flat bench rolls over and off it; every chemist sets
it in a cork ring (or back in a clamp). The robot has to see that the flask is round and use the ring.
"""

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
import mech  # noqa: E402

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box, Cylinder, Sphere  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
CONTROL_HZ = 20.0
DEBUG = False

# -- the flask -----------------------------------------------------------------------------------
R = 0.0425  # the sphere's radius (85 mm)
NECK_R, NECK_ABOVE = 0.0125, 0.050  # neck 25 mm across, 50 mm above the sphere's top
NECK_LO = 0.030  # the neck's lower end, inside the sphere (m above its centre)
NECK_TOP = R + NECK_ABOVE  # the neck's top above the sphere's centre
FLASK_KG = 0.11
GLASS = (0.78, 0.88, 0.92)
GRASP_UP = R + NECK_ABOVE / 2  # grip the neck's middle: this far above the sphere's centre

# -- the heating mantle ----------------------------------------------------------------------------
MANTLE_C = (0.45, -0.20)
M_R_OUT, M_R_FABRIC = 0.0775, 0.0675
M_FLOOR = 0.030  # the base disc's thickness
M_LAYER = 0.008
M_LAYERS = 7  # the bowl's stacked rings (top of the mantle 8.6 cm above the bench)
M_TOP = M_FLOOR + M_LAYER * M_LAYERS
BOWL_R = 0.044  # the bowl's radius, a little over the flask's
N_SEG = 12
ALU, FABRIC = (0.74, 0.75, 0.78), (0.72, 0.62, 0.42)


def _bowl_inner(j):
    """The inner radius of bowl ring j: the bowl's profile at the ring's middle."""
    d = min(M_LAYER * j + M_LAYER / 2, BOWL_R)
    return math.sqrt(max(2 * BOWL_R * d - d * d, 0.0))


def _rest_in_bowl():
    """The flask's centre height (above the bench) when it rests on the bowl's rings."""
    best = M_FLOOR + R
    for j in range(M_LAYERS):
        r = _bowl_inner(j)
        if r < R:
            best = max(best, M_FLOOR + M_LAYER * j + math.sqrt(R * R - r * r))
    return best


FLASK_ZC = _rest_in_bowl() + 0.0005  # the sphere's centre above the bench at the start

# -- the cork ring -----------------------------------------------------------------------------------
RING_C = (0.45, -0.02)
RING_OUT, RING_IN, RING_H = 0.045, 0.030, 0.020
RING_SEG = 16
RING_ZC = RING_H + math.sqrt(R * R - RING_IN * RING_IN)  # the flask centre's height resting in the ring
CORK = (0.72, 0.54, 0.34)
BEAKER_C = (0.45, 0.20)

# -- scoring thresholds -------------------------------------------------------------------------------
MOVED_M = 0.03  # moved: the flask's centre at least this far from where it started
IN_RING_M = 0.015  # the sphere's centre within this of the ring's axis (horizontally)
RING_Z_TOL = 0.010  # ... and this of the height it sits at in the ring
UPRIGHT_COS = math.cos(math.radians(20.0))  # neck within 20 degrees of vertical
BENCH_M = 0.003  # the sphere's lowest point this close to the bench is resting on it
BENCH_S = 0.5  # ... for this long, after it was moved: set down on the bare bench
DROP_STEP_M = 0.03  # a drop: the flask's centre falls this far in one control step (0.6 m/s)
STILL_S, STILL_M = 1.0, 0.003
LET_GO_M = 0.10  # both TCPs at least this far from the sphere's centre

_W = int(STILL_S * CONTROL_HZ)

# -- objects ------------------------------------------------------------------------------------------
_mx, _my = MANTLE_C
_mantle_parts = []
for _j in range(M_LAYERS):
    _z = M_FLOOR + M_LAYER * (_j + 0.5) - M_FLOOR / 2  # relative to the base disc's centre
    _ri = _bowl_inner(_j)
    for _k in range(N_SEG):
        _a = 2 * math.pi * _k / N_SEG
        _depth = M_R_FABRIC - _ri
        _rc = (_ri + M_R_FABRIC) / 2
        _mantle_parts.append(((_depth, 2 * M_R_FABRIC * math.tan(math.pi / N_SEG), M_LAYER),
                              (_rc * math.cos(_a), _rc * math.sin(_a), _z), FABRIC, (0.0, 0.0, math.degrees(_a))))
        _rc2 = (M_R_FABRIC + M_R_OUT) / 2 - 0.0005
        _mantle_parts.append((((M_R_OUT - M_R_FABRIC) + 0.001, 2 * M_R_OUT * math.tan(math.pi / N_SEG), M_LAYER),
                              (_rc2 * math.cos(_a), _rc2 * math.sin(_a), _z), ALU, (0.0, 0.0, math.degrees(_a))))

_rc = (RING_IN + RING_OUT) / 2
_ring_parts = []
for _k in range(1, RING_SEG):
    _a = 2 * math.pi * _k / RING_SEG
    _ring_parts.append(((RING_OUT - RING_IN, 2 * RING_OUT * math.tan(math.pi / RING_SEG), RING_H),
                        (_rc * math.cos(_a) - _rc, _rc * math.sin(_a), 0.0), CORK, (0.0, 0.0, math.degrees(_a))))

OBJECTS = [
    Cylinder("mantle", radius=M_R_OUT, height=M_FLOOR, pos=(_mx, _my, TOP + M_FLOOR / 2), color=ALU),
    Box("cork_ring", size=(RING_OUT - RING_IN, 2 * RING_OUT * math.tan(math.pi / RING_SEG), RING_H),
        pos=(RING_C[0] + _rc, RING_C[1], TOP + RING_H / 2), color=CORK),
    Cylinder("beaker", radius=0.035, height=0.095, pos=(*BEAKER_C, TOP + 0.0475), color=(0.80, 0.90, 0.95), mass=0.12),
    Sphere("flask", radius=R, pos=(_mx, _my, TOP + FLASK_ZC), color=GLASS, mass=FLASK_KG),
    Box("flask_mark", size=(0.006, 0.006, 0.006), pos=(_mx, _my, TOP + FLASK_ZC), color=GLASS, mass=0.002),
]
mech.compound("mantle", _mantle_parts)
mech.compound("cork_ring", _ring_parts)
mech.compound("flask", [mech.cylinder_part(NECK_R, NECK_TOP - NECK_LO, (0.0, 0.0, (NECK_TOP + NECK_LO) / 2), GLASS)])
mech.mass_centre("flask", (0.0, 0.0, 0.0))
mech.weld("flask_mark", parent="flask")


@mech.on_tick
def _reset(sim):
    if sim.t <= 1.5 / CONTROL_HZ:
        _state.clear()
        _state.update(_fresh())


# -- measurements -------------------------------------------------------------------------------------


def _rot(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def centre(objects):
    """The sphere's centre (the welded marker sits at it)."""
    return np.asarray(objects["flask_mark"].pos, float)


def neck_cos(objects) -> float:
    """The neck's cosine with straight up."""
    return float((_rot(objects["flask"].quat_xyzw) @ np.array([0.0, 0.0, 1.0]))[2])


def in_ring(objects) -> bool:
    c = centre(objects)
    return (math.hypot(c[0] - RING_C[0], c[1] - RING_C[1]) <= IN_RING_M
            and abs(c[2] - (TOP + RING_ZC)) <= RING_Z_TOL and neck_cos(objects) >= UPRIGHT_COS)


def on_bench(objects) -> bool:
    """The sphere resting on the bare bench: its lowest point within 3 mm of the bench top."""
    c = centre(objects)
    return c[2] - R - TOP <= BENCH_M and c[2] > TOP - 0.02


def let_go(objects) -> bool:
    c = centre(objects)
    return all(np.linalg.norm(np.asarray(objects[a].pos, float) - c) >= LET_GO_M for a in ("left_tcp", "right_tcp"))


# -- scoring --------------------------------------------------------------------------------------------


def _fresh():
    return dict(start=None, lifted=False, bench=0, prev=None, violated=False, why="", hist=[], n=0)


_state = _fresh()


def still(st) -> bool:
    h = st["hist"]
    if len(h) < _W:
        return False
    a = np.asarray(h[-_W:])
    return bool(np.all(np.linalg.norm(a - a[-1], axis=1) < STILL_M))


def score(objects):
    """1.0: the flask was moved out of the mantle, was never set down on the bare bench (resting on it for 0.5 s) nor
    dropped (its centre falling 3 cm or more in one step), and at the end rests in the cork ring (the sphere's centre
    within 15 mm of the ring's axis and 1 cm of its seated height, the neck within 20 degrees of vertical), still for
    1 s, both grippers 10 cm or more from it. Otherwise 0: doing nothing, set down on the bench, dropped, off the
    table, left in the mantle, anywhere but the ring, still held."""
    st = _state
    c = centre(objects)
    if st["start"] is None:
        st["start"] = c.copy()
    st["n"] += 1
    st["hist"] = (st["hist"] + [c.copy()])[-(_W + 5):]
    if float(np.linalg.norm(c - st["start"])) >= MOVED_M:
        st["lifted"] = True
    if st["lifted"]:
        if st["prev"] is not None and c[2] - st["prev"] <= -DROP_STEP_M:
            st["violated"], st["why"] = True, "dropped"
        if on_bench(objects):
            st["bench"] += 1
            if st["bench"] >= BENCH_S * CONTROL_HZ:
                st["violated"], st["why"] = True, "set down on the bare bench"
        else:
            st["bench"] = 0
    st["prev"] = float(c[2])
    if DEBUG and st["n"] % 20 == 0:
        print(f"DBG t={st['n'] / CONTROL_HZ:.0f} c=({c[0]:.3f},{c[1]:.3f},{c[2] - TOP:.4f}) neck_cos={neck_cos(objects):.2f} "
              f"lifted={st['lifted']} ring={in_ring(objects)} bench={on_bench(objects)} still={still(st)} "
              f"let_go={let_go(objects)} viol={st['why']}", flush=True)
    if st["violated"] or not st["lifted"] or c[2] < TOP - 0.05:
        return 0.0
    return 1.0 if (in_ring(objects) and still(st) and let_go(objects)) else 0.0


# -- scripted demo -----------------------------------------------------------------------------------------
# The right arm grips the neck sideways (the gripper pointing forward, fingers closing across the neck), lifts the
# flask clear of the mantle's rim, carries it over the ring, lowers it in, lets go and backs out.

TILT = math.pi / 2
ZG = TOP + FLASK_ZC + GRASP_UP  # the grip on the neck in the mantle
LIFT = M_TOP - (FLASK_ZC - R) + 0.025  # lift to bring the sphere's bottom 2.5 cm above the mantle's rim
ZR = TOP + RING_ZC + GRASP_UP + 0.002  # the grip when the flask sits in the ring (2 mm to spare)
BACK = 0.10


def _t(d, p, fing, label):
    return mech.tilted(d, p, fing, label, yaw=0.0, tilt=TILT)


def demo():
    mx, my = MANTLE_C
    rx, ry = RING_C
    right = [
        _t(3.0, (mx - BACK, my, ZG + 0.02), FINGER_OPEN, "forward-pointing grip, beside the neck"),
        _t(2.0, (mx, my, ZG), FINGER_OPEN, "around the flask's neck"),
        _t(0.8, (mx, my, ZG), FINGER_CLOSED, "grip the neck"),
        _t(1.8, (mx, my, ZG + LIFT), FINGER_CLOSED, "lift it clear of the mantle"),
        _t(3.0, (rx, ry, ZG + LIFT), FINGER_CLOSED, "over the cork ring"),
        _t(2.5, (rx, ry, ZR), FINGER_CLOSED, "lower it into the ring"),
        _t(0.8, (rx, ry, ZR), FINGER_OPEN, "let go"),
        _t(1.5, (rx - BACK, ry, ZR), FINGER_OPEN, "back away"),
        _t(1.0, (rx - BACK, ry, ZR + 0.06), FINGER_OPEN, "up"),
        Waypoint(2.0, (0.18, -0.27, TOP + 0.20), FINGER_OPEN, "park"),
        Waypoint(2.0, (0.18, -0.27, TOP + 0.20), FINGER_OPEN, "still"),
    ]
    left = [Waypoint(sum(w.duration for w in right), (0.18, 0.27, TOP + 0.20), FINGER_OPEN, "stay parked")]
    return {"left": left, "right": right}
