"""A set agarose gel in its casting tray: put it in the electrophoresis tank, wells at the black (negative) end.

On the table:
- a horizontal mini-gel tank (clear acrylic, 30 x 15 x 6 cm, long axis along y): a buffer chamber at each end with a
  platinum electrode wire, a raised gel deck between them, and a coloured end cap per chamber: red (+) at the left end,
  black (-) at the right end (as seen from the robots)
- a UV-transparent gel tray (10 x 7 cm, 1.2 cm side walls, open ends) in front of the tank, turned 90 degrees to it,
  holding a set 0.6 cm agarose gel with a row of 8 wells at the end that points away from the robots
The instruction gives only the goal. DNA is negatively charged and runs toward the red (+) electrode ("run to red"), so
the wells must lie at the black end; with the wells at the red end the samples run out of the gel into the buffer.
Buffer is poured after the gel is in place, as standard, so no liquid is modelled.
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

ACRYLIC = (0.80, 0.88, 0.93)
RED, BLACK, SILVER = (0.85, 0.05, 0.05), (0.06, 0.06, 0.07), (0.80, 0.80, 0.83)
RED_CAP, BLACK_CAP = (0.75, 0.10, 0.10), (0.16, 0.16, 0.18)
GEL_RGB, WELL_RGB = (0.80, 0.93, 0.84), (0.03, 0.03, 0.04)

# -- the tank (fixed): floor, side walls, end caps, deck, electrodes, coloured bars and jacks --------------------------
TANK_C = (0.38, 0.0)
TANK_X, TANK_Y, WALL_H = 0.15, 0.30, 0.06
FLOOR_T, WALL_T, CAP_T = 0.003, 0.003, 0.012
DECK_SIZE = (0.060, 0.160, 0.020)  # across x, along y, height above the floor
DECK_TOP = TOP + FLOOR_T + DECK_SIZE[2]
_FZ = FLOOR_T / 2  # the floor's centre height above the table


def _p(size, x, y, z_bottom, rgb):  # a part, from its world-height bottom (above the table) to an offset from the floor
    return (size, (x, y, TOP + z_bottom + size[2] / 2 - (TOP + _FZ)), rgb)


_CAP_Y = TANK_Y / 2 - CAP_T / 2
_WIRE_Y = TANK_Y / 2 - CAP_T - 0.005
TANK_PARTS = [
    _p((WALL_T, TANK_Y, WALL_H), TANK_X / 2 - WALL_T / 2, 0.0, 0.0, ACRYLIC),
    _p((WALL_T, TANK_Y, WALL_H), -(TANK_X / 2 - WALL_T / 2), 0.0, 0.0, ACRYLIC),
    _p((TANK_X, CAP_T, WALL_H), 0.0, _CAP_Y, 0.0, RED_CAP),  # end caps (electrode housings), coloured
    _p((TANK_X, CAP_T, WALL_H), 0.0, -_CAP_Y, 0.0, BLACK_CAP),
    _p(DECK_SIZE, 0.0, 0.0, FLOOR_T, ACRYLIC),
    _p((TANK_X, CAP_T, 0.006), 0.0, _CAP_Y, WALL_H, RED),  # red (+) bar along the left end cap
    _p((TANK_X, CAP_T, 0.006), 0.0, -_CAP_Y, WALL_H, BLACK),  # black (-) bar along the right end cap
    _p((0.02, 0.012, 0.012), 0.045, _CAP_Y, WALL_H + 0.006, RED),  # red jack
    _p((0.02, 0.012, 0.012), 0.045, -_CAP_Y, WALL_H + 0.006, BLACK),  # black jack
    _p((0.13, 0.0015, 0.0015), 0.0, _WIRE_Y, FLOOR_T + 0.006, SILVER),  # platinum electrode wires
    _p((0.13, 0.0015, 0.0015), 0.0, -_WIRE_Y, FLOOR_T + 0.006, SILVER),
]
for _sy in (1, -1):
    for _sx in (1, -1):
        TANK_PARTS.append(_p((0.002, 0.002, 0.006), _sx * 0.062, _sy * _WIRE_Y, FLOOR_T, SILVER))  # wire posts
for _sy in (1, -1):  # black tape marks across the deck, 5.6 cm either side of its middle (the tray's ends go between)
    TANK_PARTS.append(_p((DECK_SIZE[0], 0.003, 0.0005), 0.0, _sy * 0.0575, FLOOR_T + DECK_SIZE[2], BLACK))

# -- the tray and its gel ----------------------------------------------------------------------------------
TRAY_L, TRAY_W, TRAY_H, TRAY_FLOOR, TRAY_WALL = 0.10, 0.07, 0.012, 0.003, 0.002
TRAY_START = (0.19, 0.0)  # centre; long axis along x (turned 90 degrees to the tank), wells toward +x
TRAY_KG, GEL_KG = 0.04, 0.045
GEL_L, GEL_W, GEL_T = 0.099, 0.064, 0.006
SLAB_T = 0.0035
TRAY_PARTS = [((TRAY_L, TRAY_WALL, TRAY_H), (0.0, s * (TRAY_W / 2 - TRAY_WALL / 2), TRAY_H / 2 - TRAY_FLOOR / 2),
               ACRYLIC) for s in (1, -1)]
_ZG = TOP + TRAY_FLOOR + 0.0003  # the gel's bottom
_UP = GEL_T - SLAB_T  # the upper layer's thickness
_UZ = SLAB_T / 2 + _UP / 2  # its centre above the slab's centre
_UW = GEL_W
WELL_W, WELL_PITCH, WELL_U = 0.005, 0.0075, 0.005
_END_MARGIN = 0.009
_u_strip = GEL_L / 2 - _END_MARGIN - WELL_U / 2
_u_front = GEL_L / 2 - _END_MARGIN
GEL_PARTS = [
    ((_END_MARGIN, _UW, _UP), (GEL_L / 2 - _END_MARGIN / 2, 0.0, _UZ), GEL_RGB),  # end margin (wells' end)
    ((GEL_L - _END_MARGIN - WELL_U, _UW, _UP), (-GEL_L / 2 + (GEL_L - _END_MARGIN - WELL_U) / 2, 0.0, _UZ), GEL_RGB),
]
_edges = [(i - 3.5) * WELL_PITCH + s * WELL_W / 2 for i in range(8) for s in (-1, 1)]  # well edges, left to right
_pillars = [(-GEL_W / 2, _edges[0])] + [(_edges[2 * i + 1], _edges[2 * i + 2]) for i in range(7)] + [(_edges[-1], GEL_W / 2)]
for _a, _b in _pillars:
    GEL_PARTS.append(((WELL_U, _b - _a, _UP), (_u_strip, (_a + _b) / 2, _UZ), GEL_RGB))
for _i in range(8):  # the dark floor of each well, on the slab
    GEL_PARTS.append(((WELL_U, WELL_W, 0.0008), (_u_strip, (_i - 3.5) * WELL_PITCH, SLAB_T / 2 + 0.0004), WELL_RGB))

OBJECTS = [
    Box("tank", size=(TANK_X, TANK_Y, FLOOR_T), pos=(*TANK_C, TOP + _FZ), color=ACRYLIC),
    Box("tray", size=(TRAY_L, TRAY_W, TRAY_FLOOR), pos=(*TRAY_START, TOP + TRAY_FLOOR / 2 + 0.0002), color=ACRYLIC,
        mass=TRAY_KG),
    Box("gel", size=(GEL_L, GEL_W, SLAB_T), pos=(*TRAY_START, _ZG + SLAB_T / 2), color=GEL_RGB, mass=GEL_KG),
]
mech.compound("tank", TANK_PARTS)
mech.compound("tray", TRAY_PARTS)
mech.compound("gel", GEL_PARTS)

# -- the rule ---------------------------------------------------------------------------------
BLACK_DIR = np.array([0.0, -1.0])  # toward the black (-) end cap
SEAT_Z = DECK_TOP + (TRAY_H - TRAY_FLOOR) / 2 + TRAY_FLOOR / 2 - TRAY_FLOOR / 2  # tray's reported centre height, seated
SEAT_Z = DECK_TOP + TRAY_H / 2  # the tray's box spans its bottom to its wall tops: centre = bottom + 6 mm
SEAT_XY_M, SEAT_Z_M, SEAT_DEG, SEAT_TILT = 0.005, 0.002, 5.0, 3.0
REST_XY_M, REST_Z_M, REST_DEG, REST_TILT = 0.015, 0.005, 15.0, 10.0
GEL_OUT_U, GEL_OUT_V, GEL_OUT_Z, GEL_OUT_STEPS = 0.010, 0.006, 0.008, 3
STILL_S, STILL_M = 1.0, 0.003
LET_GO_M = 0.07
_WINDOW = int(STILL_S * CONTROL_HZ)


def _rot(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _fresh():
    return dict(hist=[], n=0, out_steps=0, violated=False)


_state = _fresh()


@mech.on_tick
def _reset(sim):
    if sim.t <= 1.5 / CONTROL_HZ:
        _state.clear()
        _state.update(_fresh())


def wells_dir(objects):
    """The direction the tray's wells end points, in the table plane (unit vector)."""
    d = _rot(objects["tray"].quat_xyzw) @ np.array([1.0, 0.0, 0.0])
    return d[:2] / max(float(np.linalg.norm(d[:2])), 1e-9)


def wells_angle_deg(objects) -> float:
    """The angle between the wells' direction and the black end's direction (0 = wells exactly at the black end)."""
    return math.degrees(math.acos(float(np.clip(np.dot(wells_dir(objects), BLACK_DIR), -1.0, 1.0))))


def tilt_deg(objects) -> float:
    up = _rot(objects["tray"].quat_xyzw) @ np.array([0.0, 0.0, 1.0])
    return math.degrees(math.acos(float(np.clip(up[2], -1.0, 1.0))))


def tray_pos(objects):
    return np.asarray(objects["tray"].pos, float)


def gel_in_tray(objects) -> bool:
    """The gel still lies in the tray: within 1 cm along the tray's length, 6 mm across and 8 mm in height of where it
    sits when the tray is level with the gel in it."""
    r = _rot(objects["tray"].quat_xyzw)
    d = r.T @ (np.asarray(objects["gel"].pos, float) - tray_pos(objects))
    return abs(d[0]) <= GEL_OUT_U and abs(d[1]) <= GEL_OUT_V and abs(d[2]) <= GEL_OUT_Z


def on_deck(objects, xy_m, z_m, tilt) -> bool:
    p = tray_pos(objects)
    return (np.linalg.norm(p[:2] - np.asarray(TANK_C)) <= xy_m and abs(p[2] - SEAT_Z) <= z_m
            and tilt_deg(objects) <= tilt)


def let_go(objects) -> bool:
    p = tray_pos(objects)
    return all(np.linalg.norm(np.asarray(objects[a].pos, float) - p) >= LET_GO_M for a in ("left_tcp", "right_tcp"))


def still(st) -> bool:
    h = st["hist"]
    if len(h) < _WINDOW:
        return False
    a = np.asarray(h[-_WINDOW:])
    return bool(np.all(np.linalg.norm(a - a[-1], axis=1) < STILL_M))


def score(objects):
    """1.0: the gel never out of the tray, and at the end the tray seated on the deck (centre within 5 mm of the deck's
    centre, bottom within 2 mm of the deck's top, within 3 degrees of level), its wells end within 5 degrees of
    pointing at the black end, the gel in the tray, still for 1 s, both grippers 7 cm clear. 0.5: the same but only
    resting on the deck (within 15 mm, 5 mm, 10 degrees) with the wells within 15 degrees of the black end. 0: doing
    nothing, wells nearer the red end or sideways, gel out of the tray at any time (latched), the tray off the deck,
    or still held."""
    st = _state
    st["n"] += 1
    st["hist"] = (st["hist"] + [tray_pos(objects)])[-(_WINDOW + 5):]
    st["out_steps"] = 0 if gel_in_tray(objects) else st["out_steps"] + 1
    if st["out_steps"] >= GEL_OUT_STEPS:
        st["violated"] = True
    ang = wells_angle_deg(objects)
    if DEBUG and st["n"] % 20 == 0:
        p, g = tray_pos(objects), np.asarray(objects["gel"].pos, float)
        print(f"DBG t={st['n'] / CONTROL_HZ:.0f} tray=({p[0]:.3f},{p[1]:.3f},{p[2] - TOP:.4f}) gel_d=({g[0] - p[0]:.3f},"
              f"{g[1] - p[1]:.3f},{g[2] - p[2]:.4f}) wells_ang={ang:.1f} tilt={tilt_deg(objects):.1f} in_tray={gel_in_tray(objects)} "
              f"seated={on_deck(objects, SEAT_XY_M, SEAT_Z_M, SEAT_TILT)} still={still(st)} let_go={let_go(objects)} viol={st['violated']}",
              flush=True)
    if st["violated"] or not (still(st) and let_go(objects)):
        return 0.0
    if on_deck(objects, SEAT_XY_M, SEAT_Z_M, SEAT_TILT) and ang <= SEAT_DEG:
        return 1.0
    if on_deck(objects, REST_XY_M, REST_Z_M, REST_TILT) and ang <= REST_DEG:
        return 0.5
    return 0.0


# -- scripted demo ----------------------------------------------------------------------------
# The right arm grips the tray's two long side walls at their middle, lifts it clear of the tank's wall, carries it over
# the deck turning it a quarter turn clockwise (the grip turns from yaw 180 to 90 degrees: the fingers are symmetric) (wells from +x to -y, the black end), lowers it onto the deck and lets go.

GRIP_Z = TOP + 0.0145  # pad tips 1.3 cm below: a millimetre above the table
CARRY_Z = TOP + 0.10
SEAT_GRIP_Z = DECK_TOP + 0.0145


def demo():
    sx, sy = TRAY_START
    tx, ty = TANK_C
    right = [
        Waypoint(2.0, (sx, sy, CARRY_Z - 0.03), FINGER_OPEN, "over the tray", math.pi),
        Waypoint(1.5, (sx, sy, GRIP_Z), FINGER_OPEN, "down beside the tray's long walls", math.pi),
        Waypoint(0.8, (sx, sy, GRIP_Z), FINGER_CLOSED, "grip the walls", math.pi),
        Waypoint(1.5, (sx, sy, CARRY_Z), FINGER_CLOSED, "lift it clear of the tank's wall", math.pi),
        Waypoint(3.5, (tx, ty, CARRY_Z), FINGER_CLOSED, "carry it over the deck, turning it a quarter turn", math.pi / 2),
        Waypoint(2.5, (tx, ty, SEAT_GRIP_Z), FINGER_CLOSED, "lower it onto the deck", math.pi / 2),
        Waypoint(0.8, (tx, ty, SEAT_GRIP_Z), FINGER_OPEN, "let go", math.pi / 2),
        Waypoint(1.2, (tx, ty, CARRY_Z), FINGER_OPEN, "up, clear of the tank", math.pi / 2),
        Waypoint(2.0, (0.18, -0.27, TOP + 0.20), FINGER_OPEN, "park"),
        Waypoint(2.0, (0.18, -0.27, TOP + 0.20), FINGER_OPEN, "still"),
    ]
    left = [Waypoint(sum(w.duration for w in right), (0.18, 0.27, TOP + 0.20), FINGER_OPEN, "stay parked")]
    return {"left": left, "right": right}
