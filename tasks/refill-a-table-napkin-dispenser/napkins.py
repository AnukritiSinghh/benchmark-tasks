"""Refill a table-top napkin dispenser: pull its spring-loaded follower back, drop in the new stack, let it close.

On the table:
- a stainless-steel open-top napkin dispenser (14 x 18 cm, 4.9 cm high sides): a floor and two side walls, a front
  plate with two retaining lips and a bottom bar (the lips overlap the stack's face by 1.6 cm), and a spring-loaded
  follower (a plate that rides along the floor, pushed toward the front by a spring, with a handle tab behind it).
  It starts empty with the follower forward against the front plate.
- a new stack of 250 interfolded beverage napkins (12.7 x 7.5 x 10 cm, 0.3 kg), standing on the table to the right.

To refill, the follower is pulled back against its spring and held, the stack is set in square between the side
walls and behind the front lips, and the follower is let go so it closes on the stack and presses it forward against
the lips. The robot's gripper opens only 9.4 cm, so the stack is gripped across its 7.5 cm thickness by its upper part,
above the side walls, where the fingers never enter the dispenser.
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

CHROME, STEEL, PAPER = (0.78, 0.79, 0.81), (0.62, 0.64, 0.67), (0.96, 0.96, 0.94)

# -- dispenser (all stainless plates 4 mm thick) ---------------------------------------------------------------
XC = 0.34  # the dispenser's centre line (x)
T = 0.004
FLOOR_TOP = TOP + T
INNER_W = 0.135  # between the side walls
WALL_H = 0.045  # side walls and lips stand this far above the floor
YF = -0.060  # the back face of the front plate (the lips): the stack's front face rests here
Y_END = 0.185  # the rear end of the floor and walls
FLOOR_Y = ((YF - T) + Y_END) / 2
FLOOR_L = Y_END - (YF - T)
LIP_W = 0.016  # each lip overlaps the stack's face by 1.6 cm (the opening between them is 10.3 cm wide)
BAR_H = 0.020  # the bottom bar of the front plate

# -- the stack ---------------------------------------------------------------------------------------------------
STACK = (0.127, 0.075, 0.100)  # x (width), y (thickness, toward the front), z (height)
STACK_KG = 0.3
STACK_C0 = (XC, -0.23)  # on the bare table, to the right of the dispenser
SEAT_Y = YF + STACK[1] / 2  # the stack's centre y, seated against the lips
SEAT_Z = FLOOR_TOP + STACK[2] / 2

# -- follower: plate + handle arm + tab, one rigid body on a slide (y) with a spring toward the front ----------------
FOL_SIZE = (0.130, T, 0.040)
FOL_Y0 = YF + T / 2  # plate centre at the start (its front face against the lips)
FOL_Z = FLOOR_TOP + 0.002 + FOL_SIZE[2] / 2
ARM = (0.016, 0.100, 0.006)  # horizontal bar behind the plate, flush with its top
TAB = (0.030, 0.016, 0.058)
TAB_DX = 0.025  # the arm and tab sit 4.5 cm to one side of the centre line, away from the hand placing the stack  # the upright handle tab at the arm's end
ARM_OFF = (TAB_DX, T / 2 + ARM[1] / 2, FOL_SIZE[2] / 2 - ARM[2] / 2)
TAB_Y_OFF = T / 2 + ARM[1] - TAB[1] / 2  # the tab's centre behind the plate's centre
TAB_OFF = (TAB_DX, TAB_Y_OFF, FOL_SIZE[2] / 2 - ARM[2] + TAB[2] / 2)
STROKE = 0.110  # the follower can be pulled back this far (cavity up to 11 cm for a 7.5 cm stack)
SPRING_N_PER_M = 90.0  # about 7.5 N when pressing the stack
FOL_KG = 0.15
POST_Y0 = FOL_Y0 + TAB_Y_OFF  # the tab's centre y at the start
POST_Z = TOP + 0.076  # grip height on the tab

OBJECTS = [
    Box("dispenser", size=(INNER_W + 2 * T, FLOOR_L, T), pos=(XC, FLOOR_Y, TOP + T / 2), color=CHROME),
    Box("napkins", size=STACK, pos=(*STACK_C0, TOP + STACK[2] / 2), color=PAPER, mass=STACK_KG),
    Box("follower", size=FOL_SIZE, pos=(XC, FOL_Y0, FOL_Z), color=STEEL, mass=FOL_KG),
]
_WZ = (T + WALL_H) / 2 - T / 2  # wall centre above the floor plate's centre
mech.compound("dispenser", [
    ((T, FLOOR_L, T + WALL_H), (dx * (INNER_W / 2 + T / 2), 0.0, _WZ), CHROME) for dx in (-1, 1)
] + [
    ((LIP_W, T, T + WALL_H), (dx * (INNER_W / 2 - LIP_W / 2), (YF - T / 2) - FLOOR_Y, _WZ), CHROME) for dx in (-1, 1)
] + [
    ((INNER_W + 2 * T, T, T + BAR_H), (0.0, (YF - T / 2) - FLOOR_Y, (BAR_H) / 2), CHROME),
])
mech.compound("follower", [(ARM, ARM_OFF, STEEL), (TAB, TAB_OFF, (0.15, 0.15, 0.17))])
mech.prismatic("follower", "Y", (XC, FOL_Y0, FOL_Z), lower=0.0, upper=STROKE, stiffness=SPRING_N_PER_M, damping=8.0,
               target=0.0)

# -- rules --------------------------------------------------------------------------------------------------------
STILL_S, STILL_M = 1.0, 0.003
LET_GO_M = 0.07  # the right TCP this far from the stack's centre
FOL_LET_GO_M = 0.06  # the left TCP this far from the follower's tab
X_TOL, X_TOL_LOOSE = 0.012, 0.015
YAW_FULL, YAW_LOOSE = math.radians(5.0), math.radians(10.0)
GAP_PRESSED = 0.005  # follower's face within 5 mm of the stack's back face, stack's face within 5 mm of the lips
Z_TOL = 0.008
_WINDOW = int(STILL_S * CONTROL_HZ)


def _rot(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def tilt_yaw(objects):
    """The stack's tilt from upright and its yaw about the vertical (rad)."""
    r = _rot(objects["napkins"].quat_xyzw)
    tilt = math.acos(max(-1.0, min(1.0, float(r[2, 2]))))
    yaw = math.atan2(float(r[1, 0]), float(r[0, 0]))
    yaw = (yaw + math.pi / 4) % (math.pi / 2) - math.pi / 4  # the stack is symmetric under a quarter turn: mod 90
    return tilt, yaw


def _fresh():
    return dict(f0=None, hist=[], n=0)


_state = _fresh()


@mech.on_tick
def _reset(sim):
    if sim.t <= 1.5 / CONTROL_HZ:
        _state.clear()
        _state.update(_fresh())


def follower_slide(objects):
    """How far the follower has been pulled back from its start (m)."""
    return float(objects["follower"].pos[1]) - _state["f0"]


def post_point(objects):
    return np.array([float(objects["follower"].pos[0]) + TAB_DX, POST_Y0 + follower_slide(objects), POST_Z])


def measures(objects):
    """(inside, seated_full, seated_loose): the stack's pose relative to the dispenser."""
    p = np.asarray(objects["napkins"].pos, float)
    tilt, yaw = tilt_yaw(objects)
    s = follower_slide(objects)
    dx, dz = abs(p[0] - XC), abs(p[2] - SEAT_Z)
    front_gap = (p[1] - STACK[1] / 2) - YF  # stack's front face behind the lips (>= 0), gap to them
    back_gap = (YF + s) - (p[1] + STACK[1] / 2)  # follower's face minus the stack's back face (<= 0: pressed)
    inside = dz <= Z_TOL and -0.002 <= front_gap <= STROKE - STACK[1] and dx <= X_TOL_LOOSE
    loose = inside and tilt <= YAW_LOOSE and abs(yaw) <= YAW_LOOSE
    full = (dx <= X_TOL and tilt <= YAW_FULL and abs(yaw) <= YAW_FULL and front_gap <= GAP_PRESSED
            and abs(back_gap) <= GAP_PRESSED and inside)
    return inside, full, loose


def still(st) -> bool:
    h = st["hist"]
    if len(h) < _WINDOW:
        return False
    a = np.asarray(h[-_WINDOW:])
    return bool(np.all(np.linalg.norm(a - a[-1], axis=1) < STILL_M))


def score(objects):
    """0.0 doing nothing; also the stack on top of or outside the dispenser, on its side, skewed more than 10
    degrees, not still for the last second, or still in the right gripper (TCP within 7 cm of its centre).
    1.0: stack seated in the dispenser (within 1.2 cm of the centre line, standing on the floor, within 5 degrees of
    square, its front face within 5 mm of the lips, the follower's face within 5 mm of its back face), still for 1 s
    and both grippers let go. 0.5: the same but looser (tilt or yaw 5-10 degrees, up to 1.5 cm off the centre line, or
    gaps over 5 mm), both grippers let go. 0.3: seated and still, the right gripper let go but the left still holds the
    follower tab (within 6 cm)."""
    st = _state
    if st["f0"] is None:
        st["f0"] = float(objects["follower"].pos[1])
    st["n"] += 1
    st["hist"] = (st["hist"] + [np.concatenate([np.asarray(objects["napkins"].pos, float),
                                                [float(objects["follower"].pos[1])]])])[-(_WINDOW + 5):]
    inside, full, loose = measures(objects)
    sp = np.asarray(objects["napkins"].pos, float)
    right_off = float(np.linalg.norm(np.asarray(objects["right_tcp"].pos, float) - sp)) >= LET_GO_M
    left_off = float(np.linalg.norm(np.asarray(objects["left_tcp"].pos, float) - post_point(objects))) >= FOL_LET_GO_M
    if DEBUG and st["n"] % 20 == 0:
        tilt, yaw = tilt_yaw(objects)
        print(f"DBG t={st['n'] / CONTROL_HZ:.0f} stack=({sp[0]:.3f},{sp[1]:.3f},{sp[2] - TOP:.3f}) "
              f"tilt={math.degrees(tilt):.1f} yaw={math.degrees(yaw):.1f} slide={follower_slide(objects):.3f} Ltcp=({objects['left_tcp'].pos[0]:.3f},{objects['left_tcp'].pos[1]:.3f},{objects['left_tcp'].pos[2]-TOP:.3f}) post_y={post_point(objects)[1]:.3f} "
              f"inside={inside} full={full} loose={loose} still={still(st)} right_off={right_off} left_off={left_off}",
              flush=True)
    if not (inside and still(st) and right_off):
        return 0.0
    if not left_off:
        return 0.3 if loose else 0.0
    if full:
        return 1.0
    return 0.5 if loose else 0.0


# -- scripted demo ----------------------------------------------------------------------------------------------
# Left arm: grips the follower's tab and pulls it back; right arm: carries the stack (gripped across its thickness
# by its upper part), sets it behind the front lips, lets go; then the left arm lets go and the follower closes.

G_STACK_Z = TOP + 0.07  # TCP on the stack standing on the table
G_STACK_SEATED_Z = FLOOR_TOP + 0.07
CARRY_Z = TOP + 0.135  # the stack base (TCP - 7 cm) clears the walls (TOP + 0.049) by 1.5 cm
HOVER_Z = TOP + 0.13
DROP_Y = YF + STACK[1] / 2 + 0.018  # the open cavity, nearer the lips
PULL_Y = POST_Y0 + 0.115  # commanded past the 0.11 stop


def demo():
    sx, sy = STACK_C0
    left = [
        Waypoint(2.0, (XC + TAB_DX, POST_Y0, HOVER_Z), FINGER_OPEN, "over the follower's tab"),
        Waypoint(1.2, (XC + TAB_DX, POST_Y0, POST_Z), FINGER_OPEN, "down around the tab"),
        Waypoint(0.6, (XC + TAB_DX, POST_Y0, POST_Z), FINGER_CLOSED, "grip the tab"),
        Waypoint(2.0, (XC + TAB_DX, PULL_Y, POST_Z), FINGER_CLOSED, "pull the follower back"),
        Waypoint(5.6, (XC + TAB_DX, PULL_Y, POST_Z), FINGER_CLOSED, "hold it back"),
        Waypoint(0.6, (XC + TAB_DX, PULL_Y, POST_Z), FINGER_OPEN, "let go: the follower closes on the stack"),
        Waypoint(1.5, (XC + TAB_DX, PULL_Y - 0.01, POST_Z + 0.06), FINGER_OPEN, "up, clear of the tab"),
        Waypoint(2.0, (0.18, 0.27, TOP + 0.20), FINGER_OPEN, "park"),
        Waypoint(2.0, (0.18, 0.27, TOP + 0.20), FINGER_OPEN, "still"),
    ]
    right = [
        Waypoint(2.0, (sx, sy, HOVER_Z), FINGER_OPEN, "over the new stack"),
        Waypoint(1.2, (sx, sy, G_STACK_Z), FINGER_OPEN, "down beside its upper part"),
        Waypoint(0.6, (sx, sy, G_STACK_Z), FINGER_CLOSED, "grip the stack"),
        Waypoint(1.5, (sx, sy, CARRY_Z), FINGER_CLOSED, "lift it"),
        Waypoint(2.5, (XC, DROP_Y, CARRY_Z), FINGER_CLOSED, "over the open dispenser"),
        Waypoint(1.0, (XC, DROP_Y, CARRY_Z), FINGER_CLOSED, "settle over the cavity"),
        Waypoint(1.5, (XC, DROP_Y, G_STACK_SEATED_Z), FINGER_CLOSED, "lower it behind the lips"),
        Waypoint(0.6, (XC, DROP_Y, G_STACK_SEATED_Z), FINGER_OPEN, "let go"),
        Waypoint(1.5, (XC, DROP_Y, CARRY_Z), FINGER_OPEN, "up, clear of the stack"),
        Waypoint(2.0, (0.18, -0.27, TOP + 0.20), FINGER_OPEN, "park"),
        Waypoint(3.1, (0.18, -0.27, TOP + 0.20), FINGER_OPEN, "still"),
    ]
    return {"left": left, "right": right}
