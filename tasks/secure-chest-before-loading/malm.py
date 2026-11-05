"""IKEA MALM-style dresser: lock it to the wall before loading a drawer.

On the table:
- a white 3-drawer dresser (about 1:6 scale of a MALM 3-drawer chest: 14 cm wide, 9 cm deep, 13.5 cm tall)
  standing against a grey wall, drawers facing the robot, each with a dark handle
- the anti-tip lock: a black bolt with a red knob lying on the wall top, right behind the dresser's top; sliding
  it 3 cm toward the robot puts its end into the black catch on the dresser's top, which holds the dresser
  to the wall
- a box of books (brown, 0.4 kg) on the table at the left

The dresser is light and the books are heavy: with the top drawer pulled out and the books in it, an unlocked
dresser tips forward, as real chests of drawers do when they aren't secured to the wall.
"""

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
import mech  # noqa: E402

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint, pick_place_waypoints  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# Dresser carcass: front face at x=F, centred on y=0
F, D, W = 0.19, 0.09, 0.14
CX, BACK = F + D / 2, F + D
TOP_Z = TOP + 0.135  # top surface of the dresser
SLOT_H = 0.04
DRAWER_Z = [TOP + 0.01 + SLOT_H * k for k in range(3)]  # slot floors: bottom, middle, top
OPEN_MAX = 0.07
OPEN_M, CLOSED_M = 0.015, 0.008  # pulled out more than OPEN_M = open; less than CLOSED_M = shut
TIP_DEG = 10.0
# Anti-tip lock: catch on the dresser's top, bolt on the wall top behind it
LY = 0.055
CATCH_X = BACK - 0.025
BOLT_L = 0.04
BOLT_Z = TOP_Z + 0.007
BOLT_X0 = BACK + BOLT_L / 2  # retracted: the bolt's front end is 1 cm behind the catch
SLIDE = 0.03
ENGAGED_M = 0.02
KNOB = (BOLT_X0 + BOLT_L / 2 + 0.012, LY, BOLT_Z)  # knob on the bolt's rear end
# Books
BOOKS = (0.04, 0.05, 0.025)
BOOKS_XY = (0.13, 0.17)

WHITE, DARK, GREY, BLACK, BROWN = (0.93, 0.93, 0.90), (0.2, 0.2, 0.2), (0.6, 0.6, 0.62), (0.08, 0.08, 0.08), (0.55, 0.35, 0.2)

OBJECTS = [
    # carcass: the base panel carries the joints; the other panels are welded to it
    Box("carcass_base", size=(D, W, 0.01), pos=(CX, 0.0, TOP + 0.005), color=WHITE, mass=0.05),
    Box("side_l", size=(D, 0.006, 0.12), pos=(CX, W / 2 - 0.003, TOP + 0.07), color=WHITE, mass=0.015),
    Box("side_r", size=(D, 0.006, 0.12), pos=(CX, -W / 2 + 0.003, TOP + 0.07), color=WHITE, mass=0.015),
    Box("top_panel", size=(D, W, 0.005), pos=(CX, 0.0, TOP_Z - 0.0025), color=WHITE, mass=0.02),
    Box("back_panel", size=(0.005, W - 0.012, 0.12), pos=(BACK - 0.0025, 0.0, TOP + 0.07), color=WHITE, mass=0.012),
    # catch: two cheeks and a cap forming a tunnel for the bolt
    Box("catch_l", size=(0.03, 0.006, 0.014), pos=(CATCH_X, LY + 0.011, TOP_Z + 0.007), color=BLACK, mass=0.003),
    Box("catch_r", size=(0.03, 0.006, 0.014), pos=(CATCH_X, LY - 0.011, TOP_Z + 0.007), color=BLACK, mass=0.003),
    Box("catch_cap", size=(0.03, 0.028, 0.004), pos=(CATCH_X, LY, TOP_Z + 0.016), color=BLACK, mass=0.003),
    # wall (as tall as the dresser), the bolt lying on its top
    Box("wall", size=(0.012, 0.36, TOP_Z - TOP), pos=(BACK + 0.011, 0.0, (TOP + TOP_Z) / 2), color=GREY),
    Box("bolt", size=(BOLT_L, 0.01, 0.01), pos=(BOLT_X0, LY, BOLT_Z), color=BLACK, mass=0.02),
    Box("bolt_knob", size=(0.016, 0.026, 0.016), pos=KNOB, color=(0.85, 0.15, 0.1), mass=0.004),
    Box("books", size=BOOKS, pos=(*BOOKS_XY, TOP + BOOKS[2] / 2 + 0.001), color=BROWN, mass=0.4),
]
for k, zb in enumerate(DRAWER_Z):
    OBJECTS += [
        Box(f"drawer{k}", size=(0.008, 0.124, 0.036), pos=(F + 0.004, 0.0, zb + 0.020), color=WHITE, mass=0.01),
        Box(f"drawer{k}_bottom", size=(0.07, 0.12, 0.004), pos=(F + 0.043, 0.0, zb + 0.004), color=WHITE, mass=0.006),
        Box(f"drawer{k}_back", size=(0.005, 0.12, 0.025), pos=(F + 0.0805, 0.0, zb + 0.0185), color=WHITE, mass=0.003),
        Box(f"drawer{k}_handle", size=(0.02, 0.03, 0.01), pos=(F - 0.01, 0.0, zb + 0.020), color=DARK, mass=0.002),
    ]

for part in ("side_l", "side_r", "top_panel", "back_panel"):
    mech.weld(part, parent="carcass_base")
for part in ("catch_l", "catch_r", "catch_cap"):
    mech.weld(part, parent="top_panel")
for k in range(3):
    mech.prismatic(f"drawer{k}", "X", (F + 0.004, 0.0, DRAWER_Z[k] + 0.02), lower=-OPEN_MAX, upper=0.0,
                   damping=0.5, parent="carcass_base")
    for part in ("bottom", "back", "handle"):
        mech.weld(f"drawer{k}_{part}", parent=f"drawer{k}")
mech.prismatic("bolt", "X", (BOLT_X0, LY, BOLT_Z), lower=-SLIDE - 0.005, upper=0.0, damping=2.0)
mech.weld("bolt_knob", parent="bolt")


# -- scoring ----------------------------------------------------------------------------------


def tilt_deg(q) -> float:
    """Angle between a body's up axis and the world's up axis, from its quaternion (x, y, z, w)."""
    x, y, z, w = q
    return math.degrees(math.acos(max(-1.0, min(1.0, 1 - 2 * (x * x + y * y)))))


def opened(objects, k) -> float:
    """How far drawer k is pulled out of the carcass (m)."""
    return float((objects["carcass_base"].pos[0] - CX) - (objects[f"drawer{k}"].pos[0] - (F + 0.004)))


def engaged(objects) -> bool:
    return objects["bolt"].pos[0] < BOLT_X0 - ENGAGED_M


def books_in_top_drawer(objects) -> bool:
    b, d = objects["books"], objects["drawer2"]
    inner = d.pos[0] + 0.004  # inner face of the drawer front
    floor = DRAWER_Z[2] + 0.006 + (objects["carcass_base"].pos[2] - (TOP + 0.005))
    return inner < b.pos[0] < inner + 0.072 and abs(b.pos[1]) < 0.06 and b.low[2] > floor - 0.004 and b.resting()


def _fresh():
    return dict(violated=False, tipped=False)


_state = _fresh()
CONTROL_HZ = 20.0


@mech.on_tick
def _new_episode(sim):
    """The simulator's episode clock marks a new episode (object positions can't: a policy may put things back)."""
    if sim.t <= 1.5 / CONTROL_HZ:
        _state.update(_fresh())


def score(objects):
    st = _state
    if not engaged(objects) and any(opened(objects, k) > OPEN_M for k in range(3)):
        st["violated"] = True
    if tilt_deg(objects["carcass_base"].quat_xyzw) > TIP_DEG:
        st["tipped"] = True
    if st["violated"] or st["tipped"]:
        return 0.0
    if books_in_top_drawer(objects):
        return 1.0 if opened(objects, 2) < CLOSED_M and engaged(objects) else 0.5
    return 0.25 if engaged(objects) else 0.0


# -- scripted demo ----------------------------------------------------------------------------


def demo():
    travel = TOP + 0.16
    open_by = 0.06
    handle0 = (F - 0.01, 0.0, DRAWER_Z[2] + 0.020)
    handle1 = (handle0[0] - open_by, 0.0, handle0[2])
    shut = (handle0[0] + 0.006, 0.0, handle0[2])  # a little past closed: the drawer seats against its stop
    books_place = (F + 0.008 - open_by + 0.004 + BOOKS[0] / 2, 0.02, DRAWER_Z[2] + 0.006 + BOOKS[2] / 2 + 0.003)
    slid = (KNOB[0] - SLIDE, KNOB[1], KNOB[2])
    left = [
        # lock the dresser to the wall: grip the bolt's knob, slide it toward the robot into the catch, let go
        Waypoint(2.0, (KNOB[0], KNOB[1], KNOB[2] + 0.03), FINGER_OPEN, "over the bolt knob"),
        Waypoint(1.2, KNOB, FINGER_OPEN, "down to the knob"),
        Waypoint(0.8, KNOB, FINGER_CLOSED, "grip"),
        Waypoint(2.0, slid, FINGER_CLOSED, "slide the bolt into the catch"),
        Waypoint(0.6, slid, FINGER_OPEN, "let go"),
        Waypoint(1.0, (slid[0], slid[1], slid[2] + 0.03), FINGER_OPEN, "up"),
        Waypoint(5.0, (0.14, 0.22, travel), FINGER_OPEN, "wait for the drawer"),
        *pick_place_waypoints((*BOOKS_XY, TOP + BOOKS[2] / 2 + 0.001), books_place, travel_z=travel),
        Waypoint(1.5, (0.14, 0.22, travel), FINGER_OPEN, "clear"),
    ]
    right = [
        Waypoint(8.5, (0.14, -0.18, travel), FINGER_OPEN, "wait for the lock"),
        Waypoint(1.5, (handle0[0], handle0[1], handle0[2] + 0.04), FINGER_OPEN, "over the top handle"),
        Waypoint(1.2, handle0, FINGER_OPEN, "down"),
        Waypoint(0.8, handle0, FINGER_CLOSED, "grip"),
        Waypoint(2.0, handle1, FINGER_CLOSED, "pull the top drawer out"),
        Waypoint(0.6, handle1, FINGER_OPEN, "let go"),
        Waypoint(1.0, (0.12, -0.24, travel), FINGER_OPEN, "clear for the books"),
        Waypoint(9.5, (0.12, -0.24, travel), FINGER_OPEN, "wait for the books"),
        Waypoint(1.5, (handle1[0], handle1[1], handle1[2] + 0.04), FINGER_OPEN, "back over the handle"),
        Waypoint(1.2, handle1, FINGER_OPEN, "down"),
        Waypoint(0.8, handle1, FINGER_CLOSED, "grip"),
        Waypoint(2.0, shut, FINGER_CLOSED, "push the drawer shut"),
        Waypoint(0.6, shut, FINGER_OPEN, "let go"),
        Waypoint(1.5, (0.12, -0.24, travel), FINGER_OPEN, "clear"),
    ]
    return {"left": left, "right": right}
