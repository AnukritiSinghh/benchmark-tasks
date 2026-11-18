"""Scorer tests for gelbox.py, run without the simulator:  python test_gelbox.py"""

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
from yam_arena.task_file import ObjectState  # noqa: E402

import gelbox as G  # noqa: E402
import mech  # noqa: E402

AWAY_L, AWAY_R = (0.18, 0.27, G.TOP + 0.20), (0.18, -0.27, G.TOP + 0.20)


def _obj(pos, q=(0.0, 0.0, 0.0, 1.0)):
    pos = np.asarray(pos, float)
    return ObjectState(pos, np.asarray(q, float), np.zeros(3), np.zeros(3), pos - 0.01, pos + 0.01)


def yaw_q(deg):
    a = math.radians(deg) / 2
    return (0.0, 0.0, math.sin(a), math.cos(a))


def tilt_q(deg):  # about world y
    a = math.radians(deg) / 2
    return (0.0, math.sin(a), 0.0, math.cos(a))


def frame(tray, yaw_deg=-90.0, gel_off=(0.0, 0.0, 0.0), left=AWAY_L, right=AWAY_R, q=None):
    q = yaw_q(yaw_deg) if q is None else q
    tray = np.asarray(tray, float)
    r = G._rot(q)
    return {"tray": _obj(tray, q), "gel": _obj(tray + r @ np.asarray(gel_off, float), q),
            "left_tcp": _obj(left), "right_tcp": _obj(right)}


def run(frames, n=None):
    G._state.clear()
    G._state.update(G._fresh())
    s = 0.0
    for f in frames:
        s = G.score(f)
    return s


SEATED = (G.TANK_C[0], G.TANK_C[1], G.SEAT_Z)
START = (G.TRAY_START[0], G.TRAY_START[1], G.TOP + G.TRAY_H / 2)
N = 30


def hold(f, n=N):
    return [f] * n


def test_full():
    assert run(hold(frame(SEATED))) == 1.0
    assert run(hold(frame((SEATED[0] + 0.003, SEATED[1] - 0.003, SEATED[2] + 0.001), -90 + 4))) == 1.0


def test_nothing():
    assert run(hold(frame(START, 0.0))) == 0.0


def test_wells_at_red():
    assert run(hold(frame(SEATED, 90.0))) == 0.0
    assert run(hold(frame(SEATED, 120.0))) == 0.0
    assert run(hold(frame(SEATED, 0.0))) == 0.0  # sideways


def test_partial():
    assert run(hold(frame((SEATED[0] + 0.012, SEATED[1], SEATED[2])))) == 0.5  # off centre
    assert run(hold(frame(SEATED, -90 + 10))) == 0.5  # turned 10 degrees
    assert run(hold(frame(SEATED, -90 + 20))) == 0.0
    assert run(hold(frame((SEATED[0] + 0.03, SEATED[1], SEATED[2])))) == 0.0
    assert run(hold(frame((SEATED[0], SEATED[1], SEATED[2] + 0.004)))) == 0.5  # 4 mm proud
    assert run(hold(frame((SEATED[0], SEATED[1], SEATED[2] + 0.02)))) == 0.0  # not on the deck


def test_gel_out_latched():
    ok, bad = frame(SEATED), frame(SEATED, gel_off=(0.03, 0.0, 0.0))
    assert run(hold(bad)) == 0.0
    assert run(hold(frame(START, 0.0), 5) + hold(bad, 5) + hold(frame(SEATED), 40)) == 0.0  # slid out and put back
    assert run(hold(ok, 3) + hold(frame(SEATED, gel_off=(0.0, 0.0, 0.03)), 2) + hold(ok, 40)) == 1.0  # 2-step glitch


def test_must_be_still_and_let_go():
    f = frame(SEATED)
    assert run(hold(f, 10)) == 0.0  # not 1 s yet
    assert run(hold(frame(SEATED, right=(SEATED[0], SEATED[1], SEATED[2] + 0.03)))) == 0.0  # still holding
    moving = [frame((SEATED[0], SEATED[1] + 0.001 * i, SEATED[2])) for i in range(30)]
    assert run(moving) == 0.0
    assert run([frame(START, 0.0)] * 10 + [frame(SEATED)] * 10) == 0.0


def test_tilted_or_flipped():
    assert run(hold(frame(SEATED, q=tilt_q(15)))) == 0.0
    assert run(hold(frame(SEATED, q=tilt_q(180)))) == 0.0


def test_off_table():
    assert run(hold(frame((0.3, 0.5, G.TOP - 0.7)))) == 0.0


def test_demo_reach():
    r = mech.check_plan(G.demo())
    assert all(mm < 10 and deg < 5 for mm, deg in r.values()), r


def test_geometry():
    assert G.GEL_W < G.TRAY_W - 2 * G.TRAY_WALL and G.GEL_L <= G.TRAY_L
    assert G.GEL_T < G.TRAY_H - G.TRAY_FLOOR  # gel below the wall tops
    assert G.DECK_SIZE[0] < G.TRAY_W < 2 * 0.072  # deck narrower than the tray, tray fits between the tank's walls
    assert G.TRAY_L < G.DECK_SIZE[1] - 0.006  # tray fits between the marks


if __name__ == "__main__":
    n = 0
    for k, v in list(globals().items()):
        if k.startswith("test_"):
            v()
            n += 1
    print(f"{n} tests passed")
