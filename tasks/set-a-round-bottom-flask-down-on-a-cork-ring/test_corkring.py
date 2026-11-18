"""Scorer tests for corkring.py, run without the simulator:  python test_corkring.py"""

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
from yam_arena.task_file import ObjectState  # noqa: E402

import corkring as C  # noqa: E402
import mech  # noqa: E402

AWAY_L, AWAY_R = (0.18, 0.27, C.TOP + 0.20), (0.18, -0.27, C.TOP + 0.20)


def _obj(pos, q=(0.0, 0.0, 0.0, 1.0), half=0.01):
    pos = np.asarray(pos, float)
    return ObjectState(pos, np.asarray(q, float), np.zeros(3), np.zeros(3), pos - half, pos + half)


def _qx(deg):
    t = math.radians(deg) / 2
    return (math.sin(t), 0.0, 0.0, math.cos(t))


def frame(c, tilt=0.0, right=AWAY_R, left=AWAY_L):
    c = np.asarray(c, float)
    return {"flask": _obj(c, _qx(tilt), 0.05), "flask_mark": _obj(c, _qx(tilt)),
            "left_tcp": _obj(left), "right_tcp": _obj(right)}


def run(frames):
    C._state.clear()
    C._state.update(C._fresh())
    s = 0.0
    for f in frames:
        s = C.score(f)
    return s


MX, MY = C.MANTLE_C
RX, RY = C.RING_C
START = frame((MX, MY, C.TOP + C.FLASK_ZC))
CARRY = frame((MX, MY + 0.05, C.TOP + 0.20))
IN_RING = frame((RX, RY, C.TOP + C.RING_ZC))
REST = 25


def lower(x, y, z_end, tilt=0.0):
    """A gentle descent from carrying height to z_end."""
    zs = np.arange(C.TOP + 0.20, z_end, -0.01)
    return [frame((x, y, z), tilt) for z in zs]


RAMP = lower(RX, RY, C.TOP + C.RING_ZC)
BENCH_RAMP = lower(0.35, -0.02, C.TOP + C.R + 0.001)


def test_doing_nothing_scores_zero():
    assert run([START] * 40) == 0.0


def test_seated_in_the_ring_scores_one():
    assert C.in_ring(IN_RING)
    assert run([START] * 3 + [CARRY] * 5 + RAMP + [IN_RING] * REST) == 1.0


def test_seated_off_centre_within_tolerance_scores_one():
    assert run([START] * 3 + [CARRY] * 5 + RAMP + [frame((RX + 0.01, RY - 0.008, C.TOP + C.RING_ZC))] * REST) == 1.0


def test_off_centre_beyond_tolerance_scores_zero():
    assert run([START] * 3 + [CARRY] * 5 + RAMP + [frame((RX + 0.03, RY, C.TOP + C.RING_ZC))] * REST) == 0.0


def test_tilted_neck_scores_zero():
    assert run([START] * 3 + [CARRY] * 5 + RAMP + [frame((RX, RY, C.TOP + C.RING_ZC), tilt=30)] * REST) == 0.0


def test_back_in_the_mantle_scores_zero():
    assert run([START] * 3 + [CARRY] * 5 + [START] * REST) == 0.0


def test_on_the_bare_bench_scores_zero():
    b = frame((0.35, -0.02, C.TOP + C.R + 0.001), tilt=90)
    assert C.on_bench(b)
    assert run([START] * 3 + [CARRY] * 5 + BENCH_RAMP + [b] * REST) == 0.0


def test_bench_then_ring_is_still_zero():
    b = frame((0.35, -0.02, C.TOP + C.R + 0.001), tilt=90)
    assert run([START] * 3 + [CARRY] * 5 + BENCH_RAMP + [b] * 15 + [CARRY] * 5 + RAMP + [IN_RING] * REST) == 0.0


def test_touching_the_bench_briefly_is_not_setting_it_down():
    b = frame((0.35, -0.02, C.TOP + C.R + 0.001))
    assert run([START] * 3 + [CARRY] * 5 + BENCH_RAMP + [b] * 5 + [CARRY] * 5 + RAMP + [IN_RING] * REST) == 1.0


def test_seated_in_the_ring_is_not_the_bench():
    assert not C.on_bench(IN_RING)


def test_dropped_into_the_ring_scores_zero():
    high = frame((RX, RY, C.TOP + C.RING_ZC + 0.08))
    assert run([START] * 3 + [CARRY] * 3 + [high] * 3 + [IN_RING] * REST) == 0.0


def test_gentle_lowering_is_not_a_drop():
    f = [frame((RX, RY, C.TOP + 0.20 - 0.01 * i)) for i in range(15)]
    assert run([START] * 3 + [CARRY] * 3 + f + [IN_RING] * REST) == 1.0


def test_still_held_scores_zero():
    held = frame((RX, RY, C.TOP + C.RING_ZC), right=(RX, RY, C.TOP + C.RING_ZC + C.GRASP_UP))
    assert run([START] * 3 + [CARRY] * 5 + RAMP + [held] * REST) == 0.0


def test_needs_a_second_at_rest():
    assert run([START] * 3 + [CARRY] * 5 + RAMP + [IN_RING] * 10) == 0.0


def test_off_the_table_scores_zero():
    assert run([START] * 3 + [CARRY] * 5 + [frame((0.45, 0.8, C.TOP - 0.4))] * REST) == 0.0


def test_demo_plan_reaches():
    worst = mech.check_plan(C.demo())
    assert worst["right"][0] < 10.0 and worst["right"][1] < 3.0, worst


def test_geometry():
    assert abs(C.FLASK_ZC - 0.073) < 0.003
    assert C.ZG - C.TOP < C.TOP + 1  # grip within the neck
    assert C.FLASK_ZC + C.R < C.M_TOP + 0.05
    assert 0.040 < C.RING_ZC < 0.060


if __name__ == "__main__":
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for t in tests:
        t()
        print("ok", t.__name__)
    print(f"{len(tests)} passed")
