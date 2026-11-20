"""Scorer tests for cases.py, run without the simulator:  python test_cases.py"""

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
from yam_arena.task_file import ObjectState  # noqa: E402

import cases as K  # noqa: E402
import mech  # noqa: E402

AWAY_L, AWAY_R = (0.18, 0.27, K.TOP + 0.20), (0.18, -0.27, K.TOP + 0.20)
Q0 = (0.0, 0.0, 0.0, 1.0)


def _o(pos, q=Q0):
    p = np.asarray(pos, float)
    return ObjectState(p, np.asarray(q, float), np.zeros(3), np.zeros(3), p - 0.01, p + 0.01)


def frame(meter, lids=(0.0, 0.0, 0.0), left=AWAY_L, right=AWAY_R, meter_q=Q0, stored_shift=(0.0, 0.0)):
    f = {"meter": _o(meter, meter_q), "left_tcp": _o(left), "right_tcp": _o(right)}
    for i, s in enumerate(lids):
        f[f"lid{i}"] = _o((K.CX + s, K.CASE_Y[i], K.LID_Z))
    for n, i in K.STORED.items():
        f[n] = _o((K.CX + stored_shift[0], K.CASE_Y[i] + stored_shift[1], K.ZF + 0.019))
    return f


def run(frames, repeat=1):
    K._state.clear()
    K._state.update(K._fresh())
    s = 0.0
    for f in frames:
        for _ in range(repeat):
            s = K.score(f)
    return s


IN_EMPTY = (K.CX, K.CASE_Y[K.EMPTY], K.ZF + 0.019)
BENCH = (*K.M_START, K.TOP + 0.019)
OCC = K.CASE_Y[next(iter(K.STORED.values()))]


def test_nothing():
    assert run([frame(BENCH)], 60) == 0.0


def test_full():
    assert run([frame(IN_EMPTY)], 60) == 1.0


def test_one_lid_open():
    assert run([frame(IN_EMPTY, lids=(0.0, 0.0, 0.0)[:0] or tuple(K.SLIDE if i == K.EMPTY else 0.0 for i in range(3)))], 60) == 0.5


def test_two_or_three_open():
    assert run([frame(IN_EMPTY, lids=(K.SLIDE, K.SLIDE, 0.0))], 60) == 0.25
    assert run([frame(IN_EMPTY, lids=(K.SLIDE,) * 3)], 60) == 0.25


def test_lid_ajar_not_closed():
    assert run([frame(IN_EMPTY, lids=(0.006, 0.0, 0.0))], 60) == 0.5
    assert run([frame(IN_EMPTY, lids=(0.004, 0.0, 0.0))], 60) == 1.0


def test_still_and_let_go():
    held = frame(IN_EMPTY, left=(IN_EMPTY[0], IN_EMPTY[1], IN_EMPTY[2] + 0.02))
    assert run([held], 60) == 0.0
    moving = [frame((K.CX + 0.001 * k, K.CASE_Y[K.EMPTY], K.ZF + 0.019)) for k in range(25)]
    assert run(moving) == 0.0
    assert run([frame(IN_EMPTY)], 5) == 0.0  # not yet still for 1 s


def test_occupied_violation_latches():
    inside = frame((K.CX, OCC, K.ZF + 0.019), lids=(K.SLIDE,) * 3)
    atop = frame((K.CX, OCC, K.ZF + 0.054), lids=(K.SLIDE,) * 3)
    for bad in (inside, atop):
        assert run([bad], 12) == 0.0
        assert run([bad] * 12 + [frame(IN_EMPTY)] * 60) == 0.0  # latched, can't be undone
    assert run([atop] * 5 + [frame(IN_EMPTY)] * 60) == 1.0  # a pass over it shorter than 0.5 s is no violation


def test_on_closed_lid_is_not_in_case():
    on_lid = frame((K.CX, OCC, K.LID_Z + 0.022))
    assert run([on_lid], 60) == 0.0
    assert run([on_lid] * 30 + [frame(IN_EMPTY)] * 60) == 1.0


def test_stored_disturbed_caps():
    assert run([frame(IN_EMPTY, stored_shift=(0.0, 0.0))], 3) == 0.0
    K._state.clear(); K._state.update(K._fresh())
    for _ in range(2):
        K.score(frame(IN_EMPTY))
    s = 0.0
    for _ in range(60):
        s = K.score(frame(IN_EMPTY, stored_shift=(0.0, 0.03)))
    assert s == 0.25


def test_tilted_or_wrong_height():
    q = mech.matrix_to_quat_xyzw(np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0.0]]))  # on its side
    assert run([frame(IN_EMPTY, meter_q=q)], 60) == 0.0
    assert run([frame((K.CX, K.CASE_Y[K.EMPTY], K.TOP + 0.09))], 60) == 0.0  # on the lid / held up
    assert run([frame((K.CX + 0.1, K.CASE_Y[K.EMPTY], K.ZF + 0.019))], 60) == 0.0  # outside the recess


def test_variants_distinct():
    import importlib
    for v in (0, 1, 2):
        import os
        os.environ["CASES_VARIANT"] = str(v)
        src = Path(K.__file__).read_text()
        ns = {"__file__": K.__file__, "__name__": f"cases_v{v}"}
        exec(compile(src, K.__file__, "exec"), ns)
        assert ns["EMPTY"] == (1, 2, 0)[v] and sorted(ns["STORED"].values() ) == [i for i in range(3) if i != ns["EMPTY"]]
    os.environ.pop("CASES_VARIANT", None)


def test_demo_reach():
    w = mech.check_plan(K.demo())
    for arm, (mm, deg) in w.items():
        assert mm < 10 and deg < 5, (arm, mm, deg)


if __name__ == "__main__":
    n = 0
    for k, v in list(globals().items()):
        if k.startswith("test_"):
            v()
            n += 1
    print(f"{n} tests passed")
