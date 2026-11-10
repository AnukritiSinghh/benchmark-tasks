"""Scorer tests for aed.py, run without the simulator:  python test_aed.py"""

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
from yam_arena.task_file import ObjectState  # noqa: E402

import aed as a  # noqa: E402

PARK_L, PARK_R = a.LEFT_PARK, a.RIGHT_PARK
ON_PAD = (*a.UPPER_XY, a.CHEST_TOP + a.PAD[2] + 0.01)
AT_BTN = (*a.BTN_XY, a.BTN_Z + 0.02)


def _obj(pos, size=(0.02, 0.02, 0.02)):
    pos, half = np.asarray(pos, float), np.asarray(size, float) / 2
    return ObjectState(pos, np.array([0.0, 0.0, 0.0, 1.0]), np.zeros(3), np.zeros(3), pos - half, pos + half)


def frame(pad_down=0.0, btn_down=0.0, left=PARK_L, right=PARK_R):
    """pad_down: how far the loose pad has been pressed (0 .. LIFT_M); btn_down: how far SHOCK is pressed."""
    return {"pad_upper": _obj((*a.UPPER_XY, a.UPPER_Z0 - pad_down), a.PAD),
            "shock_button": _obj((*a.BTN_XY, a.BTN_Z - btn_down), a.BTN_SIZE),
            "left_tcp": _obj(left), "right_tcp": _obj(right)}


class FakeSim:
    def __init__(self, t, states):
        self.t, self.states = t, states

    def set_color(self, name, rgb):
        pass


def run(frames):
    out = None
    for i, f in enumerate(frames):
        a._tick(FakeSim((i + 1) / a.CONTROL_HZ, f))
        out = a.score(f)
    return out


FLAT = a.LIFT_M
SEC = int(a.CONTROL_HZ)


def test_parks_are_clear():
    assert a.manikin_distance(PARK_L) >= a.CLEAR_M and a.manikin_distance(PARK_R) >= a.CLEAR_M
    assert a.manikin_distance(AT_BTN) >= a.CLEAR_M


def test_nothing_done_scores_zero():
    assert run([frame()] * 40) == 0.0


def test_pad_pressed_flat_scores_half():
    assert run([frame()] * 5 + [frame(pad_down=FLAT, left=ON_PAD)] * 10 + [frame(pad_down=FLAT)] * 10) == 0.5


def test_pad_then_clear_shock_scores_one():
    f = [frame(pad_down=FLAT, left=ON_PAD)] * 10 + [frame(pad_down=FLAT)] * 10
    f += [frame(pad_down=FLAT, btn_down=0.004, right=AT_BTN)] * 5 + [frame(pad_down=FLAT, right=AT_BTN)] * (SEC + 5)
    assert run(f) == 1.0


def test_shock_while_still_on_the_pad_scores_zero():
    f = [frame(pad_down=FLAT, left=ON_PAD)] * 10
    f += [frame(pad_down=FLAT, btn_down=0.004, left=ON_PAD, right=AT_BTN)] * 5 + [frame(pad_down=FLAT)] * (SEC + 5)
    assert run(f) == 0.0


def test_touching_within_the_second_after_the_press_scores_zero():
    f = [frame(pad_down=FLAT, left=ON_PAD)] * 10 + [frame(pad_down=FLAT)] * 10
    f += [frame(pad_down=FLAT, btn_down=0.004, right=AT_BTN)] * 3
    f += [frame(pad_down=FLAT, left=ON_PAD, right=AT_BTN)] * 5 + [frame(pad_down=FLAT)] * SEC
    assert run(f) == 0.0


def test_touching_after_the_shock_is_delivered_is_fine():
    f = [frame(pad_down=FLAT, left=ON_PAD)] * 10 + [frame(pad_down=FLAT)] * 10
    f += [frame(pad_down=FLAT, btn_down=0.004, right=AT_BTN)] * 3 + [frame(pad_down=FLAT)] * (SEC + 5)
    f += [frame(pad_down=FLAT, left=ON_PAD)] * 10  # checking the patient afterwards
    assert run(f) == 1.0


def test_shock_with_the_pad_loose_scores_zero():
    f = [frame(btn_down=0.004, right=AT_BTN)] * 5 + [frame()] * SEC
    f += [frame(pad_down=FLAT, left=ON_PAD)] * 10 + [frame(pad_down=FLAT)] * 10
    assert run(f) == 0.0


def test_pad_half_pressed_is_not_flat():
    assert run([frame(pad_down=a.LIFT_M / 2)] * 20) == 0.0


if __name__ == "__main__":
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for t in tests:
        t()
        print("ok", t.__name__)
    print(f"{len(tests)} passed")
