"""Scorer tests for napkins.py, run without the simulator:  python test_napkins.py"""

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
from yam_arena.task_file import ObjectState  # noqa: E402

import mech  # noqa: E402
import napkins as N  # noqa: E402

AWAY_L, AWAY_R = (0.18, 0.27, N.TOP + 0.20), (0.18, -0.27, N.TOP + 0.20)
FOL_AABB_DY = 0.023  # the follower's reported position (its bounding box's centre) is this far behind the plate's centre


def _q(yaw=0.0, roll=0.0):
    r = np.array([[math.cos(yaw), -math.sin(yaw), 0.0], [math.sin(yaw), math.cos(yaw), 0.0], [0.0, 0.0, 1.0]])
    c, s = math.cos(roll), math.sin(roll)
    r = r @ np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])
    return mech.matrix_to_quat_xyzw(r)


def _obj(pos, q=(0.0, 0.0, 0.0, 1.0)):
    pos = np.asarray(pos, float)
    return ObjectState(pos, np.asarray(q, float), np.zeros(3), np.zeros(3), pos - 0.01, pos + 0.01)


def frame(stack_pos, slide=0.0, yaw=0.0, roll=0.0, left=AWAY_L, right=AWAY_R):
    fy = N.FOL_Y0 + FOL_AABB_DY + slide
    return {"napkins": _obj(stack_pos, _q(yaw, roll)), "follower": _obj((N.XC, fy, N.FOL_Z)),
            "left_tcp": _obj(left), "right_tcp": _obj(right)}


def seated(dx=0.0, dy=0.0, yaw=0.0, roll=0.0, back_gap=0.0, **kw):
    """The stack seated against the lips with the follower pressing it (or back_gap mm off its back face)."""
    pos = (N.XC + dx, N.SEAT_Y + dy, N.SEAT_Z)
    slide = (pos[1] + N.STACK[1] / 2 + back_gap) - N.YF
    return frame(pos, slide, yaw, roll, **kw)


START = frame((*N.STACK_C0, N.TOP + N.STACK[2] / 2))
REST = 25


def run(frames):
    N._state.clear()
    N._state.update(N._fresh())
    s = 0.0
    for f in frames:
        s = N.score(f)
    return s


def test_doing_nothing_scores_zero():
    assert run([START] * 60) == 0.0


def test_seated_and_pressed_scores_one():
    assert run([START] * 3 + [seated()] * REST) == 1.0


def test_demo_end_state_scores_one_with_small_errors():
    assert run([START] * 3 + [seated(dx=0.008, yaw=math.radians(3), back_gap=0.003)] * REST) == 1.0


def test_skew_between_5_and_10_degrees_scores_half():
    assert run([START] * 3 + [seated(yaw=math.radians(8))] * REST) == 0.5


def test_skew_over_10_degrees_scores_zero():
    assert run([START] * 3 + [seated(yaw=math.radians(14))] * REST) == 0.0


def test_follower_not_closed_on_the_stack_scores_half():
    assert run([START] * 3 + [seated(back_gap=0.02)] * REST) == 0.5


def test_off_the_centre_line_scores_half_then_zero():
    assert run([START] * 3 + [seated(dx=0.014)] * REST) == 0.5
    assert run([START] * 3 + [seated(dx=0.03)] * REST) == 0.0


def test_stack_still_held_back_by_the_left_gripper_scores_point_three():
    f = seated(back_gap=0.02)
    tab = N.post_point(f)
    f = seated(back_gap=0.02, left=tuple(tab))
    assert run([START] * 3 + [f] * REST) == 0.3


def test_stack_still_in_the_right_gripper_scores_zero():
    f = seated(right=(N.XC, N.SEAT_Y, N.SEAT_Z + 0.03))
    assert run([START] * 3 + [f] * REST) == 0.0


def test_stack_on_top_of_the_dispenser_scores_zero():
    f = frame((N.XC, N.SEAT_Y, N.SEAT_Z + 0.05), 0.0)
    assert run([START] * 3 + [f] * REST) == 0.0


def test_stack_in_front_of_the_lips_scores_zero():
    f = frame((N.XC, N.YF - 0.05, N.TOP + N.STACK[2] / 2), 0.0)
    assert run([START] * 3 + [f] * REST) == 0.0


def test_stack_on_its_side_scores_zero():
    f = seated(roll=math.radians(90))
    assert run([START] * 3 + [f] * REST) == 0.0


def test_stack_resting_on_the_follower_scores_zero():
    f = frame((N.XC, N.YF + 0.09, N.SEAT_Z + 0.02), 0.0)
    assert run([START] * 3 + [f] * REST) == 0.0


def test_needs_a_second_at_rest():
    assert run([START] * 3 + [seated()] * 10) == 0.0


def test_moving_stack_scores_zero():
    f = [seated(dy=0.004 * i / 25) for i in range(REST)]
    assert run([START] * 3 + f) == 0.0


def test_quarter_turn_is_not_an_extra_error_but_real_yaw_is():
    assert run([START] * 3 + [seated(yaw=math.radians(2))] * REST) == 1.0


def test_demo_waypoints_are_within_reach():
    worst = mech.check_plan(N.demo())
    assert all(mm < 10 and deg < 5 for mm, deg in worst.values()), worst


if __name__ == "__main__":
    n = 0
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            n += 1
    print(f"{n} passed")
