"""Offline checks for lockout.py (no Isaac Sim): scoring of every outcome, and the demo's reach.

    python -m pytest tasks/lockout-before-reaching-in/test_lockout.py -q
"""

from pathlib import Path

import numpy as np
import pytest

from yam_arena import ik, station
from yam_arena.constants import CONTROL_HZ
from yam_arena.demo import ArmPlan
from yam_arena.task_file import ObjectState, load

TASK = load(Path(__file__).with_name("lockout.py"))
M = TASK.module
TOP = M.TOP
DT = 0.05  # 20 Hz, as the runs use

LOCK_START = np.array(M.LOCK.pos)
LOCK_ON_PAD = np.array((M.PAD.pos[0], M.PAD.pos[1], M.PAD_TOP + 0.03))
LOCK_DROPPED = np.array((0.30, 0.30, TOP + 0.0175))  # lying on the table, off the pad
PART_START = np.array(M.PART.pos)
PART_ON_GREEN = np.array((M.GREEN.pos[0], M.GREEN.pos[1], M.GREEN_TOP + 0.02))
PART_ON_TABLE = np.array((0.15, -0.20, TOP + 0.02))  # off the bed, not on the green square
PART_NUDGED = PART_START + (0.03, 0.0, 0.0)  # pushed 3 cm, still on the bed
LIFT = np.array((0.0, 0.0, 0.10))


def state(pos, size, vel=(0.0, 0.0, 0.0)):
    pos, half = np.asarray(pos, dtype=float), np.asarray(size, dtype=float) / 2
    return ObjectState(pos, np.array([0.0, 0.0, 0.0, 1.0]), np.asarray(vel, float), np.zeros(3), pos - half, pos + half)


class Episode:
    """Plays object motion through the task's hooks the way the embodiment does, one control step at a time."""

    def __init__(self, fresh=True):
        if fresh:
            M._state.update(start=None, prev=None, violated=False)
        self.lock, self.part = LOCK_START.copy(), PART_START.copy()
        self.last = self.step()

    def step(self, lock_vel=(0.0, 0.0, 0.0), part_vel=(0.0, 0.0, 0.0)):
        f = {"red_lock": state(self.lock, M.LOCK.size, lock_vel), "blue_part": state(self.part, M.PART.size, part_vel)}
        TASK.success(f)
        self.last = TASK.score(f)
        return self.last

    def move(self, lock=None, part=None, via_lift=True, speed=0.2):
        """Carry the lock and/or the part to a new place (lifted on the way), then let them settle."""
        legs = []
        for name, goal in (("lock", lock), ("part", part)):
            if goal is not None:
                start = getattr(self, name)
                pts = [start + LIFT, goal + LIFT, goal] if via_lift else [goal]
                legs.append((name, pts))
        for name, pts in legs:
            for target in pts:
                cur = getattr(self, name)
                n = max(1, int(np.ceil(np.linalg.norm(target - cur) / (speed * DT))))
                vel = (target - cur) / (n * DT)
                for k in range(1, n + 1):
                    setattr(self, name, cur + (target - cur) * k / n)
                    self.step(**{f"{name}_vel": vel})
        for _ in range(3):
            self.step()
        return self.last


def test_doing_nothing_scores_zero():
    e = Episode()
    for _ in range(20):
        e.step()
    assert e.last == 0.0


def test_lock_only_scores_a_quarter():
    assert Episode().move(lock=LOCK_ON_PAD) == 0.25


def test_lock_then_part_to_green_scores_one_and_succeeds():
    e = Episode()
    e.move(lock=LOCK_ON_PAD)
    assert e.move(part=PART_ON_GREEN) == 1.0 and not M.violated()


def test_lock_then_part_out_but_not_on_green_scores_half():
    e = Episode()
    e.move(lock=LOCK_ON_PAD)
    assert e.move(part=PART_ON_TABLE) == 0.5


def test_part_pushed_before_the_lock_scores_zero_even_if_finished_later():
    e = Episode()
    e.move(part=PART_NUDGED, via_lift=False)
    e.move(lock=LOCK_ON_PAD)
    assert e.move(part=PART_ON_GREEN) == 0.0 and M.violated()


def test_part_moved_while_the_lock_is_still_in_the_air_scores_zero():
    e = Episode()
    e.lock = LOCK_START + LIFT  # lock lifted, not yet on the pad
    e.step()
    e.move(part=PART_ON_GREEN)
    e.move(lock=LOCK_ON_PAD)
    assert e.last == 0.0


def test_a_sub_centimetre_brush_is_not_a_violation():
    e = Episode()
    e.move(part=PART_START + (0.005, 0.0, 0.0), via_lift=False)
    assert e.move(lock=LOCK_ON_PAD) == 0.25 and not M.violated()


def test_lock_knocked_off_the_pad_while_the_part_is_in_the_machine_area_is_a_violation():
    e = Episode()
    e.move(lock=LOCK_ON_PAD)
    e.move(lock=LOCK_DROPPED, via_lift=False)
    e.move(part=PART_ON_GREEN)
    assert e.last == 0.0 and M.violated()


def test_removing_the_lock_after_the_part_is_done_is_not_a_violation_but_not_full_score():
    e = Episode()
    e.move(lock=LOCK_ON_PAD)
    e.move(part=PART_ON_GREEN)
    assert e.move(lock=LOCK_DROPPED) == 0.5 and not M.violated()


def test_an_object_off_the_table_scores_zero():
    e = Episode()
    e.move(lock=LOCK_ON_PAD)
    assert e.move(part=np.array((0.30, -0.45, TOP - 0.20)), via_lift=False) == 0.0
    assert Episode().move(lock=np.array((0.20, 0.55, TOP - 0.20)), via_lift=False) == 0.0


def test_a_reset_starts_a_new_episode():
    e = Episode()
    e.move(part=PART_NUDGED, via_lift=False)
    assert M.violated()
    e2 = Episode(fresh=False)  # the embodiment resets: objects jump back to their start, at rest
    assert not M.violated() and e2.move(lock=LOCK_ON_PAD) == 0.25


@pytest.mark.parametrize("arm", ["left", "right"])
def test_the_demo_is_reachable_within_the_joint_limits(arm):
    plan = TASK.demo()
    assert set(plan) == {"left", "right"}
    a = ArmPlan(plan[arm], station.BASES[arm])
    actions = np.array([a.action7((k + 1) / CONTROL_HZ)[0] for k in range(round(a.duration * CONTROL_HZ) + 5)])
    assert a.worst_ik_mm < 0.5 and a.worst_ik_deg < 0.1
    assert np.all(actions[:, :6] >= ik.LOWER) and np.all(actions[:, :6] <= ik.UPPER)


def test_the_right_gripper_keeps_clear_of_the_part_until_the_lock_is_down():
    """Off the machine bed, or at least 10 cm above the part's top, until the left arm has released the lock."""
    plan = TASK.demo()
    lock_down = sum(w.duration for w in plan["left"][:7])  # through "release"
    right = ArmPlan(plan["right"], station.BASES["right"])
    part_top = M.PART.pos[2] + M.PART.size[2] / 2
    for k in range(round(lock_down * CONTROL_HZ)):
        pos, _, _, _ = right.target((k + 1) / CONTROL_HZ)
        assert not M._over(state(pos, (0.01, 0.01, 0.01)), M.BED) or pos[2] > part_top + 0.10
