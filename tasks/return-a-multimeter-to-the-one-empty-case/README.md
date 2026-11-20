# Put a returned multimeter into the one empty case on a shelf of identical cases

Three identical hard cases with opaque sliding lids stand side by side on the bench. Two hold a handheld digital
multimeter; one is empty. A third multimeter, just returned, lies on the bench. The robot is asked only to put it away
in its case, so it has to open cases to find the empty one, put the meter in flat, and leave every lid shut. Putting a
meter into an occupied case, or on top of the meter in it, is a violation.

| File | What |
|---|---|
| `cases.py` | The task file: objects, scoring (`score`) and the scripted demo (`demo`) |
| `mech.py` | Mechanisms the task file uses (sliding-lid joints, compound shapes) |
| `test_cases.py` | Offline checks of every scoring outcome and of the demo's reach |
| `BUILD.md` | How to build the three cases for a real trial |
| `task.json` | The submission's details |

## Layout variants

An evaluation is three trials, one per layout, run with `CASES_VARIANT=0`, `1` and `2` set in the environment (for example `CASES_VARIANT=1 inspect-robots run ...`); the task score is the mean of the three. 

`VARIANT` at the top of `cases.py` picks which case is empty: `0` the middle one (the submitted layout and the one the
videos show), `1` the right-hand case, `2` the left-hand case. Each variant is one fixed layout; `-E object_jitter=0`
keeps it fixed. The scripted demo opens the cases from the left until it reaches the empty one (shutting each occupied
case again), then places the meter and shuts that lid; it reads no simulator state at runtime.

## Run a trial

```bash
INSTR="Put the returned multimeter away in its case."
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=cases.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score --max-steps 2000 --store-frames --log-dir ~/yam-eval
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=cases.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score --max-steps 1300 --store-frames --log-dir ~/yam-eval
```

Offline checks (no simulator): `python test_cases.py`

## Runtime and seeds

- Run limit 2,000 control steps (100 s at 20 Hz); the scripted demo takes about 62 s.
- Runs must use `-E control_hz=20`: the episode clock runs on control steps.
- No random seed: each layout is fixed (`object_jitter=0`); `CASES_VARIANT` selects the layout and all three are run.

## Scoring

Only the returned meter's position, the three lids' positions and the stored meters' positions are used, all
measurable with a tape or a video. In the empty case: the returned meter lies flat (within 20 degrees of level) inside
the empty case's recess. At rest: moved less than 3 mm over the last second, both grippers at least 7 cm from it.
Closed lid: within 5 mm of its closed position. Violation: the returned meter rests (0.5 s or more) inside an
occupied case's recess or on top of the meter stored in it; it latches. 0: doing nothing, a violation, anywhere else,
held, or not at rest. Otherwise with the meter at rest in the empty case: 1.0 with all three lids closed, 0.5 with one
lid open, 0.25 with two or three open. A stored meter moved 2 cm or more caps the score at 0.25.

## Real trials

Use three identical cases and three identical meters (see `BUILD.md`); a dead or dummy meter is fine. Reset by
closing all lids and putting the meters back as in the layout.

## License

MIT.
