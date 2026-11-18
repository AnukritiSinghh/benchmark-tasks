# Set a round-bottom flask down on a cork ring after the reaction

A 250 mL round-bottom flask sits neck up in a heating mantle after a reaction. The robot is asked only to take it off
the mantle and put it down. A round-bottom flask has no flat base: set on the bench it rolls on any nudge and breaks,
so chemists seat it in a cork ring. The instruction doesn't say so; the robot has to see that the flask is round and use
the ring on the bench. Contents, heat and breakage are not simulated.

| File | What |
|---|---|
| `corkring.py` | The task file: objects, scoring (`score`) and the scripted demo (`demo`) |
| `mech.py` | Mechanisms the task file uses (compound rigid bodies, welded marker, tilted demo grips) |
| `test_corkring.py` | Offline checks of every scoring outcome and of the demo's reach |
| `task.json` | The submission's details |

## Run a trial

```bash
INSTR="Take the flask off the mantle and put it down."
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=corkring.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=corkring.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score --max-steps 500 --store-frames --log-dir ~/yam-eval
```

Offline checks (no simulator): `python test_corkring.py`

## Runtime and seeds

- Run limit 1,200 control steps (60 s at 20 Hz); the scripted demo takes about 21 s.
- Runs must use `-E control_hz=20`; `-E object_jitter=0` fixes the layout, so no seed is needed.

## Real-world trial

Standard lab items: a 250 mL round-bottom flask (24/29 neck), a 250 mL heating mantle, a 90 mm cork ring, a 250 mL
beaker. Use an empty, cool flask; mark the ring's position on the bench.

## Scoring

Sphere centre = the flask's 85 mm sphere's centre. Moved: sphere centre 3 cm or more from where it started. On the
bench: the sphere's lowest point within 3 mm of the bench top. Violation (after it has been moved): resting on the
bench for 0.5 s or more, or dropped (released from more than about 2 cm up; centre falling 3 cm or more in one control
step). 1.0: no violation, and at the end the flask rests in the cork ring (sphere centre within 15 mm of the ring's axis
and 1 cm of the height it sits at in the ring, 5.0 cm above the bench; neck within 20 degrees of vertical), moved less
than 3 mm over the last second, both grippers 10 cm or more from the sphere's centre. Otherwise 0, including doing
nothing, leaving or returning it to the mantle, setting it on the bench anywhere, dropping it, still holding it, or
knocking it off the bench.

## License

MIT.
