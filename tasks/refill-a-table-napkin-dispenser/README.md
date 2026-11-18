# Refill a table-top napkin dispenser with a new stack

A stainless open-top table-top napkin dispenser stands empty, its spring-loaded follower pushed forward against the
front plate; a new stack of 250 interfolded beverage napkins stands beside it. The robot is asked only to refill the
dispenser so it is ready for customers. Restocking napkin containers is a core O*NET task of waiters and waitresses and
of fast food and counter workers (stocking service stations with paper products). It needs both hands: one pulls the
follower back against its spring and holds it, the other sets the stack in square behind the front lips; the follower
is then let go and presses the stack against the lips.

| File | What |
|---|---|
| `napkins.py` | Task file: objects, scoring (`score`), scripted demo (`demo`) |
| `mech.py` | Mechanisms used (compound bodies, the sprung slide for the follower) |
| `test_napkins.py` | Offline checks of every scoring outcome and the demo's reach |
| `task.json` | The submission's details |

## Run a trial

```bash
INSTR="Refill the napkin dispenser with the new stack so it's ready for customers."
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=napkins.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=napkins.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score --max-steps 400 --store-frames --log-dir ~/yam-eval
```

Offline checks (no simulator): `python test_napkins.py`

## Runtime and seeds

- Run limit 1,200 control steps (60 s at 20 Hz); the scripted demo takes about 16 s.
- Use `-E control_hz=20` (the episode clock runs on control steps) and `-E object_jitter=0` (fixed layout, no seed).

## Scoring

See the `scoring` field of `task.json`: 1.0 for the stack seated square behind the lips with the follower pressing it,
still for 1 s with both grippers let go; 0.5 for a looser seating; 0.3 if the left gripper still holds the follower;
0 otherwise, including doing nothing.

## Real-world build

Table-top dispenser of 4 mm stainless sheet (or any open-top spring-follower napkin dispenser of similar size with a
retaining-lip front); stack of 250 interfold beverage napkins, about 12.7 x 10 x 7.5 cm, about 0.3 kg.

## License

MIT.
