# Put a gel in the tank with its wells at the black electrode

A set agarose gel lies in its UV-transparent casting tray on the table, turned 90 degrees to a horizontal mini-gel
tank (red (+) end cap on the left, black (-) on the right). The robot is asked only to put the gel in the tank, ready
for the buffer and samples. DNA runs toward the red electrode, so the tray's wells end must go at the black end; the
robot has to turn the tray the right way (a quarter turn one way, not the other) while carrying it over the tank's wall.

| File | What |
|---|---|
| `gelbox.py` | The task file: objects, scoring (`score`), scripted demo (`demo`) |
| `mech.py` | Helper that makes the tank, tray and gel multi-part rigid bodies |
| `test_gelbox.py` | Offline checks of every scoring outcome and of the demo's reach |
| `BUILD.md` | Parts list and build for the real rig |
| `task.json` | The submission's details |

## Run a trial

```bash
INSTR="The gel has set: put it in the electrophoresis tank, ready for the buffer and samples."
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=gelbox.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score --max-steps 900 --store-frames --log-dir ~/yam-eval
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=gelbox.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score --max-steps 500 --store-frames --log-dir ~/yam-eval
```

Offline checks (no simulator): `python test_gelbox.py`

## Runtime and seeds

Run limit 900 control steps (45 s at 20 Hz); the scripted demo takes about 16 s. Use `-E control_hz=20` and
`-E object_jitter=0` (fixed layout, no seed needed).

## Scoring

See the `scoring` field of `task.json`: 1.0 for the tray seated on the deck (centre within 5 mm, bottom within 2 mm of
the deck top, level within 3 degrees) with the wells end within 5 degrees of the black end, still for 1 s with both
grippers 7 cm clear; 0.5 for resting on the deck (15 mm, 5 mm, 10 degrees) with the wells within 15 degrees of the
black end; 0 for anything else, for the wells at the red end, and whenever the gel leaves the tray (latched).
In a real trial, check with a ruler and the video: tray ends between the tape marks, wells end toward the black jack.

## License

MIT.
