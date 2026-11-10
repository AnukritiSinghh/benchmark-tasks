# Press a loose AED pad down, then stand clear before pressing SHOCK

Bystanders and first responders use AEDs on people in cardiac arrest. The American Heart Association's steps are to make sure the pads are stuck to the chest and that no one touches the patient while the shock is delivered ('clear'). A loose pad needs a hand on the chest, and that hand has to come away before the button is pressed: the arm that pressed the pad must be clear before the other arm delivers the shock.

| File | What |
|---|---|
| `aed.py` | The task file: objects, scoring (`score`) and the scripted demo (`demo`) |
| `mech.py` | Mechanisms the task file uses (a sliding pad that stays where pressed, a sprung button, a flashing light, fingertip positions) |
| `test_aed.py` | Offline checks of every scoring outcome |
| `task.json` | The submission's details |

`mech.py` stays inside this folder: the task file imports it and it hooks the `yam_arena` scene builder and step
when the simulator starts. No change to `yam_arena` itself is needed.

## Run a trial

With the simulator set up by the simulation walkthrough, from this folder:

```bash
INSTR="The AED advises a shock. Its upper pad has come loose from the training manikin's chest: first press it flat. Then, as the American Heart Association teaches, make sure no one is touching the patient: with both grippers at least 15 cm from the manikin, press the flashing orange SHOCK button."
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=aed.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=aed.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score --max-steps 500 --store-frames --log-dir ~/yam-eval
```

Offline checks (no simulator): `python test_aed.py`

## Runtime and seeds

- Run limit 1,200 control steps (1 minute at 20 Hz); the scripted demo takes about 18 s.
- Runs must use `-E control_hz=20`: the episode clock runs on control steps.
- `-E object_jitter=0` fixes the layout, so no seed is needed.

## Scoring

Flat: the loose pad within 1 mm of the chest. Pressed: SHOCK down at least 3 mm. Clear: both grippers' fingertips (the point between them) at least 15 cm from every part of the manikin. Violation: SHOCK pressed while the pad is not flat, or a fingertip less than 15 cm from the manikin at any moment from the press until 1 s after it. 1.0: no violation; the pad flat, SHOCK pressed, and both grippers clear for that second. 0.5: no violation; the pad flat, SHOCK not pressed. Otherwise 0, including doing nothing or refusing.

## License

Released under the MIT License.
