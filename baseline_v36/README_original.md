# BLUR v36: MuJoCo model (from kinematic rig v36)

This physics model is generated directly from rig v36: the same body tree, joint axes, link lengths, parts and colours.
Masses, motor torques and spring rates come from the parts list.

## What changed from v35

| | v35 | v36 |
|---|---|---|
| Wrists + hocks | 2204, 0.8 N·m, ~35 rad/s (too slow for the gallop) | **2806.5 + 16:1 + B-G431B-ESC1, 4.0 N·m, ~170 rad/s** |
| Spine | 6 TPU-disc vertebrae, 2 cable drums + twist (2204, 1.0 / 0.8 N·m) | **3 direct-drive joints: S1 bend · twist · S2 bend, each a 4015 hollow gimbal + 16:1 (3.8 N·m) in the joint** |
| Jump springs | none | **steel extension springs on the shoulders and all four knees: slack at full leg extension, 2.0 N·m at the jump crouch** |
| Mass | 3.24 kg | **3.67 kg** |

## Run it

```bash
pip install mujoco numpy scipy
python replay.py                          # viewer: BLUR standing on its own motors (macOS: mjpython replay.py)
python replay.py --move gallop --kinematic   # the rig animation exactly, no physics
python replay.py --move walk              # physics replay of a rig move (open loop, no balance controller)
python replay.py --report                 # open-loop pass/fail for every move -> report.json
python physics_checks.py                  # what the HARDWARE can do, whatever controller runs it -> physics_checks.json
python -m mujoco.viewer --mjcf scene_flat.xml   # plain viewer
```

Moves: `stand sit lay a b c d walk gallop happy sweep hop jump flip stairs couch pounce tgrab`.

## Hardware verdict (physics_checks.py)

| Check | Result |
|---|---|
| Motor speed, every move | OK. Worst case is the neck at 63% of no-load speed in the gallop. The wrists/hocks need 70 rad/s and have 170. |
| Walk / happy walk (inverse dynamics, best foot forces) | OK. Legs at ≤ 0.6× torque. Spine S1 2.1 N·m, S2 1.4–1.5, twist 1.1–1.6, all against a 3.8 N·m limit. The v35 spine drums and twist were at or over their limit here. |
| Stand / sit / lay holding torque | Stand ≤ 1.3 N·m. Sit: rear knees 2.7 N·m (~27 A, so don't hold it for minutes). Lay: knees ≤ 2.8 N·m. |
| Table jump, 750 mm | **At the ceiling.** The upper bound is 751 mm, and only with the feet tucked under the hips (406 mm with the rig's splayed stance). Even a perfect push only just makes it. |
| Gallop | Speed and leg torque are OK. The rig's gallop animation itself is not physically consistent: 40% of frames need a "magic" force on the body. So spine torque for a real gallop can't be read from it. A rough estimate for a hard gallop is 4–5 N·m at S1, against a 3.8 N·m limit. |

Open-loop replay (`--report`) still fails the dynamic moves. That is expected: playing joint angles with no balance feedback is not a controller.
Stand, sit, lay and tail-grab pass open loop. Everything else is the controller's job (BrAInOS).

## Files

- `blur.xml`: the robot (included by every scene). Spine joints are `spine_s1`, `spine_twist`, `spine_s2`. Jump springs are the `jumpspring_*` tendons.
- `scene_*.xml`: flat, table (750 mm), stairs, couch, pounce (ball), tailgrab (ball)
- `meshes/`: STL per body per colour, exported from the rig
- `moves.npz`: every rig move as joint targets + body path, 100 Hz
- `replay.py`: viewer, physics replay, open-loop report
- `physics_checks.py`: hardware capability checks
- `report.json`, `physics_checks.json`: results
