# BLUR — MuJoCo model (from kinematic rig v35)

A physics model of BLUR generated directly from rig v35, with the same body tree, joint axes, link lengths, parts and colours.
Every value below comes from the parts list or the rig.

## Run it

```bash
pip install mujoco numpy
python replay.py                          # viewer: BLUR standing (motors alone, real physics)
python replay.py --move gallop            # physics replay of a rig move
python replay.py --move flip --kinematic  # the rig's exact animation, no physics, for side-by-side
python replay.py --move jump --guide 1    # physics + a soft guide on the body (shows the help the move needs)
python replay.py --report                 # headless pass/fail for every move -> report.json
python -m mujoco.viewer --mjcf scene_flat.xml   # plain viewer, drag joints / apply forces yourself
```

Moves: `stand sit lay a b c d walk gallop happy sweep hop jump flip stairs couch pounce tgrab`
(`jump` loads the 750 mm table, `stairs` the staircase, `couch` the couch, `pounce` / `tgrab` the ball).

## What's in the model

| | |
|---|---|
| Bodies / joints | 76 bodies, 62 hinges + free base: 4 × (abduction, girdle lift, girdle rock, hip/shoulder, knee/elbow, hock/wrist), 6 × spine (bend + twist), 9 × tail (pitch + yaw), neck lift/pan/tilt, jaw, 4 gripper fingers |
| Mass | 3.24 kg total, from the parts list: motors, gearboxes, bearings, encoders, both 6S packs, every board, plus 0.70 kg of prints, carbon, wiring, fasteners and springs spread over the frame |
| Actuators (26) | position servos with the real peak torque after the 16:1: **2806.5 hips/knees 4.0 N·m** (40 A), **4015 abduction 3.8 N·m**, **2204 joints 0.8 N·m**; reflected rotor inertia as armature |
| Spine | 6 vertebrae, each a bend + twist hinge with TPU 95A disc stiffness (8.5 / 5.7 N·m/rad); front + rear cable drums drive the bend through tendons (~1 N·m per vertebra), twist motor drives all six |
| Tail | 9 universal joints; pitch + yaw winches pull cables (160 N max) whose length is Σ vertebra-radius × angle — the same coupling as the real tendons |
| Springs | 12 coil-overs (0.8 × 8 × 30 mm, 1.35 N/mm, push-only), Achilles pack (McMaster 9044K375, 1.6 N/mm, pull-only), triceps (9654K211), knee extensor cables over the patella, TPU girdle bushings |
| Gripper | N20 lead-screw finger drive, 4 fingers coupled |
| Contact | TPU paw spheres (µ 1.2), bones, motor housings, packs, ribcage, head, tail; no self-collision (the rig's sweep already proves 0 self-collisions) |
| Sensors | IMU (quat, gyro, accel) at the BMI270 position, foot touch sensors |

Units: metres, kilograms, radians. Z up, X forward. Rig `FR` = `FR_*` here.

## Things the physics found that the animation can't show

1. **The passive parts need preload.** At 3.2 kg the girdles, spine discs and tail sag about 30° if their springs just rest at the stand pose.
   The model sets each passive joint's preload (`springref`) so stand is a true equilibrium. On the real robot that is the coil-over seat
   setting and TPU disc pre-shape. Biggest static loads: **hind girdle lift 0.83 N·m, hind girdle rock 0.80 N·m, front girdle rock 0.50 N·m,
   front spine discs 0.47 N·m.** With these preloads it stands on its own (3 mm settle).
2. **Open-loop replay is not a controller.** The replay plays the rig's joint angles through position servos with no balance feedback.
   Slow moves can pass like this; dynamic ones (gallop, jump, backflip) need a real controller (MPC or RL policy). For those, the report's
   **residual** column is the honest number. It's the help, as % of body weight, the body needs beyond what the motors, springs and ground give.
   Low residual + no torque saturation = the hardware can do it once a controller is trained.

## Files

- `blur.xml`: the robot (included by every scene)
- `scene_flat.xml`, `scene_table.xml`, `scene_stairs.xml`, `scene_couch.xml`, `scene_pounce.xml`, `scene_tailgrab.xml`
- `meshes/`: 423 STL meshes, one per body per colour, exported from the rig
- `moves.npz`: every rig move as joint targets + body path, 100 Hz
- `replay.py`: viewer, physics replay, report
- `report.json`: results of `--report`

## Report (python replay.py --report)

| Move | Verdict | Open-loop path error | Residual peak (% body weight) | Limiting motors |
|---|---|---|---|---|
| stand | PASS | 8 mm | 4% | none near limit |
| sit | PASS w/ controller | 76 mm | 21% | rear spine drum 56% |
| lay | PASS | 47 mm | 83% (guide only) | spine twist at limit |
| tgrab | PASS | 15 mm | 6% | none near limit |
| walk / happy | NEEDS WORK | falls open-loop | 80% | front knee + wrist 2204 saturate |
| stairs / couch | NEEDS WORK | ~1 m | 73–198% | front wrist 2204, knees |
| hop / jump / flip / pounce | NEEDS WORK | 0.4–0.9 m | 190–335% | wrists (2204) + rear knees saturate |
| gallop / sweep | NEEDS WORK | falls | 360–435% | everything saturates |

Main bottleneck: the 0.8 N·m 2204 wrist/hock joints hit their limit in every dynamic move, then the knees.
