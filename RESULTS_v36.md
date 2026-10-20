# BLUR v36 in MuJoCo: setup, verification, jump-spring sweep, bounding-gait spine test

Date: 2026-10-20. Same machine and venv as v35 (`blur_mujoco/.venv`, Python 3.12, mujoco 3.16.0, numpy 2.5.4,
scipy 1.18.1, matplotlib 3.11.2). Everything is under `~/Desktop/Arenda_Projects/BLUR/`:
`blur_mujoco_v36/` is the zip, untouched; `experiments_v36/` holds every script, variant and result named below.

## 1. Setup and verification

| check | result |
|---|---|
| unzip | `blur_mujoco_v36/` next to the v35 folder, 441 meshes, README read |
| scenes load | flat / table / stairs / couch: **74 bodies, 54 joints, 26 actuators**; pounce / tailgrab: **75 / 55 / 26** (the ball) |
| mass | **3.672 kg** (`body_subtreemass[1]`) |
| `python replay.py --report` | **matches the shipped report.json exactly**, every field of all 14 moves (rerun log: `baseline_v36/report_run.log`) |
| `python physics_checks.py` | **matches the shipped physics_checks.json exactly**, every number in speed / walk / jump / hold (`baseline_v36/physics_checks_run.log`) |
| viewers | `replay.py`, `replay.py --move gallop --kinematic` and `python -m mujoco.viewer --mjcf scene_flat.xml` all open and run on this Wayland desktop (only a harmless libdecor-gtk warning) |

No mismatch anywhere. The shipped JSONs were backed up to `baseline_v36/` before the reruns rewrote them with
identical content. `replay.py` is byte-identical to the v35 one.

### Open the viewers yourself (Linux: plain `python`, not `mjpython`)

```bash
cd ~/Desktop/Arenda_Projects/BLUR/blur_mujoco_v36
source ../blur_mujoco/.venv/bin/activate
python replay.py                              # standing on its own motors, real physics
python replay.py --move gallop --kinematic    # the rig's gallop animation, no physics
python -m mujoco.viewer --mjcf scene_flat.xml # plain viewer, drag joints / apply forces
```

## 2. Jump springs (5a)

Variants: `experiments_v36/make_spring_variants.py` scales all six `jumpspring_*` tendon stiffnesses by
T / 2.0 so each spring gives T N·m at the jump crouch (the shipped model is T = 2.0; the crouch torque was read
back from `ten_length` and `tendon_stiffness` at the lowest frame of the rig's jump and is 2.00 ± 0.001 for every spring).
Each variant directory is a full runnable copy (edited blur.xml, scene copies, symlinked meshes / moves / meta,
copied `physics_checks.py`), and `spring_checks.py` runs the shipped `check_jump()` plus a holding-torque check that
reports **every** actuator, not just the top four (`springs/k*/spring_checks.json`).

| crouch torque (N·m per spring) | stiffness × | table-jump ceiling, feet tucked (mm) | ceiling, rig stance (mm) | stand max hold | sit max hold | lay max hold | every hold < 2.5 N·m? |
|---|---|---|---|---|---|---|---|
| 0.0 (no springs) | 0 | 580 | 406 | 0.75 (FR_pitch) | 1.46 (spine_s2) | 1.12 (FL_abd) | yes |
| 0.5 | 0.25 | 614 | 406 | 0.67 (FR_abd) | 1.50 (spine_s2) | 1.06 (FL_abd) | yes |
| 1.0 | 0.5 | 660 | 406 | 0.69 (RR_knee) | 1.58 (RL_knee) | 1.47 (FR_knee) | yes |
| 1.5 | 0.75 | 706 | 406 | 0.99 (RR_knee) | 2.16 (RL_knee) | 2.12 (FR_knee) | yes |
| **1.7** | 0.85 | **724** | 406 | 1.12 (RR_knee) | **2.39** (RL_knee) | **2.38** (FR_knee) | **yes** |
| 1.8 | 0.9 | 733 | 406 | 1.18 (RR_knee) | 2.50 (RL_knee) | 2.51 (FR_knee) | no (2.504 / 2.51) |
| 1.9 | 0.95 | 742 | 406 | 1.24 (RR_knee) | 2.62 (RL_knee) | 2.64 (FR_knee) | no |
| 2.0 (shipped) | 1 | 751 | 406 | 1.30 (RR_knee) | 2.73 (RL_knee) | 2.77 (FR_knee) | no |
| 2.5 | 1.25 | 794 | 406 | 1.60 (RR_knee) | 3.30 (RL_knee) | 3.41 (FR_knee) | no |
| 3.0 | 1.5 | 835 | 406 | 1.89 (RR_knee) | 3.88 (RL_knee) | 4.00 (FR_knee, at the motor limit) | no |

Plots: `experiments_v36/results/springs_ceiling.png`, `experiments_v36/results/springs_hold.png`.

What the sweep says:
- The jump ceiling is linear in the spring rate, about **+45 mm per 0.5 N·m** of crouch torque; the springs alone
  are worth 171 mm (580 → 751) at the shipped rate.
- The price is paid in sit and lay, where the knees are flexed and the motors hold against the stretched springs:
  the sit / lay knee holding torque rises about **+0.57 N·m per 0.5 N·m** of spring. Stand barely moves.
- The rig-stance ceiling (406 mm) does not depend on the springs at all. Checked at the crouch: with the rig's
  splayed stance the vertical foot force of every leg is capped by the **abduction motor** (3.8 N·m, about 56–59 N per
  leg) while the sprung pitch / knee joints could give 94–121 N; no jump spring acts on abduction, so the springs
  only pay off with the feet tucked under the hips.

**Verdict (a): 1.7 N·m at the crouch is the stiffest spring that keeps every holding torque under 2.5 N·m**
(sit 2.39, lay 2.38). At 1.8 N·m the knees are at 2.50 / 2.51, i.e. on the line. With 1.7 N·m springs the table-jump
ceiling drops from 751 to **724 mm, below the 750 mm table**, so the 2.5 N·m holding budget and the 750 mm table
cannot both be met by the spring rate alone. Either accept 2.73–2.77 N·m (about 27 A) in sit and lay for short
holds with the shipped 2.0 N·m springs, or keep 1.7 N·m and lower the table / raise the tuck. The 3.0 N·m variant
puts the lay knee at the 4.0 N·m motor limit, so lay is not even holdable there.

## 3. Spine in a bound / gallop (5b)

The rig's gallop is not physically consistent (the shipped check says 40 % of its frames need a magic force), so I
built one instead: `experiments_v36/bound_gait.py`.

How the gait is built, so you can judge it:
- Planar bound: rear pair stance → flight → front pair stance → flight, stride frequency f, constant forward speed v.
- Vertical body motion is **derived from physics, not drawn**: each stance applies a half-sine vertical force whose
  impulse carries half the stride's weight, so the flight phases are ballistic and the stride is periodic. The chest
  height averages the rig's stand height (268 mm). Lateral motion zero, speed exactly constant.
- Feet: stance feet fixed to the ground at a touchdown point that puts them under the hip (plus a free offset) at
  mid-stance; swing feet follow a cosine arc with 40 mm clearance. Leg angles by IK on pitch / knee / dist with the
  stand pose in the nullspace (worst reach error 1 mm). Abduction, girdles, neck, jaw and tail stay at the stand pose.
- Free parameters: duty factor, chest pitch amplitude and phase, spine S1 and S2 bend amplitudes and phases, rear and
  front foot offsets. Nelder-Mead tunes them (2–3 seeds × 250–500 evaluations per cell) to minimise, in order of
  weight, the LP's magic force (inconsistency), the magic torque, the spine torque / 3.8 and the leg load.
- Evaluation is MuJoCo inverse dynamics on the middle stride at 200 Hz, then **the foot-force LP copied verbatim from
  `check_walk()`** (friction cone μ = 1, speed-derated motor limits, minimise the peak leg load factor u, magic force
  penalised ×50). Spine torques are what remains at the spine dofs after the best foot forces are subtracted.
  "Consistent frames" uses the shipped 15 % BW threshold.

Caveat on what the LP can and cannot do here: with a two-foot stance the six body-wrench equations are matched by
six foot-force components, so the LP has almost no freedom; the spine torque is set by the body trajectory, not by
the force split. The gait optimiser is where the freedom is, and it is a local search: the numbers below are the
best found, not a proof of a minimum. They are, however, consistent across 27 independent optimisations, and they
agree with the analytic floor at the end of this section.

### Results, 3 speeds × 3 stride frequencies (`results/bound/`, 2 seeds × 250 evaluations each)

| v (m/s) | f (Hz) | duty | flight each (ms) | consistent frames % | magic mean % BW | S1 p95 / peak | S2 p95 / peak | twist p95 / peak | stance leg load u p95 / peak | swing leg load p95 / peak | peak Fz (BW) | spine ≤ 3.8? | legs ≤ 1.0? |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 1.5 | 3.0 | 0.40 | 33 | 96 | 1.6 | 10.02 / 10.61 | 7.91 / 8.36 | 0.02 / 0.02 | 1.03 / 1.34 | 0.74 / 0.87 | 2.13 | NO | NO |
| 1.5 | 3.5 | 0.33 | 50 | 84 | 5.3 | 6.64 / 7.19 | 5.00 / 6.36 | 0.03 / 0.03 | 0.94 / 1.32 | 0.70 / 0.79 | 2.32 | NO | yes |
| 1.5 | 4.0 | 0.38 | 31 | 94 | 2.0 | 8.21 / 8.73 | 5.80 / 8.52 | 0.02 / 0.02 | 1.01 / 1.60 | 0.82 / 0.93 | 2.20 | NO | NO |
| 2.0 | 3.0 | 0.31 | 63 | 78 | 7.1 | 9.11 / 10.03 | 6.24 / 7.02 | 0.03 / 0.03 | 0.98 / 1.32 | 0.75 / 0.95 | 2.47 | NO | yes |
| 2.0 | 3.5 | 0.29 | 59 | 78 | 6.7 | 10.62 / 12.02 | 6.02 / 7.14 | 0.03 / 0.03 | 0.96 / 1.28 | 0.80 / 0.94 | 2.55 | NO | yes |
| 2.0 | 4.0 | 0.29 | 53 | 78 | 6.6 | 8.88 / 9.75 | 6.62 / 8.10 | 0.03 / 0.03 | 0.87 / 1.57 | 0.88 / 1.03 | 2.63 | NO | yes |
| 2.5 | 3.0 | 0.22 | 93 | 72 | 12.5 | 11.06 / 13.25 | 5.75 / 7.49 | 0.04 / 0.04 | 0.95 / 1.13 | 0.90 / 1.00 | 3.00 | NO | yes |
| 2.5 | 3.5 | 0.26 | 68 | 67 | 11.0 | 10.50 / 12.20 | 6.17 / 7.32 | 0.03 / 0.04 | 1.03 / 1.46 | 0.95 / 1.16 | 2.66 | NO | NO |
| 2.5 | 4.0 | 0.29 | 52 | 75 | 9.7 | 9.74 / 11.43 | 8.02 / 8.77 | 0.03 / 0.03 | 1.06 / 1.71 | 1.08 / 1.30 | 2.67 | NO | NO |

### Refined 3.5 Hz runs (`results/bound_refine/`, 3 seeds × 500 evaluations, duty factor allowed up to 0.45)

| v (m/s) | duty | flight each (ms) | consistent % | magic mean % BW | S1 p95 / peak | S2 p95 / peak | twist p95 | stance u p95 / peak | swing p95 / peak | peak Fz (BW) | analytic S1 floor (2.14 × Fz) |
|---|---|---|---|---|---|---|---|---|---|---|---|
| 1.5 | 0.34 | 45 | 84 | 5.0 | **6.17** / 6.93 | 6.17 / 7.17 | 0.02 | 0.85 / 1.45 | 0.75 / 0.85 | 2.28 | 4.87 |
| 2.0 | 0.31 | 56 | 78 | 7.0 | **7.61** / 8.42 | 6.73 / 7.06 | 0.03 | 1.09 / 1.47 | 0.82 / 0.96 | 2.48 | 5.31 |
| 2.5 | 0.25 | 71 | 64 | 13.6 | **6.07** / 8.24 | 6.07 / 6.97 | 0.03 | 1.28 / 1.33 | 0.93 / 1.09 | 2.60 | 5.56 |

The 2.5 m/s refined row is the least trustworthy (64 % consistent frames, legs at 1.28): the optimiser traded
consistency for spine torque there. The optimiser never chose the longest allowed stance; longer stance lowers the
peak force but costs consistency and leg load in this gait family.

Leg joint torques at 3.5 Hz (N·m, p95 / peak; limits abd 3.8, pitch / knee / dist 4.0):

| v | FR_abd | FR_pitch | FR_knee | FR_dist | RR_abd | RR_pitch | RR_knee | RR_dist |
|---|---|---|---|---|---|---|---|---|
| 1.5 | 3.06 / 3.37 | 3.24 / 3.78 | 2.31 / 2.57 | 1.39 / 1.46 | 2.10 / 5.00 | 1.64 / 1.76 | 2.57 / 4.80 | 0.91 / 1.03 |
| 2.0 | 2.96 / 3.58 | 3.35 / 3.68 | 2.41 / 2.81 | 1.34 / 1.37 | 3.39 / 3.66 | 2.37 / 2.65 | 3.56 / 4.75 | 0.87 / 1.36 |
| 2.5 | 3.24 / 3.55 | 3.74 / 4.41 | 2.64 / 3.21 | 1.72 / 1.80 | 3.49 / 3.93 | 3.52 / 4.01 | 3.10 / 5.12 | 2.02 / 2.19 |

The new 2806.5 wrists / hocks are the lightest-loaded leg joint in every bound (0.9–2.0 N·m against 4.0). The
rear knees are the heaviest, peaking at 4.8–5.1 N·m at every speed.

Plots: `results/bound_spine_vs_speed.png` (S1 / S2 / twist p95 and peak vs speed, per frequency),
`results/bound_leg_vs_speed.png`, `results/bound_spine_trace.png` (torque through the stride at 3.5 Hz),
filmstrips `results/bound/v*_f*.png` and `results/bound_refine/v*_f3.5.png`.

### Why the spine number is what it is (analytic cross-check)

Everything in front of the S1 joint (chest, collar, both front legs, neck, head) is **2.191 kg**, and its centre of
mass sits **0.100 m** ahead of the S1 axis in the stand pose. Just hanging there it loads S1 with
2.19 × 9.81 × 0.100 = **2.14 N·m**, which is exactly the shipped walk check's 2.12 N·m p95. In a bound, during rear
stance the only ground contact is the rear feet, so the whole front half has to be accelerated through the spine:
S1 ≥ 2.14 N·m × (peak vertical force / body weight), plus whatever the pitch and spine accelerations add. With two
feet carrying the whole stride's weight in a half-sine stance the peak force is π / (4 × duty) body weights, so

| duty factor | flight per half-stride at 3.5 Hz | peak Fz (BW) | S1 floor (N·m) |
|---|---|---|---|
| 0.25 | 71 ms | 3.14 | 6.7 |
| 0.33 | 48 ms | 2.38 | 5.1 |
| 0.40 | 29 ms | 1.96 | 4.2 |
| 0.44 | 17 ms | 1.78 | 3.8 |

The 3.8 N·m 4015 is reached only when the flight phase has shrunk to about 17 ms, and that is before any dynamic
term. The optimised gaits land at 1.3–2.2 × this floor. Speed enters only weakly (through swing-leg load and
horizontal forces); the gait type sets the spine load.

**Verdict (b): with the v36 spine (4015 + 16:1, 3.8 N·m) there is no bounding speed in the 1.5–2.5 m/s range that
the hardware physically allows.** The best gait found needs spine S1 = 6.2 N·m p95 (6.9 peak) at 1.5 m/s, 7.6 at
2.0, 6.1 at 2.5 (the last with 64 % consistency), i.e. 1.6–2.0 × the motor, and the analytic floor for any bound with
a real flight phase is 4.2–6.7 N·m. The legs are at their limit in the same gaits (stance load 0.85–1.28, rear
knee peaks 4.8–5.1 N·m vs 4.0) even at 1.5 m/s. What the spine and legs would need for a bound with a 50 ms flight
phase at 1.5–2.0 m/s: **S1 and S2 about 8 N·m** (p95 6–8, peaks 7–8.5 N·m), and roughly **5 N·m knees**.
What cannot be determined from this test: the top speed of a non-bounding gait (trot, with diagonal pairs and no
flight, keeps the front half partly supported by a front foot at all times and does not have this floor), and
whether an RL controller could find a bound outside the sinusoidal pitch / spine family searched here. The analytic
floor applies to any two-feet-at-a-time stance regardless of controller.

## 4. Verdicts in one place

- **Jump springs:** 1.7 N·m at the crouch (tendon stiffness × 0.85) is the stiffest rate that keeps every sit / lay /
  stand holding torque under 2.5 N·m (2.39 / 2.38 / 1.12). It drops the table-jump ceiling from 751 to 724 mm, under
  the 750 mm table. The shipped 2.0 N·m springs clear the table only by 1 mm and only with the feet tucked, at the
  cost of 2.73–2.77 N·m sit / lay holds.
- **Gallop:** the spine, not speed, is the cap. A bound at 1.5 m/s already needs S1 ≈ 6 N·m against 3.8, and the
  legs are at their torque limit in the same gait. The direct-drive spine fixed the walk (2.1 N·m, 0.6× legs), not
  the gallop. v36 is a walker with jump and sit tricks; a real bound needs roughly twice the spine torque.

## Files

```
BLUR/
  RESULTS_v36.md                     this file
  blur_mujoco_v36/                   the zip, untouched (report.json / physics_checks.json rewritten with identical content)
  baseline_v36/                      shipped JSONs + rerun logs + viewer logs
  experiments_v36/
    make_spring_variants.py          builds springs/k*/ (full runnable copies, stiffness scaled)
    spring_checks.py                 shipped check_jump() + all-actuator holding torque -> springs/k*/spring_checks.json
    plot_springs.py                  springs table + results/springs_*.png
    bound_gait.py                    the bounding-gait builder / optimiser / LP evaluator / renderer
    run_bound_grid.sh, run_bound_refine.sh
    plot_bound.py                    tables (results/bound_tables.md) + results/bound_*.png
    results/bound/, results/bound_refine/   per-cell JSON (summary + per-frame trace + all seeds), logs, filmstrips
```
