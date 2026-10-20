#!/usr/bin/env python3
"""Run the shipped jump ceiling + a full holding-torque check (all actuators) in a variant dir.  python spring_checks.py <variant_dir>"""
import os, sys, json, numpy as np, mujoco
VD = os.path.abspath(sys.argv[1]); os.chdir(VD)
src = open(os.path.join(VD, 'physics_checks.py')).read().split("\nif __name__ == '__main__':")[0]
src = src.replace("HERE = os.path.dirname(os.path.abspath(__file__))", f"HERE = {VD!r}")
exec(src)
def hold_full():
    out = {}
    for mv in ['stand', 'sit', 'lay']:
        mv_, m, d, tr, names, tau0, W = setup(mv); tr.set_state(0); acc = []; q0 = d.qpos.copy()
        for k in range(int(3 / m.opt.timestep)):
            tr.control(min(k * m.opt.timestep, tr.T), 0); mujoco.mj_step(m, d)
            if k * m.opt.timestep > 2: acc.append(np.abs(d.actuator_force.copy()))
        a = np.mean(acc, 0)
        out[mv] = dict(all={n: round(float(v), 3) for n, v in zip(names, a) if 'winch' not in n},
                       max=max((round(float(v), 3), n) for n, v in zip(names, a) if 'winch' not in n),
                       base_drop_mm=round(float((q0[2] - d.qpos[2]) * 1000), 1))
    return out
# spring torque at the jump crouch (frame of lowest base z), for the record
mv, m, d, tr, names, tau0, W = setup('jump'); f0 = int(np.argmin(NPZ['jump__base'][:, 2])); tr.set_state(f0 * tr.dt)
crouch = {m.tendon(i).name: round(float(abs(d.ten_length[i] - m.tendon_lengthspring[i][1]) * m.tendon_stiffness[i]), 3) for i in range(m.ntendon) if m.tendon(i).name.startswith('jumpspring')}
R = dict(variant=os.path.basename(VD), spring_crouch_Nm=crouch, jump=check_jump(), hold=hold_full())
json.dump(R, open(os.path.join(VD, 'spring_checks.json'), 'w'), indent=1, default=float)
print(os.path.basename(VD), 'jump', R['jump']['ceiling_feet_tucked_mm'], R['jump']['ceiling_rig_stance_mm'], 'hold max', {k: v['max'] for k, v in R['hold'].items()}, 'crouch', crouch)
