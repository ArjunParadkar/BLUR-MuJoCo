#!/usr/bin/env python3
"""
BLUR physics checks: can the HARDWARE do it, no matter what controller (BrAInOS) runs it?

  python physics_checks.py            # all checks, ~5 min on a laptop CPU -> physics_checks.json

1. SPEED     every actuator's required speed in every rig move vs its no-load speed on a sagged 6S pack
2. WALK      inverse dynamics of the rig's walk / happy walk with the best possible foot forces (an LP):
             leg-motor load and spine-motor torque. Also reports how physically consistent the animation is.
3. JUMP      table-jump energy ceiling: every leg at its torque limit all the way from the crouch to straight legs,
             balanced front/back so the body doesn't flip, feet tucked under the hips. Includes every spring.
             This is an UPPER BOUND (no motor speed losses, perfect timing).
4. HOLD      motor torque just to hold stand / sit / lay (heat budget: 2806.5 + 16:1 ~ 1 N·m per 10 A)
"""
import os, json, numpy as np, mujoco
from scipy.optimize import linprog
HERE = os.path.dirname(os.path.abspath(__file__)); __file__ = os.path.join(HERE, 'replay.py')
exec(open(__file__).read().split('if __name__')[0])
LEGS = ['FR', 'FL', 'RR', 'RL']
V = 20.0   # sagged 6S under load
W0 = {'pitch': 1300, 'knee': 1300, 'dist': 1300, 'abd': 100, 'spine': 100, 'neck': 260, 'jaw': 260}   # motor KV (4015 KV assumed ~100)
def w0(n):
    if 'winch' in n: return 260 * V / 16 * 2 * np.pi / 60 * 0.005
    for k, kv in W0.items():
        if k in n: return kv * V / 16 * 2 * np.pi / 60
    return 1e9

def setup(move):
    mv, m, d, base, q, dt = load(move); tr = Track(m, d, base, q, dt)
    names = [m.actuator(i).name for i in range(m.nu)]
    return mv, m, d, tr, names, np.abs(m.actuator_forcerange[:, 1]).copy(), np.array([w0(n) for n in names])

def com(m, d): return d.subtree_com[1].copy()

# ---------------------------------------------------------------- 1 speed
def check_speed():
    mv, m, d, tr, names, tau0, W = setup('stand'); out = {}
    for mvn in ['walk', 'happy', 'gallop', 'sweep', 'hop', 'jump', 'flip', 'pounce', 'couch', 'stairs', 'tgrab', 'sit', 'lay']:
        q = NPZ[mvn + '__q']; dt = float(NPZ[mvn + '__dt']); worst = []
        for i, terms in tr.act:
            if names[i] == 'gripper': continue
            s = sum(c * q[:, j] for j, c in terms); v = np.percentile(np.abs(np.diff(s)) / dt, 99)
            worst.append((names[i], round(float(v), 1), round(float(v / W[i]), 2)))
        out[mvn] = sorted(worst, key=lambda x: -x[2])[:3]
    return out

# ---------------------------------------------------------------- 2 walk inverse dynamics
def check_walk(move, mu=1.0, stride=2):
    mv, m, d, tr, names, tau0, W = setup(move)
    m.opt.disableflags |= mujoco.mjtDisableBit.mjDSBL_CONSTRAINT
    Q = []
    for f in range(len(tr.q)):
        tr.set_state(f * tr.dt); Q.append(d.qpos.copy())
    legdofs = {l: [m.joint(f'{l}_{p}').dofadr[0] for p in ['abd', 'pitch', 'knee', 'dist']] for l in LEGS}
    legact = {l: [names.index(f'{l}_{p}') for p in ['abd', 'pitch', 'knee', 'dist']] for l in LEGS}
    sp = {n: m.joint(n).dofadr[0] for n in ('spine_s1', 'spine_twist', 'spine_s2')}
    res = []; BW = m.body_subtreemass[1] * 9.81
    for f in range(1, len(Q) - 1, stride):
        v0 = np.zeros(m.nv); v1 = np.zeros(m.nv)
        mujoco.mj_differentiatePos(m, v0, tr.dt, Q[f - 1], Q[f]); mujoco.mj_differentiatePos(m, v1, tr.dt, Q[f], Q[f + 1])
        d.qpos[:] = Q[f]; d.qvel[:] = (v0 + v1) / 2; d.qacc[:] = (v1 - v0) / tr.dt
        mujoco.mj_inverse(m, d); r = d.qfrc_inverse.copy()
        st, J = [], {}
        for l in LEGS:
            g = m.geom(f'{l}_foot').id; p = d.geom_xpos[g].copy(); p[2] -= m.geom_size[g][0]
            if p[2] < 0.008:
                jp = np.zeros((3, m.nv)); mujoco.mj_jac(m, d, jp, None, p, m.geom_bodyid[g]); st.append(l); J[l] = jp
        nF = 3 * len(st); nvar = nF + 13; c = np.zeros(nvar); c[nF] = 1; c[nF + 1:] = 50.0
        Aeq = np.zeros((6, nvar)); beq = r[:6].copy()
        for k, l in enumerate(st): Aeq[:, 3 * k:3 * k + 3] = J[l][:, :6].T
        Aeq[:, nF + 1:nF + 7] = np.eye(6); Aeq[:, nF + 7:nF + 13] = -np.eye(6)
        Aub, bub = [], []
        for k, l in enumerate(st):
            for dof, ai in zip(legdofs[l], legact[l]):
                lim = tau0[ai] * max(0.05, 1 - abs(d.qvel[dof]) / W[ai])
                row = np.zeros(nvar); row[3 * k:3 * k + 3] = -J[l][:, dof]
                a1 = row.copy(); a1[nF] = -lim; Aub.append(a1); bub.append(-r[dof])
                a2 = -row; a2[nF] = -lim; Aub.append(a2); bub.append(r[dof])
            for ax in (0, 1):
                for sgn in (1, -1):
                    fr = np.zeros(nvar); fr[3 * k + ax] = sgn; fr[3 * k + 2] = -mu; Aub.append(fr); bub.append(0)
        bounds = [b for _ in st for b in ((None, None), (None, None), (0, None))] + [(0, None)] * 13
        sol = linprog(c, A_ub=np.array(Aub) if Aub else None, b_ub=np.array(bub) if bub else None, A_eq=Aeq, b_eq=beq, bounds=bounds, method='highs')
        if not sol.success: continue
        x = sol.x; F = np.zeros(m.nv)
        for k, l in enumerate(st): F += J[l].T @ x[3 * k:3 * k + 3]
        tq = r - F
        swing = max([abs(tq[dof]) / (tau0[ai] * max(0.05, 1 - abs(d.qvel[dof]) / W[ai])) for l in LEGS if l not in st for dof, ai in zip(legdofs[l], legact[l])] + [0])
        res.append(dict(u=x[nF], swing=swing, magic=np.abs(x[nF + 1:nF + 4] - x[nF + 7:nF + 10]).sum() / BW, **{n: abs(tq[dof]) for n, dof in sp.items()}))
    ok = np.array([r['magic'] < 0.15 for r in res]); p95 = lambda k: round(float(np.percentile([r[k] for r in res], 95)), 2)
    return dict(consistent_frames_pct=round(100 * ok.mean()), stance_leg_load_p95=round(float(np.percentile([r['u'] for r, o in zip(res, ok) if o], 95)), 2),
                swing_leg_load_p95=p95('swing'), spine_s1_Nm_p95=p95('spine_s1'), spine_s2_Nm_p95=p95('spine_s2'), spine_twist_Nm_p95=p95('spine_twist'), spine_limit_Nm=3.8)

# ---------------------------------------------------------------- 3 table-jump ceiling
def check_jump(offF=-0.5, offR=-0.3):
    mv, m, d, tr, names, tau0, W = setup('jump')
    f0 = int(np.argmin(NPZ['jump__base'][:, 2])); tr.set_state(f0 * tr.dt); BW = m.body_subtreemass[1] * 9.81
    qs = tr.q[f0].copy(); qe = qs.copy()
    for l in LEGS:
        for part in ['knee', 'dist']:
            n = f'{l}_{part}'; rg = m.joint(n).range; qe[JN.index(n)] = rg[0] if abs(rg[0]) < abs(rg[1]) else rg[1]
    for l in ['RR', 'RL']: qe[JN.index(l + '_pitch')] = 0.1
    for l in ['FR', 'FL']: qe[JN.index(l + '_pitch')] = 0.3
    def pose(q):
        for k, a in enumerate(tr.qadr): d.qpos[a] = q[k]
        mujoco.mj_forward(m, d)
    def fz(l):
        g = m.geom(f'{l}_foot').id; p = d.geom_xpos[g].copy(); p[2] -= 0.011
        jp = np.zeros((3, m.nv)); mujoco.mj_jac(m, d, jp, None, p, m.geom_bodyid[g]); best = 1e9
        for part in ['abd', 'pitch', 'knee', 'dist']:
            dof = m.joint(f'{l}_{part}').dofadr[0]; jz = jp[2, dof]; ps = d.qfrc_passive[dof]
            if abs(jz) > 1e-6: best = min(best, (tau0[names.index(f'{l}_{part}')] + abs(ps)) / abs(jz))
        return max(best, 0), p[0]
    def ceiling(oF, oR):
        a0, b0 = qs.copy(), qe.copy()
        for l in LEGS:
            i = JN.index(l + '_abd'); o = oF if l[0] == 'F' else oR; s = 1 if l[1] == 'R' else -1; a0[i] += s * o; b0[i] += s * o
        Wk = 0; prev = None
        for a in np.linspace(0, 1, 80):
            pose(a0 * (1 - a) + b0 * a)
            h = com(m, d)[2] - np.mean([d.geom_xpos[m.geom(f'{l}_foot').id][2] for l in LEGS]); xc = com(m, d)[0]
            R = [fz(l) for l in ['RR', 'RL']]; Fq = [fz(l) for l in ['FR', 'FL']]
            Fr = 2 * min(x[0] for x in R); Ff = 2 * min(x[0] for x in Fq)
            ratio = (xc - np.mean([x[1] for x in R])) / max(np.mean([x[1] for x in Fq]) - xc, 1e-3)
            if ratio * Fr > Ff: Fr = Ff / ratio
            F = Fr * (1 + ratio)
            if prev is not None and h > prev[0]: Wk += 0.5 * (F + prev[1]) * (h - prev[0])
            prev = (h, F)
        return round(Wk / BW * 1000)
    return dict(mass_kg=round(m.body_subtreemass[1], 3), needed_mm=750, ceiling_rig_stance_mm=ceiling(0, 0), ceiling_feet_tucked_mm=ceiling(offF, offR),
                note='upper bound: full torque the whole push, no speed loss, perfect timing')

# ---------------------------------------------------------------- 4 holding torque
def check_hold():
    out = {}
    for mv in ['stand', 'sit', 'lay']:
        mv_, m, d, tr, names, tau0, W = setup(mv); tr.set_state(0); acc = []
        for k in range(int(3 / m.opt.timestep)):
            tr.control(min(k * m.opt.timestep, tr.T), 0); mujoco.mj_step(m, d)
            if k * m.opt.timestep > 2: acc.append(np.abs(d.actuator_force.copy()))
        a = np.mean(acc, 0)
        out[mv] = [(n, round(float(v), 2)) for n, v in sorted(zip(names, a), key=lambda x: -x[1]) if 'winch' not in n][:4]
    return out

if __name__ == '__main__':
    R = {}
    print('1 SPEED (needed / no-load speed; <1 = OK)'); R['speed'] = check_speed()
    for k, v in R['speed'].items(): print(f'  {k:7s}', v)
    print('2 WALK inverse dynamics'); R['walk'] = {mv: check_walk(mv) for mv in ['walk', 'happy']}
    for k, v in R['walk'].items(): print(f'  {k:7s}', v)
    print('3 TABLE JUMP ceiling'); R['jump'] = check_jump(); print('  ', R['jump'])
    print('4 HOLD torque (N·m)'); R['hold'] = check_hold()
    for k, v in R['hold'].items(): print(f'  {k:7s}', v)
    json.dump(R, open(os.path.join(HERE, 'physics_checks.json'), 'w'), indent=1, default=float)
