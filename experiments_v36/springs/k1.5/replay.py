#!/usr/bin/env python3
"""
BLUR — MuJoCo replay + physics check of every move from the kinematic rig.

  pip install mujoco numpy
  python replay.py                         # opens the viewer, physics replay of 'stand'
  python replay.py --move gallop           # physics replay of a move (viewer)
  python replay.py --move backflip --kinematic   # play the rig's motion exactly (no physics), to compare
  python replay.py --report                # headless: run every move, print the pass/fail table (report.json)

Physics replay = every actuator is a position servo with its REAL torque limit tracking the rig's joint angles.
A "guide" spring can hold the body on the rig's path; the force it has to add is the RESIDUAL: the help the move
needs beyond what the motors, springs and ground can provide. ~0 means the real hardware can do it as animated.
"""
import argparse, json, os, sys, time
import numpy as np
import mujoco

HERE = os.path.dirname(os.path.abspath(__file__))
META = json.load(open(os.path.join(HERE, 'blur_meta.json')))
NPZ = np.load(os.path.join(HERE, 'moves.npz'))
JN = list(NPZ['joint_names'])
ALIAS = {'backflip': 'flip', 'table': 'jump', 'tailgrab': 'tgrab', 'couchclimb': 'couch', 'jumpup': 'hop'}
LOOP = {'walk': 3, 'gallop': 6}


def load(move):
    mv = ALIAS.get(move, move)
    scene = META['scene_of'].get(mv, 'flat')
    m = mujoco.MjModel.from_xml_path(os.path.join(HERE, f'scene_{scene}.xml'))
    d = mujoco.MjData(m)
    base, q, dt = NPZ[f'{mv}__base'].copy(), NPZ[f'{mv}__q'].copy(), float(NPZ[f'{mv}__dt'])
    if mv in LOOP:                       # periodic gaits: repeat the cycle, carrying the forward travel
        reps = LOOP[mv]; step = base[-1, :3] - base[0, :3]
        B = [base[:-1] + np.r_[step * k, np.zeros(4)] for k in range(reps)]
        base = np.vstack(B + [base[-1:] + np.r_[step * (reps - 1), np.zeros(4)]])
        q = np.vstack([q[:-1]] * reps + [q[-1:]])
    return mv, m, d, base, q, dt


class Track:
    def __init__(self, m, d, base, q, dt):
        self.m, self.d, self.base, self.q, self.dt = m, d, base, q, dt
        self.T = (len(q) - 1) * dt
        self.jid = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n) for n in JN]
        self.qadr = [m.jnt_qposadr[j] for j in self.jid]
        self.dadr = [m.jnt_dofadr[j] for j in self.jid]
        self.chest = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, 'chest')
        # actuator -> how to compute its target from the joint targets
        self.act = []
        for a in META['actuators']:
            i = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, a['name'])
            if a['tendon']:
                t = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_TENDON, a['target'])
                adr, num = m.tendon_adr[t], m.tendon_num[t]
                terms = [(JN.index(mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, m.wrap_objid[k])), m.wrap_prm[k]) for k in range(adr, adr + num)]
                self.act.append((i, terms))
            else:
                self.act.append((i, [(JN.index(a['target']), 1.0)]))

    def ref(self, t):
        t = min(max(t, 0), self.T); x = t / self.dt; i = min(int(x), len(self.q) - 2); u = x - i
        q = self.q[i] * (1 - u) + self.q[i + 1] * u
        b = self.base[i] * (1 - u) + self.base[i + 1] * u; b[3:] /= np.linalg.norm(b[3:])
        dq = (self.q[i + 1] - self.q[i]) / self.dt
        dp = (self.base[i + 1, :3] - self.base[i, :3]) / self.dt
        w = np.zeros(3); mujoco.mju_subQuat(w, self.base[i + 1, 3:], self.base[i, 3:]); w /= self.dt
        return q, dq, b, dp, w

    def set_state(self, t):
        q, dq, b, dp, w = self.ref(t)
        d = self.d; d.qpos[:7] = b
        for k, a in enumerate(self.qadr): d.qpos[a] = q[k]
        d.qvel[:] = 0; d.qvel[:3] = dp; d.qvel[3:6] = w   # world-frame lin vel; body ang vel approx
        for k, a in enumerate(self.dadr): d.qvel[a] = dq[k]
        mujoco.mj_forward(self.m, self.d)

    def control(self, t, guide=0.0):
        q, dq, b, dp, w = self.ref(t)
        d = self.d
        for i, terms in self.act:
            d.ctrl[i] = sum(c * q[j] for j, c in terms)
        d.xfrc_applied[:] = 0
        if guide > 0:                          # soft guide on the chest toward the rig's path (the residual)
            m = self.m; M = m.body_subtreemass[self.chest]
            kp, kd = guide * M * 400.0, guide * M * 40.0
            err = b[:3] - d.qpos[:3]; verr = dp - d.qvel[:3]
            F = kp * err + kd * verr
            qe = np.zeros(3); mujoco.mju_subQuat(qe, b[3:], d.qpos[3:7])
            R = d.xmat[self.chest].reshape(3, 3)
            I = M * 0.02 ** 2
            Tq = guide * I * 2000 * (R @ qe) - guide * I * 90 * (R @ d.qvel[3:6] - w)
            d.xfrc_applied[self.chest, :3] = F
            d.xfrc_applied[self.chest, 3:] = Tq
        return q


def run(move, guide=1.0, record=False):
    mv, m, d, base, q, dt = load(move)
    tr = Track(m, d, base, q, dt)
    tr.set_state(0.0)
    W = m.body_subtreemass[tr.chest] * 9.81
    lim = np.array([abs(m.actuator_forcerange[i, 1]) for i, _ in tr.act])
    sat, peak, resid, perr, n = np.zeros(len(lim)), np.zeros(len(lim)), [], [], 0
    t = 0.0; steps = int(tr.T / m.opt.timestep)
    fell = False
    for k in range(steps):
        tr.control(t, guide)
        mujoco.mj_step(m, d); t += m.opt.timestep
        if k % 10 == 0:
            f = np.array([abs(d.actuator_force[i]) for i, _ in tr.act])
            sat += f >= lim * 0.98; peak = np.maximum(peak, f / lim); n += 1
            resid.append(np.linalg.norm(d.xfrc_applied[tr.chest, :3]) / W)
            ref = tr.ref(t)[2]; perr.append(np.linalg.norm(ref[:3] - d.qpos[:3]))
            up = d.xmat[tr.chest].reshape(3, 3)[:, 2]
            if d.qpos[2] < 0.06 and mv not in ('lay',):
                fell = True
    names = [META['actuators'][k]['name'] for k in range(len(lim))]
    worst = sorted(zip(names, peak, sat / max(n, 1)), key=lambda x: -x[1])[:4]
    return dict(move=mv, seconds=round(tr.T, 2), residual_mean=float(np.mean(resid)), residual_peak=float(np.percentile(resid, 98)),
                path_err_mm=float(np.max(perr) * 1000), fell=fell,
                saturated=[(a, round(float(p), 2), round(float(s) * 100, 1)) for a, p, s in worst])


def report():
    rows = []
    order = ['stand', 'sit', 'lay', 'walk', 'happy', 'gallop', 'sweep', 'hop', 'jump', 'flip', 'stairs', 'couch', 'pounce', 'tgrab']
    for mv in order:
        free = run(mv, guide=0.0)        # motors alone
        held = run(mv, guide=1.0)        # residual needed to follow the animation
        verdict = ('PASS' if (not free['fell'] and free['path_err_mm'] < 60) else
                   'PASS w/ controller' if held['residual_peak'] < 0.35 else 'NEEDS WORK')
        rows.append(dict(move=mv, verdict=verdict, open_loop_path_err_mm=round(free['path_err_mm']), open_loop_fell=free['fell'],
                         residual_mean_pct_bw=round(held['residual_mean'] * 100, 1), residual_peak_pct_bw=round(held['residual_peak'] * 100, 1),
                         hardest_actuators=held['saturated']))
        r = rows[-1]
        print(f"{mv:8s} {verdict:18s} open-loop err {r['open_loop_path_err_mm']:5d} mm  fell={str(r['open_loop_fell']):5s} "
              f"residual mean {r['residual_mean_pct_bw']:5.1f}% / peak {r['residual_peak_pct_bw']:5.1f}% of body weight   "
              + ', '.join(f'{a} {p:.2f}x' for a, p, s in r['hardest_actuators'][:3]), flush=True)
    json.dump(rows, open(os.path.join(HERE, 'report.json'), 'w'), indent=1)
    return rows


def view(move, kinematic=False, guide=0.0):
    import mujoco.viewer
    mv, m, d, base, q, dt = load(move)
    tr = Track(m, d, base, q, dt); tr.set_state(0.0)
    with mujoco.viewer.launch_passive(m, d) as v:
        t0 = time.time(); t = 0.0
        while v.is_running():
            if kinematic:
                t = (time.time() - t0) % (tr.T + 0.5); tr.set_state(t)
            else:
                tr.control(t, guide); mujoco.mj_step(m, d); t += m.opt.timestep
                if t > tr.T + 1.0:
                    tr.set_state(0.0); t = 0.0
                while time.time() - t0 < t: time.sleep(0.0005)
                if t == 0.0: t0 = time.time()
            v.sync()


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--move', default='stand', help='stand sit lay a b c d walk gallop happy sweep hop jump flip stairs couch pounce tgrab')
    ap.add_argument('--kinematic', action='store_true', help='play the rig motion exactly (no physics)')
    ap.add_argument('--guide', type=float, default=0.0, help='0 = motors alone, 1 = body guided along the rig path')
    ap.add_argument('--report', action='store_true')
    a = ap.parse_args()
    if a.report: report()
    else: view(a.move, a.kinematic, a.guide)
