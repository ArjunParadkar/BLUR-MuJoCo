#!/usr/bin/env python3
"""
Physically consistent bounding-gait test for BLUR v36.

  python bound_gait.py --v 2.0 --f 3.5 --opt 300 --out results/bound_v2.0_f3.5.json --render results/bound_v2.0_f3.5.png

Builds a planar bound (rear pair stance -> flight -> front pair stance -> flight) at constant forward speed v and stride
frequency f, with the vertical body motion derived from half-sine stance forces (so flight is ballistic and the
stride is periodic), body height = the rig's stand height, legs placed by IK, spine S1/S2 and body pitch as sinusoids.
The free gait parameters (duty factor, pitch amplitude/phase, spine amplitudes/phases, foot offsets) are tuned by
Nelder-Mead to make the whole thing dynamically consistent: the body wrench from MuJoCo inverse dynamics must be
deliverable by the stance feet inside friction cones and motor torque/speed limits.  The foot-force LP is a verbatim
copy of check_walk() in physics_checks.py; the residual "magic" force on the body is the inconsistency measure.

Outputs: spine_s1 / spine_s2 / spine_twist torque (p95, peak) against the 3.8 N·m limit, leg load factor (stance LP
u, swing), consistency (% frames with magic < 15 % BW), and a filmstrip render.
"""
import argparse, json, os, sys, time
import numpy as np, mujoco
from scipy.optimize import linprog, minimize

HERE = os.path.dirname(os.path.abspath(__file__))
MODEL = os.path.normpath(os.path.join(HERE, '..', 'blur_mujoco_v36'))
NPZ = dict(np.load(os.path.join(MODEL, 'moves.npz'))); JN = [str(n) for n in NPZ['joint_names']]
LEGS = ['FR', 'FL', 'RR', 'RL']
V_PACK = 20.0
W0 = {'pitch': 1300, 'knee': 1300, 'dist': 1300, 'abd': 100, 'spine': 100, 'neck': 260, 'jaw': 260}
def w0(n):
    if 'winch' in n: return 260 * V_PACK / 16 * 2 * np.pi / 60 * 0.005
    for k, kv in W0.items():
        if k in n: return kv * V_PACK / 16 * 2 * np.pi / 60
    return 1e9

G = 9.81
BETA_MAX = 0.40
FOOT_Z = 0.010          # foot sphere centre height in the rig's stand (bottom of the sphere ~1 mm into the floor)


class Gait:
    def __init__(self, v, f, dt=0.005, strides=2):
        self.m = mujoco.MjModel.from_xml_path(os.path.join(MODEL, 'scene_flat.xml')); self.d = mujoco.MjData(self.m)
        m = self.m
        self.names = [m.actuator(i).name for i in range(m.nu)]
        self.tau0 = np.abs(m.actuator_forcerange[:, 1]).copy(); self.W = np.array([w0(n) for n in self.names])
        self.v, self.f, self.dt, self.strides = v, f, dt, strides
        self.T = 1.0 / f
        self.qstand = NPZ['stand__q'][0].copy(); self.bstand = NPZ['stand__base'][0].copy()
        self.qadr = {n: m.joint(n).qposadr[0] for n in JN}; self.dadr = {n: m.joint(n).dofadr[0] for n in JN}
        self.legdofs = {l: [m.joint(f'{l}_{p}').dofadr[0] for p in ['abd', 'pitch', 'knee', 'dist']] for l in LEGS}
        self.legact = {l: [self.names.index(f'{l}_{p}') for p in ['abd', 'pitch', 'knee', 'dist']] for l in LEGS}
        self.sp = {n: m.joint(n).dofadr[0] for n in ('spine_s1', 'spine_twist', 'spine_s2')}
        self.foot = {l: m.geom(f'{l}_foot').id for l in LEGS}
        self.sag = {l: [f'{l}_pitch', f'{l}_knee', f'{l}_dist'] for l in LEGS}
        self.rng = {n: m.joint(n).range.copy() for n in JN}
        self.BW = m.body_subtreemass[1] * G
        # hip (abduction body) x position relative to the chest in the stand pose, used to place the feet
        self.set_pose(self.bstand, self.qstand)
        self.hipx = {l: float(self.d.xpos[m.body(f'{l}_pod').id][0] - self.d.xpos[m.body('chest').id][0]) for l in LEGS}
        self.hipy = {l: float(self.d.xpos[m.body(f'{l}_pod').id][1]) for l in LEGS}
        self.z0 = float(self.bstand[2])

    # ---------------------------------------------------------------- kinematics helpers
    def set_pose(self, base, q):
        d = self.d; d.qpos[:7] = base
        for n, val in zip(JN, q): d.qpos[self.qadr[n]] = val
        mujoco.mj_forward(self.m, d)

    def vertical(self, tau, beta):
        """chest z(tau) and its design: half-sine stance forces, ballistic flight, period T/2."""
        T = self.T; ts = beta * T; tf = T / 2 - ts; vz = G * tf / 2
        Fpk_m = np.pi * G * T / (4 * ts)           # Fpk / m
        s = tau % (T / 2)
        if s < ts:
            z = -vz * s + Fpk_m * ts / np.pi * (s - ts / np.pi * np.sin(np.pi * s / ts)) - G * s * s / 2
        else:
            u = s - ts
            z_to = -vz * ts + Fpk_m * ts / np.pi * ts - G * ts * ts / 2
            z = z_to + vz * u - G * u * u / 2
        return z

    def build(self, P):
        """P = dict(beta, A, phi, B1, phi1, B2, phi2, offr, offf, hsw). Returns base[N,7], q[N,nJ], stance[N,4], reach_err"""
        beta = float(np.clip(P['beta'], 0.15, BETA_MAX)); T = self.T; ts = beta * T
        N = int(round(self.strides * T / self.dt)) + 2; t = np.arange(N) * self.dt
        # vertical offset so that the mean chest height over a stride equals the stand height
        zz = np.array([self.vertical(tt, beta) for tt in np.linspace(0, T, 400, endpoint=False)]); zoff = self.z0 - zz.mean()
        base = np.zeros((N, 7)); q = np.tile(self.qstand, (N, 1)); stance = np.zeros((N, 4), bool)
        # stance windows: rear pair at [kT, kT+ts], front pair at [kT+T/2, kT+T/2+ts]
        def in_stance(tt, front):
            s = (tt - (T / 2 if front else 0)) % T; return s < ts, s
        # foot world targets: during stance the foot is fixed at its touchdown point; mid-stance it is under the hip + offset
        targets = np.zeros((N, 4, 3))
        for i, tt in enumerate(t):
            th = P['A'] * np.sin(2 * np.pi * tt / T + P['phi'])
            base[i, :3] = [self.v * tt, 0.0, zoff + self.vertical(tt, beta)]
            base[i, 3:] = [np.cos(th / 2), 0, np.sin(th / 2), 0]
            q[i, JN.index('spine_s1')] = np.clip(P['B1'] * np.sin(2 * np.pi * tt / T + P['phi1']), *self.rng['spine_s1'])
            q[i, JN.index('spine_s2')] = np.clip(P['B2'] * np.sin(2 * np.pi * tt / T + P['phi2']), *self.rng['spine_s2'])
        for k, l in enumerate(LEGS):
            front = l[0] == 'F'; off = P['offf'] if front else P['offr']
            for i, tt in enumerate(t):
                st, s = in_stance(tt, front)
                stance[i, k] = st
                # touchdown time of the current / next stance
                k_td = np.floor((tt - (T / 2 if front else 0)) / T) * T + (T / 2 if front else 0)
                td_cur = k_td; td_next = k_td + T
                def td_point(td):      # foot x at touchdown so that it is under the hip (+off) at mid-stance
                    return self.v * (td + ts / 2) + self.hipx[l] + off
                if st:
                    targets[i, k] = [td_point(td_cur), self.hipy[l], FOOT_Z]
                else:
                    lo = td_cur + ts; u = (tt - lo) / max(td_next - lo, 1e-6); u = np.clip(u, 0, 1)
                    x = td_point(td_cur) + (td_point(td_next) - td_point(td_cur)) * (0.5 - 0.5 * np.cos(np.pi * u))
                    targets[i, k] = [x, self.hipy[l], FOOT_Z + P['hsw'] * np.sin(np.pi * u)]
        # IK, frame by frame, warm-started from the previous frame
        err = 0.0; qprev = None
        for i in range(N):
            if qprev is not None:
                for l in LEGS:
                    for n in self.sag[l]: q[i, JN.index(n)] = qprev[JN.index(n)]
            q[i], e = self.ik(base[i], q[i], targets[i]); err = max(err, e); qprev = q[i]
        return base, q, stance, err, targets

    def ik(self, base, q, targets, iters=12):
        m, d = self.m, self.d; q = q.copy()
        for it in range(iters):
            self.set_pose(base, q); worst = 0.0
            for k, l in enumerate(LEGS):
                p = d.geom_xpos[self.foot[l]]; e = targets[k] - p; e2 = e[[0, 2]]
                worst = max(worst, np.linalg.norm(e2))
                jp = np.zeros((3, m.nv)); mujoco.mj_jacGeom(m, d, jp, None, self.foot[l])
                dofs = [self.dadr[n] for n in self.sag[l]]; J = jp[[0, 2]][:, dofs]
                qn = np.array([q[JN.index(n)] for n in self.sag[l]]); q0 = np.array([self.qstand[JN.index(n)] for n in self.sag[l]])
                JJ = J @ J.T + 1e-4 * np.eye(2); dq = J.T @ np.linalg.solve(JJ, e2)
                Nn = np.eye(3) - J.T @ np.linalg.solve(JJ, J); dq += 0.3 * (Nn @ (q0 - qn))
                qn = qn + np.clip(dq, -0.5, 0.5)
                for n, val in zip(self.sag[l], qn): q[JN.index(n)] = float(np.clip(val, *self.rng[n]))
            if worst < 2e-4: break
        self.set_pose(base, q); worst = max(np.linalg.norm((targets[k] - d.geom_xpos[self.foot[l]])[[0, 2]]) for k, l in enumerate(LEGS))
        return q, worst

    # ---------------------------------------------------------------- inverse dynamics + the check_walk LP (verbatim logic)
    def evaluate(self, base, q, mu=1.0):
        m, d = self.m, self.d; dt = self.dt
        m.opt.disableflags |= int(mujoco.mjtDisableBit.mjDSBL_CONSTRAINT)
        Q = []
        for i in range(len(q)):
            self.set_pose(base[i], q[i]); Q.append(d.qpos.copy())
        names, tau0, W = self.names, self.tau0, self.W
        legdofs, legact, sp, BW = self.legdofs, self.legact, self.sp, self.BW
        res = []; frames = []
        T = self.T; i0 = int(round(T / dt)); i1 = int(round(2 * T / dt))      # evaluate the middle stride only
        for f in range(max(1, i0), min(len(Q) - 1, i1 + 1)):
            v0 = np.zeros(m.nv); v1 = np.zeros(m.nv)
            mujoco.mj_differentiatePos(m, v0, dt, Q[f - 1], Q[f]); mujoco.mj_differentiatePos(m, v1, dt, Q[f], Q[f + 1])
            d.qpos[:] = Q[f]; d.qvel[:] = (v0 + v1) / 2; d.qacc[:] = (v1 - v0) / dt
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
            # stance leg load without the LP's u (actual torque / speed-scaled limit), per joint
            stload = max([abs(tq[dof]) / (tau0[ai] * max(0.05, 1 - abs(d.qvel[dof]) / W[ai])) for l in st for dof, ai in zip(legdofs[l], legact[l])] + [0])
            res.append(dict(t=f * dt, n_st=len(st), u=x[nF] if st else 0.0, stload=stload, swing=swing,
                            magic=np.abs(x[nF + 1:nF + 4] - x[nF + 7:nF + 10]).sum() / BW,
                            magic_tau=np.abs(x[nF + 4:nF + 7] - x[nF + 10:nF + 13]).sum(),
                            Fz=sum(x[3 * k + 2] for k in range(len(st))) / BW,
                            **{n: abs(tq[dof]) for n, dof in sp.items()},
                            **{f'{l}_{p}': abs(tq[dof]) for l in LEGS for p, dof in zip(['abd', 'pitch', 'knee', 'dist'], legdofs[l])},
                            qvel_max=float(np.max(np.abs([d.qvel[dof] for l in LEGS for dof in legdofs[l]])))))
        m.opt.disableflags &= ~int(mujoco.mjtDisableBit.mjDSBL_CONSTRAINT)
        return res


def summarize(res, reach_err):
    a = lambda k: np.array([r[k] for r in res])
    ok = a('magic') < 0.15
    p95 = lambda k: float(np.percentile(a(k), 95)); pk = lambda k: float(np.max(a(k)))
    out = dict(frames=len(res), consistent_frames_pct=float(100 * ok.mean()), magic_mean_pct_bw=float(100 * a('magic').mean()),
               magic_peak_pct_bw=float(100 * a('magic').max()), magic_torque_mean_Nm=float(a('magic_tau').mean()),
               flight_frames_pct=float(100 * (a('n_st') == 0).mean()), reach_err_mm=float(reach_err * 1000),
               stance_leg_load_u_p95=float(np.percentile(a('u')[a('n_st') > 0], 95)) if (a('n_st') > 0).any() else None,
               stance_leg_load_u_peak=float(a('u')[a('n_st') > 0].max()) if (a('n_st') > 0).any() else None,
               swing_leg_load_p95=p95('swing'), swing_leg_load_peak=pk('swing'), Fz_peak_bw=pk('Fz'),
               leg_qvel_max_rad_s=pk('qvel_max'))
    for n in ('spine_s1', 'spine_s2', 'spine_twist'):
        out[n + '_p95_Nm'] = p95(n); out[n + '_peak_Nm'] = pk(n)
    leg = {}
    for l in LEGS:
        for p in ['abd', 'pitch', 'knee', 'dist']:
            leg[f'{l}_{p}'] = dict(p95=round(p95(f'{l}_{p}'), 3), peak=round(pk(f'{l}_{p}'), 3))
    out['leg_joint_torque_Nm'] = leg
    out['spine_limit_Nm'] = 3.8
    return out


def objective_terms(g, P):
    base, q, stance, err, _ = g.build(P)
    res = g.evaluate(base, q)
    if len(res) < 5: return 1e3, None, err
    S = summarize(res, err)
    spine = max(S['spine_s1_p95_Nm'], S['spine_s2_p95_Nm'], S['spine_twist_p95_Nm']) / 3.8
    leg = (S['stance_leg_load_u_p95'] or 0) + 0.5 * S['swing_leg_load_p95']
    J = 20 * S['magic_mean_pct_bw'] / 100 + 0.5 * S['magic_torque_mean_Nm'] + 3 * max(0, S['reach_err_mm'] - 2) / 10 + spine + 0.5 * leg
    return J, S, err


KEYS = ['beta', 'A', 'phi', 'B1', 'phi1', 'B2', 'phi2', 'offr', 'offf']
def unpack(x, hsw): return dict(zip(KEYS, x), hsw=hsw)


def optimize(v, f, n_iter, seed=0, hsw=0.04, verbose=True):
    g = Gait(v, f); rng = np.random.default_rng(seed)
    x0 = np.array([0.28, 0.08, 0.0, 0.2, 0.0, 0.2, np.pi, 0.0, 0.0])
    if seed: x0 = x0 + rng.normal(0, [0.04, 0.05, 1.0, 0.1, 1.0, 0.1, 1.0, 0.03, 0.03])
    best = [1e9, None, None]
    def fun(x):
        x = x.copy(); x[0] = np.clip(x[0], 0.15, BETA_MAX); x[1] = np.clip(x[1], -0.35, 0.35); x[3] = np.clip(x[3], -0.55, 0.55); x[5] = np.clip(x[5], -0.55, 0.55)
        x[7] = np.clip(x[7], -0.08, 0.08); x[8] = np.clip(x[8], -0.08, 0.08)
        J, S, err = objective_terms(g, unpack(x, hsw))
        if J < best[0]: best[0], best[1], best[2] = J, x.copy(), S
        return J
    t0 = time.time()
    r = minimize(fun, x0, method='Nelder-Mead', options=dict(maxfev=n_iter, xatol=1e-3, fatol=1e-3, initial_simplex=None))
    x = best[1]; P = unpack(x, hsw); S = best[2]
    if verbose: print(f'v={v} f={f} seed={seed}: J={best[0]:.3f} after {r.nfev} evals, {time.time() - t0:.0f} s', flush=True)
    return g, P, S, best[0]


def _opt_job(args):
    global BETA_MAX
    v, f, n, seed, bm = args; BETA_MAX = bm
    g, P, S, J = optimize(v, f, n, seed=seed); return P, S, J


def render(g, P, path, nframes=10):
    try:
        os.environ.setdefault('MUJOCO_GL', 'egl')
        base, q, stance, err, _ = g.build(P)
        m = g.m; ren = mujoco.Renderer(m, height=300, width=420)
        cam = mujoco.MjvCamera(); cam.type = mujoco.mjtCamera.mjCAMERA_FREE; cam.distance = 0.9; cam.elevation = -10; cam.azimuth = 90
        opt = mujoco.MjvOption(); opt.geomgroup[:] = 0; opt.geomgroup[1] = 1; opt.geomgroup[3] = 1
        T = g.T; i0 = int(round(T / g.dt)); idx = np.linspace(i0, i0 + int(round(T / g.dt)), nframes, endpoint=False).astype(int)
        imgs = []
        for i in idx:
            g.set_pose(base[i], q[i]); cam.lookat[:] = [base[i, 0], 0, 0.2]
            ren.update_scene(g.d, cam, opt); imgs.append(ren.render().copy())
        import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
        fig, axs = plt.subplots(2, nframes // 2, figsize=(2.2 * nframes / 2, 3.4))
        for ax, im, i in zip(axs.ravel(), imgs, idx):
            ax.imshow(im); ax.set_title(f't={((i - i0) * g.dt / T):.2f} T  stance {"".join(l[0] + l[1] for k, l in enumerate(LEGS) if stance[i, k]) or "flight"}', fontsize=7); ax.axis('off')
        fig.suptitle(f'BLUR v36 bound, v={g.v} m/s, f={g.f} Hz, duty {P["beta"]:.2f}', fontsize=9)
        fig.tight_layout(); fig.savefig(path, dpi=130); plt.close(fig); return True
    except Exception as e:
        print('render failed:', e); return False


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--v', type=float, default=2.0); ap.add_argument('--f', type=float, default=3.5)
    ap.add_argument('--opt', type=int, default=300); ap.add_argument('--seeds', type=int, default=3)
    ap.add_argument('--out', default=None); ap.add_argument('--render', default=None); ap.add_argument('--beta-max', type=float, default=0.40)
    a = ap.parse_args(); BETA_MAX = a.beta_max
    from concurrent.futures import ProcessPoolExecutor
    with ProcessPoolExecutor(max_workers=a.seeds) as ex:
        outs = list(ex.map(_opt_job, [(a.v, a.f, a.opt, s, a.beta_max) for s in range(a.seeds)]))
    P, S, J = min(outs, key=lambda o: o[2]); g = Gait(a.v, a.f)
    base, q, stance, err, _ = g.build(P); res = g.evaluate(base, q)
    S['trace'] = [{k: float(v) for k, v in r.items()} for r in res]
    S['seeds'] = [dict(J=o[2], params={k: float(v) for k, v in o[0].items()}) for o in outs]
    S['params'] = {k: float(v) for k, v in P.items()}; S['v_m_s'] = a.v; S['f_Hz'] = a.f; S['objective'] = J
    S['beta_max'] = BETA_MAX; S['stance_s'] = P['beta'] * g.T; S['flight_s_each'] = g.T / 2 - P['beta'] * g.T
    print(json.dumps({k: v for k, v in S.items() if k != 'leg_joint_torque_Nm'}, indent=1, default=float))
    if a.out: json.dump(S, open(a.out, 'w'), indent=1, default=float)
    if a.render: render(g, P, a.render)
