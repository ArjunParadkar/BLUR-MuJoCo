#!/usr/bin/env python3
"""Tables + plots from results/bound/v*_f*.json."""
import json, glob, os, re
import numpy as np
import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
INK, INK2, GRID, RED = '#1f1f1e', '#5a5a57', '#d9d9d6', '#c0392b'
SH = {3.0: '#86b6ef', 3.5: '#3987e5', 4.0: '#0d366b'}
R = {}
for f in glob.glob('results/bound/v*_f*.json'):
    mo = re.search(r'v([\d.]+)_f([\d.]+)\.json', f); R[(float(mo.group(1)), float(mo.group(2)))] = json.load(open(f))
VS = sorted({k[0] for k in R}); FS = sorted({k[1] for k in R})
print('### Bounding gait: spine torque vs 3.8 N·m limit and leg load, per speed and stride frequency')
print('| v (m/s) | f (Hz) | duty | flight each (ms) | consistent frames % | magic mean % BW | S1 p95 / peak | S2 p95 / peak | twist p95 / peak | stance leg load u p95 / peak | swing leg load p95 / peak | peak Fz (BW) | spine OK? | legs OK? |')
print('|---|---|---|---|---|---|---|---|---|---|---|---|---|---|')
for v in VS:
    for f in FS:
        if (v, f) not in R: continue
        S = R[(v, f)]
        sp_ok = max(S['spine_s1_p95_Nm'], S['spine_s2_p95_Nm'], S['spine_twist_p95_Nm']) <= 3.8
        leg_ok = (S['stance_leg_load_u_p95'] or 9) <= 1.0 and S['swing_leg_load_p95'] <= 1.0
        print(f"| {v} | {f} | {S['params']['beta']:.2f} | {1000*S['flight_s_each']:.0f} | {S['consistent_frames_pct']:.0f} | {S['magic_mean_pct_bw']:.1f} | "
              f"{S['spine_s1_p95_Nm']:.2f} / {S['spine_s1_peak_Nm']:.2f} | {S['spine_s2_p95_Nm']:.2f} / {S['spine_s2_peak_Nm']:.2f} | {S['spine_twist_p95_Nm']:.2f} / {S['spine_twist_peak_Nm']:.2f} | "
              f"{S['stance_leg_load_u_p95']:.2f} / {S['stance_leg_load_u_peak']:.2f} | {S['swing_leg_load_p95']:.2f} / {S['swing_leg_load_peak']:.2f} | {S['Fz_peak_bw']:.2f} | {'yes' if sp_ok else 'NO'} | {'yes' if leg_ok else 'NO'} |")
print()
print('### Leg joint torque (N·m, p95 / peak) at 3.5 Hz; limits: abd 3.8, pitch/knee/dist 4.0')
print('| v | ' + ' | '.join(f'{l}_{p}' for l in ['FR', 'RR'] for p in ['abd', 'pitch', 'knee', 'dist']) + ' |'); print('|---|' + '---|' * 8)
for v in VS:
    if (v, 3.5) not in R: continue
    L = R[(v, 3.5)]['leg_joint_torque_Nm']
    print(f'| {v} | ' + ' | '.join(f"{L[f'{l}_{p}']['p95']:.2f} / {L[f'{l}_{p}']['peak']:.2f}" for l in ['FR', 'RR'] for p in ['abd', 'pitch', 'knee', 'dist']) + ' |')
# ---- plot 1: spine torque p95 and peak vs speed, one line per frequency
fig, axs = plt.subplots(1, 3, figsize=(12, 4), facecolor='#fcfcfb', sharey=True)
for ax, key, title in zip(axs, ['spine_s1', 'spine_s2', 'spine_twist'], ['spine S1 (front bend)', 'spine S2 (rear bend)', 'spine twist']):
    for f in FS:
        xs = [v for v in VS if (v, f) in R]
        ax.plot(xs, [R[(v, f)][key + '_p95_Nm'] for v in xs], '-o', color=SH[f], lw=2, ms=6, label=f'{f} Hz p95')
        ax.plot(xs, [R[(v, f)][key + '_peak_Nm'] for v in xs], ':', color=SH[f], lw=1.2, marker='_', ms=9, label=f'{f} Hz peak')
    ax.axhline(3.8, color=RED, lw=1, ls='--'); ax.text(min(VS), 3.9, '3.8 N·m 4015 limit', fontsize=8, color=RED)
    ax.set_title(title, fontsize=10, color=INK, loc='left'); ax.set_xlabel('forward speed (m/s)', fontsize=9, color=INK); ax.grid(color=GRID, lw=0.6); ax.set_axisbelow(True)
    ax.tick_params(colors=INK2, length=0); ax.set_xticks(VS)
    for s in ('top', 'right'): ax.spines[s].set_visible(False)
axs[0].set_ylabel('torque (N·m)', fontsize=9, color=INK); axs[0].legend(fontsize=7.5, frameon=False, ncol=2)
fig.suptitle('Bounding gait: spine motor torque needed vs speed (feet forces from the check_walk LP)', fontsize=11, color=INK, x=0.01, ha='left')
fig.savefig('results/bound_spine_vs_speed.png', dpi=160, bbox_inches='tight')
# ---- plot 2: leg load vs speed
fig, ax = plt.subplots(figsize=(7, 4), facecolor='#fcfcfb')
for f in FS:
    xs = [v for v in VS if (v, f) in R]
    ax.plot(xs, [R[(v, f)]['stance_leg_load_u_p95'] for v in xs], '-o', color=SH[f], lw=2, ms=6, label=f'{f} Hz stance (LP u, p95)')
    ax.plot(xs, [R[(v, f)]['swing_leg_load_p95'] for v in xs], '--s', color=SH[f], lw=1.3, ms=5, label=f'{f} Hz swing (p95)')
ax.axhline(1.0, color=RED, lw=1, ls='--'); ax.text(min(VS), 1.02, '1.0 = at the speed-scaled torque limit', fontsize=8, color=RED)
ax.set_xlabel('forward speed (m/s)', fontsize=9, color=INK); ax.set_ylabel('leg motor load factor', fontsize=9, color=INK); ax.set_xticks(VS)
ax.grid(color=GRID, lw=0.6); ax.set_axisbelow(True); ax.tick_params(colors=INK2, length=0); ax.legend(fontsize=7.5, frameon=False, ncol=2)
for s in ('top', 'right'): ax.spines[s].set_visible(False)
ax.set_title('Leg motor load in the bound (torque / speed-derated limit)', fontsize=10, color=INK, loc='left')
fig.savefig('results/bound_leg_vs_speed.png', dpi=160, bbox_inches='tight')
# ---- plot 3: per-frame traces at 3.5 Hz
fig, axs = plt.subplots(len(VS), 1, figsize=(9, 2.6 * len(VS)), facecolor='#fcfcfb', sharex=True)
for ax, v in zip(np.atleast_1d(axs), VS):
    if (v, 3.5) not in R: continue
    S = R[(v, 3.5)]; tr = S['trace']; T = 1 / 3.5; t = (np.array([r['t'] for r in tr]) - T) / T
    for key, col, lab in [('spine_s1', '#0d366b', 'S1'), ('spine_s2', '#3987e5', 'S2'), ('spine_twist', '#86b6ef', 'twist')]:
        ax.plot(t, [r[key] for r in tr], color=col, lw=1.8, label=lab)
    st = np.array([r['n_st'] for r in tr])
    ax.fill_between(t, 0, 14, where=st == 0, color=GRID, alpha=0.5, step='mid', label='flight')
    ax.axhline(3.8, color=RED, lw=1, ls='--'); ax.set_ylim(0, 14); ax.set_ylabel(f'v = {v} m/s\nN·m', fontsize=9, color=INK)
    ax.grid(color=GRID, lw=0.5); ax.set_axisbelow(True); ax.tick_params(colors=INK2, length=0)
    for s in ('top', 'right'): ax.spines[s].set_visible(False)
np.atleast_1d(axs)[0].legend(fontsize=8, frameon=False, ncol=4); np.atleast_1d(axs)[-1].set_xlabel('stride phase (rear stance starts at 0, front stance at 0.5)', fontsize=9, color=INK)
fig.suptitle('Spine torque through one stride at 3.5 Hz (grey = flight)', fontsize=11, color=INK, x=0.01, ha='left')
fig.savefig('results/bound_spine_trace.png', dpi=160, bbox_inches='tight'); print('wrote plots')
