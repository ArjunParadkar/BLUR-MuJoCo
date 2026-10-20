#!/usr/bin/env python3
"""Jump-spring sweep table + plot from springs/k*/spring_checks.json."""
import json, glob, os
import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
R = sorted((json.load(open(f)) for f in glob.glob('springs/k*/spring_checks.json')), key=lambda r: float(r['variant'][1:]))
rows = []
for r in R:
    k = float(r['variant'][1:]); h = r['hold']
    rows.append(dict(k=k, tucked=r['jump']['ceiling_feet_tucked_mm'], rig=r['jump']['ceiling_rig_stance_mm'],
                     stand=h['stand']['max'], sit=h['sit']['max'], lay=h['lay']['max'],
                     sit_knee=max(v for n, v in h['sit']['all'].items() if 'knee' in n), lay_knee=max(v for n, v in h['lay']['all'].items() if 'knee' in n),
                     worst=max(h[m]['max'][0] for m in ('stand', 'sit', 'lay'))))
print('| crouch torque (N·m) | tendon stiffness factor | table-jump ceiling, feet tucked (mm) | ceiling, rig stance (mm) | stand max hold | sit max hold | lay max hold | every hold < 2.5? |')
print('|---|---|---|---|---|---|---|---|')
for r in rows:
    f = lambda t: f'{t[0]:.2f} ({t[1]})'
    print(f"| {r['k']:.1f} | ×{r['k']/2:.3g} | {r['tucked']} | {r['rig']} | {f(r['stand'])} | {f(r['sit'])} | {f(r['lay'])} | {'yes' if r['worst'] < 2.5 else 'NO'} |")
json.dump(rows, open('results/springs_table.json', 'w'), indent=1)
INK, INK2, GRID, BLUE, BLUE2, RED = '#1f1f1e', '#5a5a57', '#d9d9d6', '#256abf', '#86b6ef', '#c0392b'
fig, ax = plt.subplots(figsize=(8, 4.6), facecolor='#fcfcfb')
k = [r['k'] for r in rows]
ax.plot(k, [r['tucked'] for r in rows], '-o', color=BLUE, lw=2, ms=6, label='table-jump ceiling, feet tucked (mm, upper bound)')
ax.axhline(750, color=INK2, lw=1, ls='--'); ax.text(0.02, 756, '750 mm table', fontsize=8, color=INK2)
ax.set_xlabel('jump-spring torque at the crouch (N·m per spring)', fontsize=9, color=INK); ax.set_ylabel('jump ceiling (mm)', fontsize=9, color=BLUE)
ax.set_ylim(540, 880); ax.grid(color=GRID, lw=0.6); ax.set_axisbelow(True); ax.tick_params(colors=INK2, length=0)
for s in ('top',): ax.spines[s].set_visible(False)
ax2 = ax.twinx()   # second measure on its own axis would be a dual-axis chart; instead draw holding torque as a separate panel below
ax2.remove()
fig.savefig('results/springs_ceiling.png', dpi=160, bbox_inches='tight')
fig, ax = plt.subplots(figsize=(8, 4.2), facecolor='#fcfcfb')
ax.plot(k, [r['sit_knee'] for r in rows], '-o', color=BLUE, lw=2, ms=6, label='sit: max knee holding torque')
ax.plot(k, [r['lay_knee'] for r in rows], '-s', color=BLUE2, lw=2, ms=6, label='lay: max knee holding torque')
ax.plot(k, [r['stand'][0] for r in rows], '-^', color=INK2, lw=1.5, ms=5, label='stand: max holding torque')
ax.axhline(2.5, color=RED, lw=1, ls='--'); ax.text(0.02, 2.54, '2.5 N·m holding budget', fontsize=8, color=RED)
ax.axhline(4.0, color=INK2, lw=0.8, ls=':'); ax.text(0.02, 4.04, '4.0 N·m motor limit', fontsize=8, color=INK2)
ax.set_xlabel('jump-spring torque at the crouch (N·m per spring)', fontsize=9, color=INK); ax.set_ylabel('holding torque (N·m)', fontsize=9, color=INK)
ax.set_ylim(0, 4.3); ax.grid(color=GRID, lw=0.6); ax.set_axisbelow(True); ax.tick_params(colors=INK2, length=0); ax.legend(fontsize=8, frameon=False, loc='lower right')
for s in ('top', 'right'): ax.spines[s].set_visible(False)
fig.savefig('results/springs_hold.png', dpi=160, bbox_inches='tight'); print('wrote results/springs_ceiling.png results/springs_hold.png')
