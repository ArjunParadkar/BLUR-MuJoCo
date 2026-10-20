#!/usr/bin/env python3
"""
Plots from replay_exp.py report JSONs.

  python plot_reports.py baseline ../baseline/report_full.json ../baseline/baseline.png
  python plot_reports.py variants results ../RESULTS_variants.png     # all results/*.json

baseline : heatmap move x actuator of peak |torque| / limit, two panels (motors alone / guided along the rig path).
variants : same heatmap (guided run, leg + spine actuators) for every variant side by side, + residual-peak dot chart.
"""
import json, os, sys, glob
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap, Normalize
from matplotlib.patches import Rectangle

BLUE = ['#fcfcfb', '#cde2fb', '#9ec5f4', '#6da7ec', '#3987e5', '#256abf', '#184f95', '#0d366b']
CMAP = LinearSegmentedColormap.from_list('seqblue', BLUE)
INK, INK2, GRID = '#1f1f1e', '#5a5a57', '#d9d9d6'
SAT = '#c0392b'   # outline for saturated cells (status: critical) — never used as a series colour

ORDER = ['stand', 'sit', 'lay', 'tgrab', 'walk', 'happy', 'stairs', 'couch', 'hop', 'jump', 'flip', 'pounce', 'gallop', 'sweep']
VNAMES = {'base': 'base 0.8 N·m', 'w15': '1.5 N·m', 'w25': '2.5 N·m', 'w40': '4.0 N·m +mass +arm',
          'w40_light': '4.0 N·m (light, extra)', 'w40_kp60': '4.0 N·m +mass, kp60 (extra)'}
VORDER = ['base', 'w15', 'w25', 'w40', 'w40_light', 'w40_kp60']


def load(path):
    j = json.load(open(path)); rows = j['rows'] if isinstance(j, dict) else j
    return {r['move']: r for r in rows}


def heat(ax, rows, acts, which, title, show_y=True, annotate=True):
    M = np.array([[rows[m][which]['peak'][a] for a in acts] for m in ORDER])
    S = np.array([[rows[m][which]['sat'][a] for a in acts] for m in ORDER])
    ax.imshow(np.clip(M, 0, 1), cmap=CMAP, norm=Normalize(0, 1), aspect='auto', interpolation='nearest')
    for i in range(len(ORDER)):
        for k in range(len(acts)):
            v = M[i, k]
            if v >= 0.98:
                ax.add_patch(Rectangle((k - 0.5, i - 0.5), 1, 1, fill=False, ec=SAT, lw=1.6))
                if annotate: ax.text(k, i, f'{S[i, k]:.0f}%' if S[i, k] >= 1 else '≥1', ha='center', va='center', fontsize=6.5, color='white', fontweight='bold')
            elif annotate:
                ax.text(k, i, f'{v:.2f}', ha='center', va='center', fontsize=6.5, color='white' if v > 0.6 else INK)
    ax.set_xticks(range(len(acts))); ax.set_xticklabels(acts, rotation=90, fontsize=7, color=INK)
    ax.set_yticks(range(len(ORDER))); ax.set_yticklabels(ORDER if show_y else [], fontsize=8, color=INK)
    ax.set_title(title, fontsize=9.5, color=INK, loc='left', pad=6)
    for s in ax.spines.values(): s.set_edgecolor(GRID)
    ax.tick_params(length=0)
    ax.set_xticks(np.arange(-0.5, len(acts), 1), minor=True); ax.set_yticks(np.arange(-0.5, len(ORDER), 1), minor=True)
    ax.grid(which='minor', color='white', lw=1.0); ax.tick_params(which='minor', length=0)


def baseline(src, out):
    rows = load(src); acts = list(rows['stand']['limits'].keys())
    fig, axs = plt.subplots(2, 1, figsize=(13, 12), facecolor='#fcfcfb', gridspec_kw=dict(hspace=0.55))
    heat(axs[0], rows, acts, 'free', 'A — motors alone (open-loop replay): peak |torque| / limit per actuator')
    heat(axs[1], rows, acts, 'held', 'B — body guided along the rig path: peak |torque| / limit per actuator')
    cb = fig.colorbar(plt.cm.ScalarMappable(norm=Normalize(0, 1), cmap=CMAP), ax=axs, fraction=0.025, pad=0.02)
    cb.set_label('peak |torque| / limit  (1.0 = saturated)', color=INK, fontsize=8); cb.ax.tick_params(labelsize=7, colors=INK)
    fig.suptitle('BLUR baseline — which motors hit their limit in each move   (red outline = saturated; number inside = % of samples at the limit)',
                 fontsize=11, color=INK, x=0.01, ha='left')
    lim = rows['stand']['limits']
    fig.text(0.01, 0.005, 'Limits: ' + ', '.join(f'{a} {lim[a]:g}' for a in acts if a.startswith('FR_') or a.startswith('spine') or a.startswith('neck_lift') or a == 'jaw' or a == 'gripper' or a.startswith('tail_winch_p')) + ' N·m  (FL/RR/RL legs same as FR)',
             fontsize=7, color=INK2)
    fig.savefig(out, dpi=160, bbox_inches='tight'); print('wrote', out)


def variants(resdir, out):
    R = {v: load(os.path.join(resdir, f'{v}.json')) for v in VORDER if os.path.exists(os.path.join(resdir, f'{v}.json'))}
    acts = [a for a in R['base']['stand']['limits'] if a.split('_')[0] in ('FR', 'FL', 'RR', 'RL') or a.startswith('spine')]
    n = len(R)
    fig, axs = plt.subplots(1, n, figsize=(3.2 * n + 1, 7.5), facecolor='#fcfcfb')
    for ax, (v, rows) in zip(np.atleast_1d(axs), R.items()):
        heat(ax, rows, acts, 'held', VNAMES[v], show_y=(v == 'base'), annotate=False)
    cb = fig.colorbar(plt.cm.ScalarMappable(norm=Normalize(0, 1), cmap=CMAP), ax=axs, fraction=0.015, pad=0.01)
    cb.set_label('peak |torque| / limit (guided run)', color=INK, fontsize=8); cb.ax.tick_params(labelsize=7, colors=INK)
    fig.suptitle('Wrist/hock (2204 "dist") upgrade — leg + spine actuator loading per variant   (red outline = saturated)', fontsize=11, color=INK, x=0.01, ha='left')
    fig.savefig(out, dpi=160, bbox_inches='tight'); print('wrote', out)

    # residual peak per move, one dot per variant (sequential shades = increasing upgrade; extras hollow)
    out2 = out.replace('.png', '_residual.png')
    fig, ax = plt.subplots(figsize=(11, 5.5), facecolor='#fcfcfb')
    shades = {'base': '#9ec5f4', 'w15': '#6da7ec', 'w25': '#3987e5', 'w40': '#0d366b', 'w40_light': '#0d366b', 'w40_kp60': '#0d366b'}
    mk = {'base': 'o', 'w15': 'o', 'w25': 'o', 'w40': 'o', 'w40_light': '^', 'w40_kp60': 's'}
    x = np.arange(len(ORDER)); k = 0
    for v, rows in R.items():
        y = [rows[m]['residual_peak_pct_bw'] for m in ORDER]
        ax.scatter(x + (k - (n - 1) / 2) * 0.11, y, s=34, marker=mk[v], c=shades[v] if v in ('base', 'w15', 'w25', 'w40') else 'none',
                   edgecolors=shades[v], linewidths=1.4, label=VNAMES[v], zorder=3); k += 1
    ax.axhline(35, color=INK2, lw=1, ls='--'); ax.text(len(ORDER) - 0.5, 36.5, '35 % = "PASS w/ controller" threshold', ha='right', fontsize=7.5, color=INK2)
    ax.set_xticks(x); ax.set_xticklabels(ORDER, fontsize=8.5, color=INK); ax.set_ylabel('residual peak (% body weight, guided run)', fontsize=8.5, color=INK)
    ax.set_yscale('log'); ax.set_ylim(1.5, 1200); ax.grid(axis='y', color=GRID, lw=0.7); ax.set_axisbelow(True)
    for s in ('top', 'right'): ax.spines[s].set_visible(False)
    for s in ('left', 'bottom'): ax.spines[s].set_edgecolor(GRID)
    ax.tick_params(colors=INK2, length=0); ax.legend(fontsize=7.5, frameon=False, ncol=3, loc='upper left')
    ax.set_title('Residual help the body still needs, per move and variant (lower is better, log scale)', fontsize=10.5, color=INK, loc='left')
    fig.savefig(out2, dpi=160, bbox_inches='tight'); print('wrote', out2)


if __name__ == '__main__':
    if sys.argv[1] == 'baseline': baseline(sys.argv[2], sys.argv[3])
    else: variants(sys.argv[2], sys.argv[3])
