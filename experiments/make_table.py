#!/usr/bin/env python3
"""Markdown tables from results/*.json:  python make_table.py results > ../tables.md"""
import json, os, sys
from plot_reports import ORDER, VORDER, VNAMES, load

resdir = sys.argv[1]
R = {v: load(os.path.join(resdir, f'{v}.json')) for v in VORDER if os.path.exists(os.path.join(resdir, f'{v}.json'))}
V = list(R)
short = {'PASS': 'PASS', 'PASS w/ controller': 'PASS/ctrl', 'NEEDS WORK': 'FAIL'}


def top(r, n=3):
    # top saturated motors in the guided run (peak >= 0.98), with % of samples at the limit
    s = sorted(((a, r['held']['peak'][a], r['held']['sat'][a]) for a in r['held']['peak']), key=lambda x: (-x[1], -x[2]))
    sat = [f'{a} ({p:.0f}%)' if p >= 0.5 else a for a, pk, p in s if pk >= 0.98][:n]
    return ', '.join(sat) if sat else f'none (max {s[0][0]} {s[0][1]:.2f}×)'


print('### Verdict')
print('| move | ' + ' | '.join(VNAMES[v] for v in V) + ' |'); print('|---|' + '---|' * len(V))
for m in ORDER:
    print(f'| {m} | ' + ' | '.join(short[R[v][m]['verdict']] for v in V) + ' |')
print('\n### Residual peak (% body weight, guided run; <35 = PASS w/ controller)')
print('| move | ' + ' | '.join(VNAMES[v] for v in V) + ' |'); print('|---|' + '---|' * len(V))
for m in ORDER:
    print(f'| {m} | ' + ' | '.join(f"{R[v][m]['residual_peak_pct_bw']:.0f}" for v in V) + ' |')
print('\n### Open-loop (motors alone): path error mm / fell')
print('| move | ' + ' | '.join(VNAMES[v] for v in V) + ' |'); print('|---|' + '---|' * len(V))
for m in ORDER:
    print(f'| {m} | ' + ' | '.join(f"{R[v][m]['open_loop_path_err_mm']} {'fell' if R[v][m]['open_loop_fell'] else ''}".strip() for v in V) + ' |')
print('\n### Saturated motors in the guided run (peak ≥ 0.98 × limit; % = share of samples at the limit, shown when ≥ 0.5 %)')
print('| move | ' + ' | '.join(VNAMES[v] for v in V) + ' |'); print('|---|' + '---|' * len(V))
for m in ORDER:
    print(f'| {m} | ' + ' | '.join(top(R[v][m]) for v in V) + ' |')
print('\n### Count of saturated actuators per move (guided run)')
print('| move | ' + ' | '.join(VNAMES[v] for v in V) + ' |'); print('|---|' + '---|' * len(V))
for m in ORDER:
    print(f'| {m} | ' + ' | '.join(str(sum(1 for a in R[v][m]['held']['peak'] if R[v][m]['held']['peak'][a] >= 0.98)) for v in V) + ' |')
if any(R[v][m].get('nan') for v in V for m in ORDER):
    print('\nNaN runs: ' + ', '.join(f'{v}/{m}' for v in V for m in ORDER if R[v][m].get('nan')))
else:
    print('\nNo run went NaN.')
