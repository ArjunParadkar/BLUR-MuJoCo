#!/usr/bin/env python3
"""Jump-spring variants: scale every jumpspring_* tendon stiffness so the crouch torque is T N·m (shipped = 2.0 N·m).
Each variant dir is a full runnable copy (edited blur.xml, scene copies, symlinked meshes/moves/meta, copied scripts)."""
import os, re, shutil, sys
HERE = os.path.dirname(os.path.abspath(__file__)); ORIG = os.path.normpath(os.path.join(HERE, '..', 'blur_mujoco_v36'))
RATES = [0.0, 0.5, 1.0, 1.5, 1.7, 1.8, 1.9, 2.0, 2.5, 3.0]
src = open(os.path.join(ORIG, 'blur.xml')).read()
for T in RATES:
    d = os.path.join(HERE, 'springs', f'k{T:.1f}'); os.makedirs(d, exist_ok=True)
    f = T / 2.0
    xml, n = re.subn(r'(<fixed name="jumpspring_[A-Z]{2}_[a-z]+" stiffness=")([\d.]+)(")', lambda mo: f'{mo.group(1)}{float(mo.group(2)) * f:.4f}{mo.group(3)}', src)
    assert n == 6, n
    open(os.path.join(d, 'blur.xml'), 'w').write(xml)
    for fn in os.listdir(ORIG):
        if fn.startswith('scene_') or fn in ('replay.py', 'physics_checks.py'): shutil.copy(os.path.join(ORIG, fn), d)
    for fn in ('meshes', 'moves.npz', 'blur_meta.json'):
        p = os.path.join(d, fn)
        if not os.path.lexists(p): os.symlink(os.path.relpath(os.path.join(ORIG, fn), d), p)
    print('wrote', d, 'factor', f)
