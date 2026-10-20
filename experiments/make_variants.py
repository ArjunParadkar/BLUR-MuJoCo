#!/usr/bin/env python3
"""
Build wrist/hock (2204 "dist") motor-upgrade variants of blur.xml under experiments/variants/<name>/.

Each variant directory gets: blur.xml (edited copy), scene_*.xml (verbatim copies, they include blur.xml by
relative path), meshes -> symlink to the original meshes.  The original blur_mujoco/ is never touched.

Variants:
  base        0.8 N·m   unchanged copy (control: must reproduce report.json)
  w15         1.5 N·m   torque limit only
  w25         2.5 N·m   torque limit only
  w40         4.0 N·m   + motor mass 0.048 -> 0.099 kg (+0.051 kg at each dist motor) + armature 1e-3   (= 2806.5 16:1 module swap)
  w40_light   4.0 N·m   torque limit only (extra: isolates the cost of the added mass)
  w40_kp60    like w40, plus the servo gains raised to the hip values (kp 60, kd 1.5)  (extra: is the 2204's kp=15 a limit?)
"""
import os, re, json, shutil
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ORIG = os.path.normpath(os.path.join(HERE, '..', 'blur_mujoco'))
OUT = os.path.join(HERE, 'variants')
DIST = ['FR_dist', 'FL_dist', 'RR_dist', 'RL_dist']
M2204, M2806 = 0.048, 0.099           # module masses (motor + 16:1) from the parts list
R2806, H2806 = 0.01675, 2 * 0.0101    # 2806.5 module housing (the c_*-knee_mm cylinder): radius, full height


def cyl_inertia(m, r, h):
    """solid cylinder, axis = local y (the hinge axis of every leg joint here)."""
    ia = 0.5 * m * r * r; ip = m * (3 * r * r + h * h) / 12.0
    return np.diag([ip, ia, ip])


def add_point_mass(mass, com, I_full, dm, p, I_dm):
    """Combine a body (mass, com, inertia about com) with an added part (dm at p, inertia I_dm about p)."""
    I = np.array([[I_full[0], I_full[3], I_full[4]], [I_full[3], I_full[1], I_full[5]], [I_full[4], I_full[5], I_full[2]]])
    M = mass + dm; c = (mass * com + dm * p) / M
    def shift(Ic, m, d):            # parallel axis: inertia about com -> about point displaced by d
        return Ic + m * (np.dot(d, d) * np.eye(3) - np.outer(d, d))
    In = shift(I, mass, com - c) + shift(I_dm, dm, p - c)
    return M, c, [In[0, 0], In[1, 1], In[2, 2], In[0, 1], In[0, 2], In[1, 2]]


def edit(xml, name, limit, heavy, armature, kp=None, kd=None):
    for j in DIST:
        leg = j.split('_')[0]
        # actuator line
        pat = re.compile(rf'(<general name="{j}" joint="{j}" gaintype="fixed" gainprm=")([\d.]+)(" biastype="affine" biasprm="0 -)([\d.]+) -([\d.]+)(" forcelimited="true" forcerange="-)([\d.]+) ([\d.]+)(")')
        def rep(mo):
            g = kp if kp is not None else float(mo.group(2)); d_ = kd if kd is not None else float(mo.group(5))
            return f'{mo.group(1)}{g:g}{mo.group(3)}{g:g} -{d_:g}{mo.group(6)}{limit} {limit}{mo.group(9)}'
        xml, n = pat.subn(rep, xml); assert n == 1, (j, 'actuator', n)
        # joint armature
        if armature is not None:
            pat = re.compile(rf'(<joint name="{j}" type="hinge" axis="0 1 0" range="[^"]+" armature=")([\d.e-]+)(")')
            xml, n = pat.subn(lambda mo: f'{mo.group(1)}{armature}{mo.group(3)}', xml); assert n == 1, (j, 'armature', n)
        # body mass: the dist motor housing geom c_<leg>-dist_mm sits in body <leg>_disJ; add the mass delta there
        if heavy:
            body = f'{leg}_disJ'
            pat = re.compile(rf'(<body name="{body}"[^>]*>\s*<joint name="{j}"[^>]*/>\s*<inertial pos=")([^"]+)(" mass=")([\d.]+)(" fullinertia=")([^"]+)(")')
            geom = re.search(rf'<geom class="col" name="c_{leg}-dist_mm" type="cylinder" pos="([^"]+)"', xml)
            p = np.array([float(v) for v in geom.group(1).split()])
            def rep2(mo):
                com = np.array([float(v) for v in mo.group(2).split()]); mass = float(mo.group(4))
                I = [float(v) for v in mo.group(6).split()]
                dm = M2806 - M2204
                M, c, In = add_point_mass(mass, com, I, dm, p, cyl_inertia(dm, R2806, H2806))
                return (f'{mo.group(1)}{c[0]:.6g} {c[1]:.6g} {c[2]:.6g}{mo.group(3)}{M:.5f}{mo.group(5)}'
                        + ' '.join(f'{v:.6g}' for v in In) + mo.group(7))
            xml, n = pat.subn(rep2, xml); assert n == 1, (j, 'inertial', n)
    return xml


VARIANTS = {
    'base':      dict(limit=0.8, heavy=False, armature=None),
    'w15':       dict(limit=1.5, heavy=False, armature=None),
    'w25':       dict(limit=2.5, heavy=False, armature=None),
    'w40':       dict(limit=4.0, heavy=True,  armature=1e-3),
    'w40_light': dict(limit=4.0, heavy=False, armature=None),
    'w40_kp60':  dict(limit=4.0, heavy=True,  armature=1e-3, kp=60, kd=1.5),
}

if __name__ == '__main__':
    src = open(os.path.join(ORIG, 'blur.xml')).read()
    scenes = [f for f in os.listdir(ORIG) if f.startswith('scene_') and f.endswith('.xml')]
    for name, kw in VARIANTS.items():
        d = os.path.join(OUT, name); os.makedirs(d, exist_ok=True)
        open(os.path.join(d, 'blur.xml'), 'w').write(edit(src, name, **kw))
        for s in scenes: shutil.copy(os.path.join(ORIG, s), os.path.join(d, s))
        link = os.path.join(d, 'meshes')
        if not os.path.lexists(link): os.symlink(os.path.relpath(os.path.join(ORIG, 'meshes'), d), link)
        json.dump(kw, open(os.path.join(d, 'variant.json'), 'w'), indent=1)
        print('wrote', d, kw)
