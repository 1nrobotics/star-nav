"""Measures how much the Microduck shakes: constant commands and the recorded mission."""
import dataclasses, json, math, sys, os
os.environ.setdefault('MUJOCO_GL', 'egl')
import numpy as np, mujoco
from star_nav_mujoco import microduck, quadrotor, sim, world

def make(obstacles=None):
    cfg = sim.SimConfig(world=dataclasses.replace(world.WorldConfig(size_x=12, size_y=12, seed=1, pillar_num=60, ring_num=0), **world.MICRODUCK_FOREST),
                        start=(0, 0, microduck.STAND_HEIGHT), robot=sim.MICRODUCK)
    return sim.Simulator(cfg, obstacles=obstacles if obstacles is not None else [])

class Metrics:
    def __init__(s, m, d):
        s.m, s.d = m, d
        s.trunk = m.body(microduck.TRUNK).id
        s.cam = m.camera(microduck.CAMERA).id
        s.w, s.cw, s.z, s.da, s.tw, s.v = [], [], [], [], [], []
    def sample(s, walker, twist):
        d = s.d
        w = d.cvel[s.trunk][:3]  # world-frame angular vel of trunk (rot part)
        s.w.append(w.copy())
        s.z.append(d.xpos[s.trunk][2])
        s.da.append(np.abs(walker._last).copy())
        s.tw.append(np.array(twist, float))
        s.v.append(d.qvel[:2].copy())
    def report(s, last_actions):
        w = np.array(s.w); tw = np.array(s.tw)
        a = np.array(last_actions); da = np.abs(np.diff(a, axis=0)).mean()
        switches = (np.abs(np.diff(tw, axis=0)).max(1) > 0.05).sum() / (len(tw) / 50)
        return dict(roll_pitch_rate_rms=float(np.sqrt((w[:, :2] ** 2).mean())),
                    yaw_rate_std=float(w[:, 2].std()),
                    z_std_mm=float(1000 * np.std(s.z)),
                    action_rate=float(da), cmd_switch_per_s=float(switches),
                    speed=float(np.linalg.norm(np.array(s.v), axis=1).mean()))

def run(s, seconds, twist_fn):
    m, d, w = s.model, s.data, s.walker
    met = Metrics(m, d); acts = []
    for _ in range(int(seconds / m.opt.timestep)):
        if w.policy_due:
            twist = twist_fn(d)
            w.twist = np.asarray(twist, np.float32)
        w.before_step()
        mujoco.mj_step(m, d)
        if w._steps % microduck.DECIMATION == 1:
            met.sample(w, w.twist); acts.append(w._last.copy())
    return met.report(acts[100:]) | dict(fallen=bool(d.qpos[2] < 0.6 * microduck.STAND_HEIGHT))

import sys as _s, os; _s.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
VARIANTS = {}
def variant(s, name):
    if name == 'current': return s
    from faithful import Faithful
    kw = dict(faithful=dict(), faithful_lag4=dict(lag=4), enc_only=dict(mjlab_friction=True), noenc=dict(encoder=False))[name]
    s.walker = Faithful(s.model, s.data, **kw); s.reset(); return s

def battery(name):
    out = {}
    for vx in (0.2, 0.3, 0.4):
        s = variant(make(), name); s.step(int(1 / s.model.opt.timestep))
        out[f'{name} vx={vx}'] = run(s, 10, lambda d, vx=vx: (vx, 0, 0))
    for wz in (0.5, 1.0):
        s = variant(make(), name); s.step(int(1 / s.model.opt.timestep))
        out[f'{name} wz={wz}'] = run(s, 6, lambda d, wz=wz: (0, 0, wz))
    return out

if __name__ == '__main__' and len(_s.argv) > 1:
    for name in _s.argv[1:]:
        for k, v in battery(name).items():
            print(f'{k:22s} ' + ' '.join(f'{a}={b:.3f}' if isinstance(b, float) else f'{a}={b}' for a, b in v.items()), flush=True)
    _s.exit()

if __name__ == '__main__':
    out = {}
    for vx in (0.0, 0.2, 0.3, 0.4, 0.5):
        s = make(); s.step(int(1 / s.model.opt.timestep))
        out[f'const vx={vx}'] = run(s, 10, lambda d, vx=vx: (vx, 0, 0))
    for wz in (0.5, 1.0, 1.5):
        s = make(); s.step(int(1 / s.model.opt.timestep))
        out[f'const wz={wz}'] = run(s, 6, lambda d, wz=wz: (0, 0, wz))
    for k, v in out.items():
        print(f'{k:16s} ' + ' '.join(f'{a}={b:.3f}' if isinstance(b, float) else f'{a}={b}' for a, b in v.items()))
