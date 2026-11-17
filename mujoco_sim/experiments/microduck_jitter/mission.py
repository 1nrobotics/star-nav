"""Replays the recorded duck mission through a tracker; reports shake and mode switching."""
import json, math, sys, os, dataclasses
os.environ.setdefault('MUJOCO_GL', 'egl')
sys.path.insert(0, os.path.dirname(__file__))
import numpy as np, mujoco
from star_nav_mujoco import microduck, sim, world
from jitter import make

ROWS = json.load(open(os.path.join(os.path.dirname(os.path.abspath(__file__)), '../../../site/assets/mujoco/microduck_setpoints.json')))['rows']
R = np.array(ROWS)

def setpoint(t):
    t = min(max(t, R[0, 0]), R[-1, 0])
    i = np.searchsorted(R[:, 0], t)
    i = min(max(i, 1), len(R) - 1)
    a, b = R[i - 1], R[i]
    s = 0 if b[0] == a[0] else (t - a[0]) / (b[0] - a[0])
    r = a + (b - a) * s
    return r[1:4], r[4:7]

def mode(tw):
    if not tw.any(): return 'stand'
    return 'turn' if tw[0] == 0 else 'walk'

def run(tracker, seconds=None, obstacles=True):
    s = make(None if obstacles else [])
    m, d, w = s.model, s.data, s.walker
    cam = m.camera(microduck.CAMERA).id
    trunk = m.body(microduck.TRUNK).id
    end = seconds or (R[-1, 0] + 3)
    modes, err, camw, yawr, contacts = [], [], [], [], 0
    prev_cam = None
    n = int(end / m.opt.timestep)
    for k in range(n):
        if w.policy_due:
            p, v = setpoint(d.time - 1.0)
            yaw = 2 * math.atan2(d.qpos[6], d.qpos[3])
            w.twist = np.asarray(tracker.twist(p, v, d.qpos[:3], yaw), np.float32)
            modes.append(mode(w.twist)); err.append(np.hypot(*(p[:2] - d.qpos[:2])))
            R_cam = d.cam_xmat[cam].reshape(3, 3).copy()
            if prev_cam is not None:
                dR = prev_cam.T @ R_cam
                camw.append(np.arccos(np.clip((np.trace(dR) - 1) / 2, -1, 1)) / 0.02)
            prev_cam = R_cam
            yawr.append(d.qvel[5])
        w.before_step(); mujoco.mj_step(m, d)
        if d.ncon and s._touching(): contacts += 1
    sw = sum(a != b for a, b in zip(modes, modes[1:]))
    frac = {k: modes.count(k) / len(modes) for k in ('stand', 'turn', 'walk')}
    return dict(time=end, mode_switches_per_min=60 * sw / end,
                stand=frac['stand'], turn=frac['turn'], walk=frac['walk'],
                cam_rate_rms_deg=math.degrees(np.sqrt(np.mean(np.square(camw)))),
                cam_rate_p95_deg=math.degrees(np.percentile(camw, 95)),
                err_mean=float(np.mean(err)), err_max=float(np.max(err)),
                final_err=float(err[-1]), contact_steps=contacts,
                fallen=bool(d.qpos[2] < 0.6 * microduck.STAND_HEIGHT))

if __name__ == '__main__':
    import trackers
    for name in sys.argv[1:]:
        r = run(getattr(trackers, name)())
        print(f'{name:12s} ' + ' '.join(f'{k}={v:.3f}' if isinstance(v, float) else f'{k}={v}' for k, v in r.items()), flush=True)
