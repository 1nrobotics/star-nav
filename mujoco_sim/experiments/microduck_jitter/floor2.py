import sys, os, math; sys.path.insert(0, os.path.dirname(__file__))
import numpy as np, mujoco
from star_nav_mujoco import microduck
from jitter import make
from faithful import Faithful
def cam_rate(tw, kw, secs=12):
    s = make(); s.walker = Faithful(s.model, s.data, **kw); s.reset()
    m, d, w = s.model, s.data, s.walker; cam = m.camera(microduck.CAMERA).id
    s.step(int(1 / m.opt.timestep)); prev = None; r = []; yaw0 = None; x0 = None
    for k in range(int(secs / m.opt.timestep)):
        if w.policy_due:
            w.twist = np.asarray(tw, np.float32)
            R = d.cam_xmat[cam].reshape(3, 3).copy()
            if prev is not None and k * m.opt.timestep > 2: r.append(np.arccos(np.clip((np.trace(prev.T @ R) - 1) / 2, -1, 1)) / 0.02)
            prev = R
            if yaw0 is None and k * m.opt.timestep > 2: yaw0 = 2 * math.atan2(d.qpos[6], d.qpos[3]); t0 = d.time; x0 = d.qpos[:2].copy()
        w.before_step(); mujoco.mj_step(m, d)
    yaw1 = 2 * math.atan2(d.qpos[6], d.qpos[3]); T = d.time - t0
    return math.degrees(np.sqrt(np.mean(np.square(r)))), math.remainder(yaw1 - yaw0, 2 * math.pi) / T, np.linalg.norm(d.qpos[:2] - x0) / T
kw = eval(sys.argv[1])
for tw in [(0.2,0,0), (0.3,0,0), (0.4,0,0), (0,0,0.6), (0,0,1.0), (0.2,0,0.6), (0.2,0,1.0)]:
    print(sys.argv[1], tw, 'cam %.1f deg/s  yaw %.2f rad/s  speed %.3f m/s' % cam_rate(tw, kw), flush=True)
