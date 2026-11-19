"""Microduck walker matching the training setup (mjlab BamActuator + backlash encoder)."""
import collections, mujoco, numpy as np
from star_nav_mujoco import microduck as md

P = dict(kt=0.36601349688984386, R=2.8113923539223227, fb=0.004771183165566, fs=0.004676345799486616,
         lfm=0.2667860954283698, lfe=8.515871897059342e-06, lfms=1.0722918395099123e-05,
         lfes=0.08077928978935671, lfmq=0.009972471242139415, lfeq=0.004902565732332559,
         dth=2.890372094130307, alpha=8.683259907618984, fv=0.005359668274599504)
EG = (4096 / (2 * np.pi)) / (256 * 885)

class Faithful(md.Walker):
    def __init__(self, model, data, *, encoder=True, mjlab_friction=True, lag=0, vin=7.4, drop=0.1):
        super().__init__(model, data)
        self.encoder, self.mjlab_friction, self.lag, self.vin, self.drop = encoder, mjlab_friction, lag, vin, drop
        acts = [md.PREFIX + j for j in md.JOINTS]
        jids = [model.actuator(a).trnid[0] for a in acts]
        self.qa = model.jnt_qposadr[jids]; self.da = model.jnt_dofadr[jids]
        bl = [model.joint(f'{md.PREFIX}passive_{j}_backlash').id for j in md.JOINTS]
        self.qb = model.jnt_qposadr[bl]; self.db = model.jnt_dofadr[bl]
        self.reset_servo()

    def reset_servo(self):
        self.prev = np.zeros(14); self.buf = collections.deque(maxlen=self.lag + 1)

    def reset(self):
        super().reset()
        if hasattr(self, 'qa'): self.reset_servo()

    def enc_pos(self):
        d = self.data
        return d.qpos[self.qa] + (d.qpos[self.qb] if self.encoder else 0)

    def observation(self):
        o = super().observation()
        if self.encoder:
            d = self.data
            o[6:20] = (d.qpos[self.qa] + d.qpos[self.qb]) - md.DEFAULT_POSE
            o[20:34] = d.qvel[self.da] + d.qvel[self.db]
        return o

    def before_step(self):
        if self._steps % md.DECIMATION == 0:
            a = self._policy.run(None, {self._input: self.observation()[None]})[0][0]
            self._last = a.astype(np.float32)
        self._steps += 1
        self.buf.append(md.DEFAULT_POSE + self._last)
        target = self.buf[0]  # lag physics steps old
        m, d = self.model, self.data
        vin = max(self.vin - self.drop * np.abs(self.prev).sum(), 6.0)
        q, dq = self.enc_pos(), d.qvel[self.da]
        duty = np.clip((target - q) * 200.0 * EG, -1, 1)
        tau = P['kt'] * vin * duty / P['R'] - P['kt'] ** 2 * dq / P['R']
        d.ctrl[:14] = tau; self.prev = tau
        # friction budget for the next step
        fric = np.zeros(m.nv)
        for k in range(d.nefc):
            if d.efc_type[k] == mujoco.mjtConstraint.mjCNSTR_FRICTION_DOF:
                fric[d.efc_id[k]] += d.efc_force[k]
        ext = -d.qfrc_bias[self.da] + d.qfrc_constraint[self.da] - fric[self.da]
        mot = d.qfrc_actuator[self.da]
        s = np.exp(-np.abs(dq / P['dth']) ** P['alpha'])
        loss = P['fb'] + s * P['fs'] + np.abs(ext * P['lfe'] - mot * P['lfm']) + s * np.abs(ext * P['lfes'] - mot * P['lfms'])
        drive = np.abs(mot) > np.abs(ext)
        loss += s * np.where(drive, P['lfeq'] * ext ** 2, P['lfmq'] * mot ** 2)
        m.dof_frictionloss[self.da] = loss; m.dof_damping[self.da] = P['fv']
