import math, numpy as np
from star_nav_mujoco import microduck

def current(): return microduck.Tracker()

class Smooth(microduck.Tracker):
    """Stateful tracker: hysteresis between stand / turn / walk, minimum dwell, smoothed output."""
    def __init__(self, start_walk=0.15, stop_walk=0.06, turn_enter=math.radians(60), turn_exit=math.radians(20),
                 dwell=1.0, tau=0.3, dt=0.02, wait=True, **kw):
        super().__init__(**kw)
        self.start_walk, self.stop_walk, self.turn_enter, self.turn_exit = start_walk, stop_walk, turn_enter, turn_exit
        self.dwell, self.a, self.dt, self.wait = dwell, dt / (tau + dt), dt, wait
        self.state, self.t_state, self.out = 'stand', 0.0, np.zeros(3, np.float32)

    def _go(self, s):
        if s != self.state: self.state, self.t_state = s, 0.0

    def twist(self, sp_pos, sp_vel, pos, yaw):
        self.t_state += self.dt
        err = np.asarray(sp_pos[:2]) - pos[:2]; v = np.asarray(sp_vel[:2], float)
        dist, sp_speed = float(np.hypot(*err)), float(np.hypot(*v))
        vel = v + self.k_pos * err
        turn = math.remainder(math.atan2(vel[1], vel[0]) - yaw, 2 * math.pi)
        ahead = False
        if sp_speed > 0.01 and dist < self.wait_ahead:
            p = v / sp_speed
            ahead = float(err @ p) < 0 and abs(err[0] * p[1] - err[1] * p[0]) < self.arrive
        settled = self.t_state >= self.dwell
        if self.state == 'stand':
            if settled and dist > self.start_walk and not (self.wait and ahead):
                self._go('turn' if abs(turn) > self.turn_enter else 'walk')
        elif self.state == 'turn':
            if abs(turn) < self.turn_exit: self._go('walk')
            elif dist < self.stop_walk and sp_speed < 0.02: self._go('stand')
        else:  # walk
            if settled and ((dist < self.stop_walk and sp_speed < 0.02) or (self.wait and ahead and dist > 0.1)):
                self._go('stand')
            elif abs(turn) > self.turn_enter + math.radians(30): self._go('turn')
        if self.state == 'stand': target = np.zeros(3)
        elif self.state == 'turn': target = np.array([0, 0, math.copysign(self.turn_in_place, turn)])
        else:
            speed = float(np.hypot(*vel))
            fwd = np.clip((speed + self.speed_offset) / self.speed_gain, self.min_walk_cmd, self.max_walk_cmd)
            target = np.array([fwd, 0, np.clip(self.k_yaw * turn, -self.max_turn_walking, self.max_turn_walking)])
        # Smooth only within a mode; snap to zero when standing so the policy really stands.
        self.out = (self.out + self.a * (target - self.out)) if self.state == 'walk' else target
        return self.out.astype(np.float32)

def smooth(): return Smooth()
def smooth_nodwell(): return Smooth(dwell=0.0, tau=0.0)
def smooth_long(): return Smooth(dwell=2.0)

class Pursuit(microduck.Tracker):
    """Walk at one calm speed, steer while walking, stand only at resting setpoints."""
    def __init__(self, walk=0.3, k_turn=1.2, max_turn=0.6, start=0.12, stop=0.06, dwell=1.0, dt=0.02, tau=0.4, **kw):
        super().__init__(**kw)
        self.walk, self.k_turn, self.max_turn, self.start, self.stop, self.dwell, self.dt = walk, k_turn, max_turn, start, stop, dwell, dt
        self.a = dt / (tau + dt); self.walking = False; self.t = 0.0; self.wz = 0.0
    def twist(self, sp_pos, sp_vel, pos, yaw):
        self.t += self.dt
        err = np.asarray(sp_pos[:2]) - pos[:2]; v = np.asarray(sp_vel[:2], float)
        dist, sp_speed = float(np.hypot(*err)), float(np.hypot(*v))
        ahead = False
        if sp_speed > 0.01 and dist < self.wait_ahead:
            p = v / sp_speed
            ahead = float(err @ p) < 0 and abs(err[0] * p[1] - err[1] * p[0]) < self.arrive
        if self.walking and self.t > self.dwell and ((dist < self.stop and sp_speed < 0.02) or (ahead and dist > 0.08)):
            self.walking, self.t = False, 0.0
        elif not self.walking and self.t > self.dwell and dist > self.start and not ahead:
            self.walking, self.t = True, 0.0
        if not self.walking:
            self.wz = 0.0
            return np.zeros(3, np.float32)
        aim = v * 1.0 + err  # 1 s lookahead along the planner's motion
        turn = math.remainder(math.atan2(aim[1], aim[0]) - yaw, 2 * math.pi)
        self.wz += self.a * (np.clip(self.k_turn * turn, -self.max_turn, self.max_turn) - self.wz)
        return np.array([self.walk, 0.0, self.wz], np.float32)

def pursuit(): return Pursuit()
def pursuit_fast_turn(): return Pursuit(max_turn=1.0)
