// Quadrotor flight in MuJoCo (WebAssembly), for the browser demo.
//
// A JavaScript port of mujoco_sim/star_nav_mujoco/quadrotor.py (same
// parameters, same geometric SE(3) controller) that replays the setpoints
// EGO-Planner sent during a recorded exploration mission in the same forest.
// No rendering here, so it also runs under Node for testing.

const GRAVITY = 9.81;

export const PARAMS = {
  mass: 1.0,
  inertia: [0.0049, 0.0049, 0.0088],
  arm: 0.12,
  torqueCoeff: 0.016,
  maxThrust: 6.0,
  drag: 0.15,
};

export const GAINS = {
  kp: [7.0, 7.0, 10.0],
  kv: [4.5, 4.5, 6.0],
  ki: [0.8, 0.8, 1.5],
  kr: [2.5, 2.5, 0.8],
  kw: [0.18, 0.18, 0.12],
  maxTiltDeg: 35.0,
  maxIntegral: 2.0,
  integralZone: 0.3,
};

const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const norm = (a) => Math.hypot(a[0], a[1], a[2]);
const scale = (a, s) => [a[0] * s, a[1] * s, a[2] * s];
// 3x3 matrices as arrays of rows.
const matT = (m) => [[m[0][0], m[1][0], m[2][0]], [m[0][1], m[1][1], m[2][1]], [m[0][2], m[1][2], m[2][2]]];
const matMul = (a, b) => a.map((r) => [0, 1, 2].map((j) => r[0] * b[0][j] + r[1] * b[1][j] + r[2] * b[2][j]));
const matVec = (m, v) => m.map((r) => dot(r, v));

function invert4(m) {
  // Gauss-Jordan inverse of a 4x4 matrix (array of rows).
  const a = m.map((r, i) => [...r, ...[0, 1, 2, 3].map((j) => (i === j ? 1 : 0))]);
  for (let c = 0; c < 4; c++) {
    let p = c;
    for (let r = c + 1; r < 4; r++) if (Math.abs(a[r][c]) > Math.abs(a[p][c])) p = r;
    [a[c], a[p]] = [a[p], a[c]];
    const d = a[c][c];
    for (let j = 0; j < 8; j++) a[c][j] /= d;
    for (let r = 0; r < 4; r++) {
      if (r === c) continue;
      const f = a[r][c];
      for (let j = 0; j < 8; j++) a[r][j] -= f * a[c][j];
    }
  }
  return a.map((r) => r.slice(4));
}

export class Controller {
  constructor(p = PARAMS, g = GAINS) {
    this.p = p;
    this.g = g;
    const a = p.arm;
    const rotors = [[a, a, 1], [-a, -a, 1], [a, -a, -1], [-a, a, -1]];
    // Rows: collective thrust, Mx, My, Mz; one column per rotor.
    const mixer = [
      rotors.map(() => 1),
      rotors.map(([, y]) => y),
      rotors.map(([x]) => -x),
      rotors.map(([, , s]) => s * p.torqueCoeff),
    ];
    this.mixInv = invert4(mixer);
    this.tanTilt = Math.tan((g.maxTiltDeg * Math.PI) / 180);
    this.reset();
  }

  reset() {
    this.integral = [0, 0, 0];
  }

  // cmd: {pos, vel, acc, yaw, yawRate}; rot: body-to-world (rows);
  // vel: world; omega: body. Returns four rotor thrusts.
  update(cmd, pos, rot, vel, omega, dt) {
    const { p, g } = this;
    const m = p.mass;
    const force = [0, 0, 0];
    for (let i = 0; i < 3; i++) {
      const ep = pos[i] - cmd.pos[i];
      const ev = vel[i] - cmd.vel[i];
      if (Math.abs(ep) < g.integralZone) {
        this.integral[i] = Math.min(g.maxIntegral, Math.max(-g.maxIntegral, this.integral[i] + ep * dt));
      }
      force[i] = m * (-g.kp[i] * ep - g.kv[i] * ev - g.ki[i] * this.integral[i] + cmd.acc[i]);
    }
    force[2] += m * GRAVITY;
    force[2] = Math.max(force[2], 0.2 * m * GRAVITY);
    const horiz = Math.hypot(force[0], force[1]);
    const maxHoriz = force[2] * this.tanTilt;
    if (horiz > maxHoriz) {
      force[0] *= maxHoriz / horiz;
      force[1] *= maxHoriz / horiz;
    }

    const b3 = scale(force, 1 / norm(force));
    const b1c = [Math.cos(cmd.yaw), Math.sin(cmd.yaw), 0];
    let b2 = cross(b3, b1c);
    b2 = scale(b2, 1 / norm(b2));
    const b1 = cross(b2, b3);
    const rotD = [[b1[0], b2[0], b3[0]], [b1[1], b2[1], b3[1]], [b1[2], b2[2], b3[2]]];

    const a = matMul(matT(rotD), rot);
    const b = matMul(matT(rot), rotD);
    const eR = [0.5 * (a[2][1] - b[2][1]), 0.5 * (a[0][2] - b[0][2]), 0.5 * (a[1][0] - b[1][0])];
    const omegaD = matVec(b, [0, 0, cmd.yawRate]);
    const Iw = omega.map((w, i) => p.inertia[i] * w);
    const wxIw = cross(omega, Iw);
    const moment = [0, 1, 2].map((i) => -g.kr[i] * eR[i] - g.kw[i] * (omega[i] - omegaD[i]) + wxIw[i]);
    const thrust = dot(force, [rot[0][2], rot[1][2], rot[2][2]]);

    const u = [thrust, ...moment];
    return this.mixInv.map((r) => Math.min(p.maxThrust, Math.max(0, dot4(r, u))));
  }
}

const dot4 = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];

// Linear interpolation in the recorded setpoints (rows of t, x, y, z, vx, vy,
// vz, ax, ay, az, yaw, yaw_rate).
export function setpointAt(rows, t) {
  const toCmd = (r) => ({ pos: r.slice(1, 4), vel: r.slice(4, 7), acc: r.slice(7, 10), yaw: r[10], yawRate: r[11] });
  if (t <= rows[0][0]) return toCmd(rows[0]);
  const last = rows[rows.length - 1];
  if (t >= last[0]) return { ...toCmd(last), vel: [0, 0, 0], acc: [0, 0, 0], yawRate: 0 };
  let lo = 0, hi = rows.length - 1;
  while (hi - lo > 1) {
    const mid = (lo + hi) >> 1;
    if (rows[mid][0] <= t) lo = mid; else hi = mid;
  }
  const a = rows[lo], b = rows[hi];
  const s = (t - a[0]) / (b[0] - a[0]);
  const r = a.map((v, i) => v + (b[i] - v) * s);
  let dyaw = b[10] - a[10];
  dyaw = Math.atan2(Math.sin(dyaw), Math.cos(dyaw));
  r[10] = a[10] + dyaw * s;
  return toCmd(r);
}

// Owns the MuJoCo model/data and runs the mission loop: hover, replay the
// planner's setpoints, hold, then start over.
export class Flight {
  constructor(mujoco, xml, setpoints, { hover = 1.5, hold = 2.5 } = {}) {
    this.mj = mujoco;
    this.model = mujoco.MjModel.from_xml_string(xml);
    this.data = new mujoco.MjData(this.model);
    this.rows = setpoints.rows;
    this.hoverTime = hover;
    this.holdTime = hold;
    this.duration = hover + this.rows[this.rows.length - 1][0] + hold;
    this.quad = this.bodyId('quad', 1);
    this.mocap = 0;
    this.controller = new Controller();
    this.reset();
  }

  bodyId(name, fallback) {
    try {
      return this.model.body(name).id;
    } catch (e) {
      return fallback;
    }
  }

  reset() {
    const { mj, model, data } = this;
    mj.mj_resetData(model, data);
    const hover = (PARAMS.mass * GRAVITY) / 4;
    data.ctrl.fill(hover);
    data.act.fill(hover);
    this.controller.reset();
    this.contacts = 0;
    this.laps = (this.laps ?? -1) + 1;
    mj.mj_forward(model, data);
  }

  get time() {
    return this.data.time;
  }

  // Setpoint for the current time in the mission loop.
  command() {
    return setpointAt(this.rows, this.data.time - this.hoverTime);
  }

  step() {
    const { mj, model, data, quad } = this;
    if (data.time >= this.duration) this.reset();
    const cmd = this.command();
    const q = data.qpos, v = data.qvel, x = data.xmat;
    const o = 9 * quad;
    const rot = [[x[o], x[o + 1], x[o + 2]], [x[o + 3], x[o + 4], x[o + 5]], [x[o + 6], x[o + 7], x[o + 8]]];
    const thrusts = this.controller.update(cmd, [q[0], q[1], q[2]], rot, [v[0], v[1], v[2]],
                                           [v[3], v[4], v[5]], model.opt.timestep);
    const ctrl = data.ctrl;
    for (let i = 0; i < 4; i++) ctrl[i] = thrusts[i];
    const f = data.xfrc_applied;
    for (let i = 0; i < 3; i++) f[6 * quad + i] = -PARAMS.drag * v[i];
    const mp = data.mocap_pos;
    for (let i = 0; i < 3; i++) mp[3 * this.mocap + i] = cmd.pos[i];
    mj.mj_step(model, data);
    if (data.ncon > 0) this.contacts += 1;
  }

  // Advances the simulation by `seconds` of simulated time.
  advance(seconds) {
    const n = Math.round(seconds / this.model.opt.timestep);
    for (let i = 0; i < n; i++) this.step();
  }

  position() {
    const q = this.data.qpos;
    return [q[0], q[1], q[2]];
  }
}
