// The walking robots (Lite3, T1) in MuJoCo (WebAssembly), for the browser demo.
//
// A JavaScript port of the simulator's walking controllers
// (mujoco_sim/star_nav_mujoco/robots/lite3.py and t1.py: the manufacturer's
// velocity policy, then a joint PD controller) and of tracking.Holonomic,
// which steers the robot along the path it walked in a recorded exploration
// mission with the CMU planner. The planner itself does not run here.
//
// The robot's MJCF, meshes and policy are not in this repository: load()
// downloads them from the upstream repositories, at the commits the simulator
// pins (<robot>.json, written by site/tools/export_web_scene.py).
// No rendering here, so it also runs under Node for testing.

const ROOT = '/star-nav';  // in MuJoCo's file system

// -- reading the policies' weights -------------------------------------------

function varint(buf, i) {
  let value = 0, shift = 1;
  for (;;) {
    const b = buf[i++];
    value += (b & 0x7f) * shift;
    if (!(b & 0x80)) return [value, i];
    shift *= 128;
  }
}

// The fields of one protobuf message: [number, wire type, value], where a
// length-delimited value is a view of its bytes.
function* fields(buf) {
  let i = 0;
  while (i < buf.length) {
    let key, n;
    [key, i] = varint(buf, i);
    const wire = key & 7;
    if (wire === 0) {
      [n, i] = varint(buf, i);
      yield [key >>> 3, wire, n];
    } else if (wire === 2) {
      [n, i] = varint(buf, i);
      yield [key >>> 3, wire, buf.subarray(i, i + n)];
      i += n;
    } else {
      i += wire === 1 ? 8 : 4;  // fixed64, fixed32: not needed
    }
  }
}

function floats(bytes) {
  // A copy: the bytes are little-endian and may not be aligned.
  return new Float32Array(bytes.slice().buffer);
}

// Dense layers [[weight, bias], ...] of an ONNX file whose initializers are
// float32 raw data named "<...>.<i>.weight" / "<...>.<i>.bias" (the Lite3's).
export function onnxLayers(bytes) {
  const text = new TextDecoder();
  const found = {};
  for (const [num, , graph] of fields(bytes)) {
    if (num !== 7) continue;  // ModelProto.graph
    for (const [gnum, , tensor] of fields(graph)) {
      if (gnum !== 5) continue;  // GraphProto.initializer
      const t = { dims: [] };
      for (const [f, wire, v] of fields(tensor)) {
        if (f === 1 && wire === 0) t.dims.push(v);
        else if (f === 1) for (let i = 0; i < v.length;) { let d; [d, i] = varint(v, i); t.dims.push(d); }
        else if (f === 2) t.type = v;
        else if (f === 8) t.name = text.decode(v);
        else if (f === 9) t.raw = v;
      }
      if (t.type !== 1 || !t.raw) throw new Error(`ONNX tensor ${t.name}: not float32 raw data`);
      found[t.name] = { dims: t.dims, data: floats(t.raw) };
    }
  }
  const index = (name) => Number(name.split('.').at(-2));
  const weights = Object.keys(found).filter((n) => n.endsWith('.weight')).sort((a, b) => index(a) - index(b));
  return weights.map((name) => {
    const w = found[name], b = found[name.replace(/weight$/, 'bias')];
    if (!b || w.dims.length !== 2 || b.data.length !== w.dims[0]) throw new Error(`ONNX layer ${name}: unexpected shape`);
    return { rows: w.dims[0], cols: w.dims[1], w: w.data, b: b.data };
  });
}

// Dense layers of a TorchScript archive (the T1's): a zip whose entries
// "<name>/data/<i>" are raw little-endian float32, weight then bias of each
// layer (as t1.extract_weights reads them). shapes: [[rows, cols], ...].
export function torchscriptLayers(bytes, shapes) {
  const view = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
  const text = new TextDecoder();
  let end = bytes.length - 22;
  while (end >= 0 && view.getUint32(end, true) !== 0x06054b50) end--;
  if (end < 0) throw new Error('policy archive: not a zip file');
  const count = view.getUint16(end + 10, true);
  const entries = {};
  let p = view.getUint32(end + 16, true);
  for (let n = 0; n < count; n++) {
    if (view.getUint32(p, true) !== 0x02014b50) throw new Error('policy archive: bad directory');
    const method = view.getUint16(p + 10, true);
    const size = view.getUint32(p + 24, true);
    const nameLen = view.getUint16(p + 28, true);
    const local = view.getUint32(p + 42, true);
    const name = text.decode(bytes.subarray(p + 46, p + 46 + nameLen));
    const start = local + 30 + view.getUint16(local + 26, true) + view.getUint16(local + 28, true);
    entries[name] = { method, data: bytes.subarray(start, start + size) };
    p += 46 + nameLen + view.getUint16(p + 30, true) + view.getUint16(p + 32, true);
  }
  const top = Object.keys(entries)[0].split('/')[0];
  const read = (name) => {
    const e = entries[`${top}/${name}`];
    if (!e || e.method !== 0) throw new Error(`policy archive: ${name} missing or compressed`);
    return e.data;
  };
  if (text.decode(read('byteorder')).trim() !== 'little') throw new Error('policy archive: not little-endian');
  return shapes.map(([rows, cols], i) => {
    const w = floats(read(`data/${2 * i}`)), b = floats(read(`data/${2 * i + 1}`));
    if (w.length !== rows * cols || b.length !== rows) throw new Error(`policy archive: layer ${i} has an unexpected size`);
    return { rows, cols, w, b };
  });
}

// Dense layers with ELU between them (policy.mlp).
export function mlp(layers, x) {
  layers.forEach(({ rows, cols, w, b }, k) => {
    const y = new Float32Array(rows);
    for (let r = 0, o = 0; r < rows; r++, o += cols) {
      let s = b[r];
      for (let c = 0; c < cols; c++) s += w[o + c] * x[c];
      y[r] = k < layers.length - 1 && s < 0 ? Math.expm1(s) : s;
    }
    x = y;
  });
  return x;
}

// -- tracking.Holonomic -------------------------------------------------------

const clip = (v, lo, hi) => Math.min(hi, Math.max(lo, v));
const wrap = (a) => Math.atan2(Math.sin(a), Math.cos(a));

// Body-frame velocity (forward, sideways, turn rate) towards a setpoint.
export function trackTwist(k, sp, pos, yaw) {
  const ex = sp.pos[0] - pos[0], ey = sp.pos[1] - pos[1];
  let vx = sp.vel[0] + k.k_pos * ex, vy = sp.vel[1] + k.k_pos * ey;
  if (Math.hypot(sp.vel[0], sp.vel[1]) < 0.02 && Math.hypot(ex, ey) < k.hold) vx = vy = 0;
  const speed = Math.hypot(vx, vy);
  if (speed > k.max_speed) { vx *= k.max_speed / speed; vy *= k.max_speed / speed; }
  const c = Math.cos(yaw), s = Math.sin(yaw);
  const yawErr = wrap(sp.yaw - yaw);
  const turn = Math.abs(yawErr) < k.hold_yaw ? 0 : clip(k.k_yaw * yawErr, -k.max_turn, k.max_turn);
  return [c * vx + s * vy, -s * vx + c * vy, turn];
}

// A velocity command within the policy's limits.
export function limitTwist(k, [fwd, side, turn]) {
  const speed = Math.hypot(fwd, side);
  if (speed > k.max_speed) { fwd *= k.max_speed / speed; side *= k.max_speed / speed; }
  return [fwd, side, clip(turn, -k.max_turn, k.max_turn)];
}

// Linear interpolation in the recorded path (rows of t, x, y, z, yaw, vx, vy).
export function pathAt(rows, t) {
  const at = (r, moving) => ({ pos: [r[1], r[2], r[3]], yaw: r[4], vel: moving ? [r[5], r[6]] : [0, 0] });
  if (t <= rows[0][0]) return at(rows[0], false);
  const last = rows[rows.length - 1];
  if (t >= last[0]) return at(last, false);
  let lo = 0, hi = rows.length - 1;
  while (hi - lo > 1) {
    const mid = (lo + hi) >> 1;
    if (rows[mid][0] <= t) lo = mid; else hi = mid;
  }
  const a = rows[lo], b = rows[hi], s = (t - a[0]) / (b[0] - a[0]);
  return at(a.map((v, i) => v + (b[i] - v) * s), true);
}

// -- the robots' policies -----------------------------------------------------

// What the policy sees. s: {gravity, omega, twist, q, qd, last, phase}, with
// q and qd the joints' offsets from the standing pose and their velocities.
const OBSERVATION = {
  lite3: (cfg, s) => [
    ...s.omega.map((w) => w * cfg.omega_scale), ...s.gravity,
    ...s.twist.map((v) => clip(v, -cfg.max_cmd, cfg.max_cmd)),
    ...s.q, ...s.qd.map((v) => v * cfg.joint_vel_scale), ...s.last],
  t1: (cfg, s) => {
    const on = s.twist.some((v) => v !== 0) ? 1 : 0;
    return [
      ...s.gravity, ...s.omega, ...s.twist.map((v) => clip(v, -cfg.max_cmd, cfg.max_cmd)),
      Math.cos(2 * Math.PI * s.phase) * on, Math.sin(2 * Math.PI * s.phase) * on,
      ...s.q, ...s.qd.map((v) => v * cfg.joint_vel_scale), ...s.last];
  },
};

// Policy every cfg.decimation steps, joint PD every step.
export class Locomotion {
  constructor(model, data, cfg, layers) {
    Object.assign(this, { model, data, cfg, layers });
    const n = cfg.actuators.length;
    if (model.nu !== n) throw new Error(`${cfg.robot}: ${model.nu} actuators, expected ${n}`);
    cfg.actuators.forEach((name, i) => {
      if (model.actuator(i).name !== name) throw new Error(`${cfg.robot}: unexpected actuator order`);
    });
    const trn = model.actuator_trnid, qadr = model.jnt_qposadr, vadr = model.jnt_dofadr;
    this.qpos = cfg.actuators.map((_, i) => qadr[trn[2 * i]]);
    this.qvel = cfg.actuators.map((_, i) => vadr[trn[2 * i]]);
    this.trunk = model.body(cfg.trunk).id;
    this.rootVel = vadr[model.body_jntadr[this.trunk]];
    const range = model.actuator_ctrlrange;
    this.lo = cfg.actuators.map((_, i) => (cfg.torque_limit ? -cfg.torque_limit : range[2 * i]));
    this.hi = cfg.actuators.map((_, i) => cfg.torque_limit ?? range[2 * i + 1]);
    // The T1's action is clipped and added to the pose as it is.
    this.scale = cfg.action_scale ?? cfg.actuators.map(() => 1);
    this.clipAction = !cfg.action_scale;
    this.observe = OBSERVATION[cfg.robot];
    this.twist = [0, 0, 0];
    this.reset();
  }

  reset() {
    const q = this.data.qpos;
    this.qpos.forEach((adr, i) => { q[adr] = this.cfg.default_pose[i]; });
    this.last = this.cfg.actuators.map(() => 0);
    this.target = [...this.cfg.default_pose];
    this.steps = 0;
    this.phase = 0;
  }

  get policyDue() {
    return this.steps % this.cfg.decimation === 0;
  }

  beforeStep() {
    const { data, cfg } = this;
    const q = data.qpos, v = data.qvel;
    if (this.policyDue) {
      const x = data.xmat, o = 9 * this.trunk, r = this.rootVel;
      const obs = this.observe(cfg, {
        gravity: [-x[o + 6], -x[o + 7], -x[o + 8]],  // world down, in the body frame
        omega: [v[r + 3], v[r + 4], v[r + 5]],  // body frame
        twist: this.twist,
        q: this.qpos.map((adr, i) => q[adr] - cfg.default_pose[i]),
        qd: this.qvel.map((adr) => v[adr]),
        last: this.last,
        phase: this.phase,
      });
      const action = mlp(this.layers, Float32Array.from(obs));
      this.last = Array.from(action, (a) => (this.clipAction ? clip(a, -1, 1) : a));
      this.target = this.last.map((a, i) => cfg.default_pose[i] + a * this.scale[i]);
    }
    this.steps += 1;
    if (cfg.gait_frequency && this.twist.some((t) => t !== 0)) {
      this.phase = (this.phase + this.model.opt.timestep * cfg.gait_frequency) % 1;
    }
    const ctrl = data.ctrl;
    for (let i = 0; i < this.target.length; i++) {
      const torque = cfg.kp[i] * (this.target[i] - q[this.qpos[i]]) - cfg.kd[i] * v[this.qvel[i]];
      ctrl[i] = clip(torque, this.lo[i], this.hi[i]);
    }
  }
}

// -- the mission loop ---------------------------------------------------------

// Owns the MuJoCo model/data: stand, walk the recorded path, stand, start
// over. With drive() set, a velocity of the user's goes to the policy instead.
export class Walk {
  constructor(mujoco, model, cfg, layers, path, { stand = 1.0, hold = 2.5, lag = 0.4, down = 2.0 } = {}) {
    this.mj = mujoco;
    this.model = model;
    this.data = new mujoco.MjData(model);
    this.cfg = cfg;
    this.rows = path.rows;
    Object.assign(this, { standTime: stand, holdTime: hold, lag, downTime: down });
    this.locomotion = new Locomotion(model, this.data, cfg, layers);
    this.manual = null;
    this.falls = 0;
    this.reset();
  }

  reset() {
    const { mj, model, data } = this;
    mj.mj_resetData(model, data);
    data.qpos[2] = this.cfg.start_height;
    this.locomotion.reset();
    this.clock = -this.standTime;  // time along the recorded path
    this.fallen = null;  // simulated time of a fall
    this.waiting = false;
    this.laps = (this.laps ?? -1) + 1;
    mj.mj_forward(model, data);
  }

  get time() {
    return this.data.time;
  }

  position() {
    const q = this.data.qpos;
    return [q[0], q[1], q[2]];
  }

  // The point of the recorded path the robot is steered to.
  target() {
    return pathAt(this.rows, this.clock);
  }

  // A body-frame velocity (forward, sideways, turn) to follow instead of the
  // path; null goes back to the path, from its start.
  drive(twist) {
    if (!twist && this.manual) this.reset();
    this.manual = twist;
  }

  step() {
    const { mj, model, data, cfg, locomotion } = this;
    const q = data.qpos, dt = model.opt.timestep;
    if (this.fallen === null && q[2] < 0.5 * cfg.stand_height) {
      this.fallen = data.time;
      this.falls += 1;
    }
    const end = this.rows[this.rows.length - 1][0] + this.holdTime;
    if ((this.fallen !== null && data.time - this.fallen > this.downTime) ||
        (!this.manual && this.clock >= end)) {
      this.reset();
    }
    if (locomotion.policyDue) {
      if (this.manual) {
        locomotion.twist = limitTwist(cfg.tracker, this.manual);
      } else {
        const sp = this.target();
        const yaw = 2 * Math.atan2(q[6], q[3]);  // the body stays level
        locomotion.twist = trackTwist(cfg.tracker, sp, q, yaw);
        // The path's clock waits for a robot that has fallen behind.
        this.waiting = Math.hypot(sp.pos[0] - q[0], sp.pos[1] - q[1]) > this.lag;
      }
    }
    if (!this.manual && !this.waiting) this.clock += dt;
    locomotion.beforeStep();
    mj.mj_step(model, data);
  }

  // Advances the simulation by `seconds` of simulated time.
  advance(seconds) {
    const n = Math.round(seconds / this.model.opt.timestep);
    for (let i = 0; i < n; i++) this.step();
  }
}

// -- loading ------------------------------------------------------------------

async function readBytes(url) {
  const r = await fetch(url);
  if (!r.ok) throw new Error(`${url}: HTTP ${r.status}`);
  return new Uint8Array(await r.arrayBuffer());
}

async function sha256(bytes) {
  const digest = await crypto.subtle.digest('SHA-256', bytes);
  return Array.from(new Uint8Array(digest), (b) => b.toString(16).padStart(2, '0')).join('');
}

function writeFile(FS, path, bytes) {
  const parts = path.split('/').slice(1, -1);
  let dir = '';
  for (const part of parts) {
    dir += '/' + part;
    try { FS.mkdir(dir); } catch (e) { /* exists */ }
  }
  FS.writeFile(path, bytes);
}

// Loads a walking robot ('lite3' or 't1') and returns a Walk. base: the URL
// of this folder; read(url) -> Uint8Array replaces fetch (tests); progress
// (done, total) is called as the upstream files arrive.
export async function load(mujoco, name, { base = new URL('.', import.meta.url), read = readBytes, progress } = {}) {
  const text = new TextDecoder();
  const local = (file) => read(new URL(file, base).href);
  const [cfg, scene, path] = await Promise.all([
    local(`${name}.json`).then((b) => JSON.parse(text.decode(b))),
    local(`${name}.xml`),
    local(`${name}-path.json`).then((b) => JSON.parse(text.decode(b))),
  ]);
  const files = Object.entries(cfg.files);
  let done = 0;
  const count = (bytes) => { progress?.(++done, files.length + 1); return bytes; };
  const [policy] = await Promise.all([
    read(cfg.policy.url).then(count),
    ...files.map(([file, url]) => read(url).then(count).then((bytes) => writeFile(mujoco.FS, `${ROOT}/${file}`, bytes))),
  ]);
  if (await sha256(policy) !== cfg.policy.sha256) throw new Error(`${cfg.policy.url}: not the pinned policy`);
  const layers = cfg.policy.format === 'onnx' ? onnxLayers(policy) : torchscriptLayers(policy, cfg.policy.layers);
  writeFile(mujoco.FS, `${ROOT}/${name}.xml`, scene);
  const model = mujoco.MjModel.from_xml_path(`${ROOT}/${name}.xml`);
  return new Walk(mujoco, model, cfg, layers, path);
}
