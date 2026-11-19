// Microduck walking in MuJoCo (WebAssembly), for the browser demo.
//
// A JavaScript port of mujoco_sim/star_nav_mujoco/microduck.py: the velstand
// walking policy, the BAM model of the XL330 servos and the Tracker that turns
// the planner's setpoints into walking commands. It replays the setpoints
// EGO-Planner sent during a recorded mission in the same forest.
//
// The duck's meshes (CC BY-NC-SA 4.0) and policy (Apache-2.0) are not part of
// this site: the browser downloads them from the pinned upstream revisions.
// No rendering here.

import { setpointAt } from './flight.js';

const MODEL_COMMIT = '0daab160d3fb1c2cbaba16a4a19052f36709dd9e';
export const MESH_BASE = 'https://raw.githubusercontent.com/pollen-robotics/microduck_rl/' +
  `${MODEL_COMMIT}/src/mjlab_microduck/robot/microduck/assets/`;
export const POLICY_URL = 'https://huggingface.co/pollen-robotics/microduck-policies/resolve/' +
  '236d1835b03ac7b45d962efbf2fe3a6940206112/velstand.onnx';
const POLICY_SHA256 = 'ce894d30b3343036d499bfac1102bdaa58d59a21c1b58401ecb70fb6a2f63e0a';

const PREFIX = 'duck/';
const DECIMATION = 4;  // policy at 50 Hz, physics at 200 Hz
const STAND_HEIGHT = 0.125;
export const JOINTS = ['left_hip_yaw', 'left_hip_roll', 'left_hip_pitch', 'left_knee',
  'left_ankle', 'neck_pitch', 'head_pitch', 'head_yaw', 'head_roll',
  'right_hip_yaw', 'right_hip_roll', 'right_hip_pitch', 'right_knee', 'right_ankle'];
const DEFAULT_POSE = [0.0, -0.0873, -0.4579, -0.0049, 0.4530, 0.3491, 0.3491, 0.0, 0.0,
  0.0, 0.0873, 0.4579, 0.0049, -0.4530];

// BAM M6 model of the XL330 (bam/params/xl330/m6.json) with the settings of
// microduck.Walker: kp 200, 7.4 V with a load-dependent drop, no current limit.
export const BAM = {
  kt: 0.36601349688984386,
  R: 2.8113923539223227,
  frictionBase: 0.004771183165566,
  frictionStribeck: 0.004676345799486616,
  loadFrictionMotor: 0.2667860954283698,
  loadFrictionExternal: 8.515871897059342e-06,
  loadFrictionMotorStribeck: 1.0722918395099123e-05,
  loadFrictionExternalStribeck: 0.08077928978935671,
  loadFrictionMotorQuad: 0.009972471242139415,
  loadFrictionExternalQuad: 0.004902565732332559,
  dthetaStribeck: 2.890372094130307,
  alpha: 8.683259907618984,
  frictionViscous: 0.005359668274599504,
  kp: 200.0,
  errorGain: (4096 / (2 * Math.PI)) / (256 * 885),
  maxPwm: 1.0,
  vin: 7.4,
  vinDropGain: 0.1,
  vinMin: 6.0,
};
const FRICTION_DOF = 1;  // mjCNSTR_FRICTION_DOF

// BAM's friction budget (Model.compute_frictions, m6): [frictionloss, damping].
export function frictions(motor, external, dq, b = BAM) {
  const gearbox = Math.abs(external * b.loadFrictionExternal - motor * b.loadFrictionMotor);
  const gearboxStribeck = Math.abs(external * b.loadFrictionExternalStribeck -
                                   motor * b.loadFrictionMotorStribeck);
  const stribeck = Math.exp(-(Math.abs(dq / b.dthetaStribeck) ** b.alpha));
  let loss = b.frictionBase + gearbox + stribeck * b.frictionStribeck + gearboxStribeck * stribeck;
  if (Math.sign(external) !== Math.sign(motor)) {
    const ae = Math.abs(external), am = Math.abs(motor);
    if (ae < am) loss += stribeck * b.loadFrictionExternalQuad * ae * ae;
    else if (ae > am) loss += stribeck * b.loadFrictionMotorQuad * am * am;
  }
  return [loss, b.frictionViscous];
}

// -- policy -------------------------------------------------------------------

// Reads the initializers (name -> Float32Array) of an ONNX file: just enough
// protobuf for ModelProto.graph (7) -> GraphProto.initializer (5) ->
// TensorProto {dims (1), float_data (4), name (8), raw_data (9)}.
export function onnxInitializers(bytes) {
  function* fields(buf, start, end) {
    let i = start;
    const varint = () => {
      let v = 0, s = 1, b;
      do { b = buf[i++]; v += (b & 0x7f) * s; s *= 128; } while (b & 0x80);
      return v;
    };
    while (i < end) {
      const key = varint(), field = Math.floor(key / 8), wire = key & 7;
      if (wire === 0) yield [field, wire, varint()];
      else if (wire === 1) { yield [field, wire, i]; i += 8; }
      else if (wire === 5) { yield [field, wire, i]; i += 4; }
      else if (wire === 2) { const n = varint(); yield [field, wire, [i, i + n]]; i += n; }
      else throw new Error(`onnx: unsupported wire type ${wire}`);
    }
  }
  const floats = (a, b) => new Float32Array(bytes.slice(a, b).buffer);
  const out = new Map();
  for (const [f, w, graph] of fields(bytes, 0, bytes.length)) {
    if (f !== 7 || w !== 2) continue;
    for (const [g, gw, tensor] of fields(bytes, ...graph)) {
      if (g !== 5 || gw !== 2) continue;
      let name = '', data = null;
      for (const [t, tw, v] of fields(bytes, ...tensor)) {
        if (t === 8) name = new TextDecoder().decode(bytes.subarray(...v));
        else if (t === 9) data = floats(...v);
        else if (t === 4 && tw === 2) data = floats(...v);
      }
      out.set(name, data);
    }
  }
  return out;
}

// The velstand network: normalize, then 61 -> 512 -> 256 -> 128 -> 14 with ELU.
export class Policy {
  constructor(init) {
    this.mean = init.get('obs_normalizer._mean');
    // The normalizer's divisor is the other [1, 61] initializer.
    for (const [name, v] of init) {
      if (v.length === this.mean.length && name !== 'obs_normalizer._mean') this.std = v;
    }
    this.layers = [0, 2, 4, 6].map((i) => {
      const w = init.get(`mlp.${i}.weight`), b = init.get(`mlp.${i}.bias`);
      return { w, b, n: b.length, k: w.length / b.length };
    });
    if (!this.mean || !this.std || this.layers.some((l) => !l.w)) throw new Error('unexpected policy file');
  }

  run(obs) {
    let x = new Float32Array(obs.length);
    for (let i = 0; i < obs.length; i++) x[i] = (obs[i] - this.mean[i]) / this.std[i];
    this.layers.forEach(({ w, b, n, k }, li) => {
      const y = new Float32Array(n);
      for (let o = 0; o < n; o++) {
        let s = b[o];
        const row = o * k;
        for (let j = 0; j < k; j++) s += w[row + j] * x[j];
        y[o] = li < 3 && s < 0 ? Math.expm1(s) : s;  // ELU, not on the output
      }
      x = y;
    });
    return x;
  }
}

// -- tracker -------------------------------------------------------------------

// microduck.Tracker: position setpoint -> [forward, sideways, turn] command.
export class Tracker {
  constructor(o = {}) {
    Object.assign(this, {
      kPos: 1.0, kYaw: 1.5, speedGain: 0.69, speedOffset: 0.09, minWalk: 0.3, maxWalk: 0.5,
      maxTurnWalking: 1.0, turnInPlace: 1.5, turnInPlaceAbove: (50 * Math.PI) / 180,
      arrive: 0.08, waitAhead: 0.3,
    }, o);
  }

  twist(spPos, spVel, pos, yaw) {
    const ex = spPos[0] - pos[0], ey = spPos[1] - pos[1];
    const dist = Math.hypot(ex, ey), spSpeed = Math.hypot(spVel[0], spVel[1]);
    if (dist < this.arrive && spSpeed < 0.02) return [0, 0, 0];  // at a resting setpoint
    if (spSpeed > 0.01 && dist < this.waitAhead) {
      const px = spVel[0] / spSpeed, py = spVel[1] / spSpeed;
      const along = ex * px + ey * py, across = Math.abs(ex * py - ey * px);
      if (along < 0 && across < this.arrive) return [0, 0, 0];  // ahead of the setpoint: wait
    }
    const vx = spVel[0] + this.kPos * ex, vy = spVel[1] + this.kPos * ey;
    const speed = Math.hypot(vx, vy);
    let turn = Math.atan2(vy, vx) - yaw;
    turn -= 2 * Math.PI * Math.round(turn / (2 * Math.PI));
    if (Math.abs(turn) > this.turnInPlaceAbove) return [0, 0, Math.sign(turn) * this.turnInPlace];
    const clip = (v, a, b) => Math.min(b, Math.max(a, v));
    return [clip((speed + this.speedOffset) / this.speedGain, this.minWalk, this.maxWalk), 0,
            clip(this.kYaw * turn, -this.maxTurnWalking, this.maxTurnWalking)];
  }
}

// -- downloads -------------------------------------------------------------------

async function fetchBytes(url) {
  const r = await fetch(url);
  if (!r.ok) throw new Error(`${url}: HTTP ${r.status}`);
  return new Uint8Array(await r.arrayBuffer());
}

// Downloads the meshes the MJCF names and the policy; progress(done, total).
export async function fetchAssets(xml, progress = () => {}) {
  const names = [...new Set([...xml.matchAll(/file="([^"]+\.stl)"/g)].map((m) => m[1]))];
  const total = names.length + 1;
  let done = 0;
  const tick = (x) => { progress(++done, total); return x; };
  const [policy, ...meshes] = await Promise.all([
    fetchBytes(POLICY_URL).then(tick),
    ...names.map((n) => fetchBytes(MESH_BASE + n).then(tick)),
  ]);
  const digest = [...new Uint8Array(await crypto.subtle.digest('SHA-256', policy))]
    .map((b) => b.toString(16).padStart(2, '0')).join('');
  if (digest !== POLICY_SHA256) throw new Error('velstand.onnx: unexpected checksum');
  return { meshes: new Map(names.map((n, i) => [n, meshes[i]])), policy };
}

// -- simulation ------------------------------------------------------------------

// Owns the MuJoCo model/data and runs the mission loop: stand, replay the
// planner's setpoints, stand, then start over. Same interface as Flight.
export class Walk {
  constructor(mujoco, xml, assets, setpoints, { stand = 1.0, hold = 3.0 } = {}) {
    this.mj = mujoco;
    const vfs = new mujoco.MjVFS();
    for (const [name, bytes] of assets.meshes) vfs.addBuffer(name, bytes);
    try {
      this.model = mujoco.MjModel.from_xml_string(xml, vfs);
    } finally {
      vfs.delete();
    }
    this.data = new mujoco.MjData(this.model);
    this.policy = new Policy(onnxInitializers(assets.policy));
    this.tracker = new Tracker();
    this.rows = setpoints.rows;
    this.standTime = stand;
    this.duration = stand + this.rows[this.rows.length - 1][0] + hold;

    const m = this.model;
    this.trunk = m.body(`${PREFIX}trunk_base`).id;
    this.mocap = m.body_mocapid[m.body('setpoint').id];
    this.ground = m.geom('ground').id;
    this.gyro = m.sensor_adr[m.sensor(`${PREFIX}imu_ang_vel`).id];
    this.act = JOINTS.map((j) => m.actuator(PREFIX + j).id);
    this.jnt = this.act.map((a) => m.actuator_trnid[2 * a]);
    this.qadr = this.jnt.map((j) => m.jnt_qposadr[j]);
    this.dadr = this.jnt.map((j) => m.jnt_dofadr[j]);
    // BAM matches friction constraints by joint index (MujocoController); kept as is.
    this.byJoint = new Map(this.jnt.map((j, i) => [j, i]));
    const root = m.body_rootid[this.trunk];
    this.robotGeoms = new Set();
    for (let g = 0; g < m.ngeom; g++) if (m.body_rootid[m.geom_bodyid[g]] === root) this.robotGeoms.add(g);
    this.reset();
  }

  reset() {
    const { mj, model, data } = this;
    mj.mj_resetData(model, data);
    const q = data.qpos;
    q[2] = STAND_HEIGHT;
    this.qadr.forEach((a, i) => { q[a] = DEFAULT_POSE[i]; });
    this.target = Float64Array.from(DEFAULT_POSE);
    this.last = new Float32Array(JOINTS.length);
    this.prevTorque = new Float64Array(JOINTS.length);
    this.twist = [0, 0, 0];
    this.steps = 0;
    this.contacts = 0;
    this.fallen = false;
    this.laps = (this.laps ?? -1) + 1;
    this.setMarker(this.command().pos);
    mj.mj_forward(model, data);
  }

  get time() {
    return this.data.time;
  }

  command() {
    return setpointAt(this.rows, this.data.time - this.standTime);
  }

  setMarker(p) {
    const mp = this.data.mocap_pos;
    for (let i = 0; i < 3; i++) mp[3 * this.mocap + i] = p[i];
  }

  observation() {
    const d = this.data, q = d.qpos, v = d.qvel, x = d.xmat, s = d.sensordata;
    const o = new Float32Array(61);
    const r = 9 * this.trunk;
    // Gravity in the trunk frame: R^T (0, 0, -1).
    o.set([s[this.gyro], s[this.gyro + 1], s[this.gyro + 2], -x[r + 6], -x[r + 7], -x[r + 8]], 0);
    for (let i = 0; i < JOINTS.length; i++) {
      o[6 + i] = q[this.qadr[i]] - DEFAULT_POSE[i];
      o[20 + i] = v[this.dadr[i]];
    }
    o.set(this.last, 34);
    o.set(this.twist, 48);  // then head and body pose commands, zero
    return o;
  }

  // BAM MujocoController.update: motor torques, then friction for the next step.
  servos() {
    const { model, data } = this, b = BAM;
    const q = data.qpos, dq = data.qvel, ctrl = data.ctrl;
    let load = 0;
    for (const t of this.prevTorque) load += Math.abs(t);
    const vin = Math.max(b.vin - b.vinDropGain * load, b.vinMin);
    for (let i = 0; i < JOINTS.length; i++) {
      const duty = Math.min(b.maxPwm, Math.max(-b.maxPwm, (this.target[i] - q[this.qadr[i]]) * b.kp * b.errorGain));
      const torque = (b.kt * vin * duty) / b.R - (b.kt * b.kt * dq[this.dadr[i]]) / b.R;
      ctrl[this.act[i]] = torque;
      this.prevTorque[i] = torque;
    }

    const friction = new Float64Array(JOINTS.length);
    const n = data.nefc, id = data.efc_id, type = data.efc_type, force = data.efc_force;
    for (let k = 0; k < n; k++) {
      if (type[k] !== FRICTION_DOF) continue;
      const i = this.byJoint.get(id[k]);
      if (i !== undefined) friction[i] += force[k];
    }
    const bias = data.qfrc_bias, con = data.qfrc_constraint, act = data.qfrc_actuator;
    const loss = model.dof_frictionloss, damping = model.dof_damping;
    for (let i = 0; i < JOINTS.length; i++) {
      const d = this.dadr[i];
      [loss[d], damping[d]] = frictions(act[d], -bias[d] + con[d] - friction[i], dq[d]);
    }
  }

  step() {
    const { mj, model, data } = this;
    if (data.time >= this.duration) this.reset();
    if (this.steps % DECIMATION === 0) {
      const cmd = this.command();
      const q = data.qpos;
      this.twist = this.tracker.twist(cmd.pos, cmd.vel, [q[0], q[1]], 2 * Math.atan2(q[6], q[3]));
      this.setMarker(cmd.pos);
      this.last = this.policy.run(this.observation());
      for (let i = 0; i < JOINTS.length; i++) this.target[i] = DEFAULT_POSE[i] + this.last[i];
    }
    this.steps += 1;
    this.servos();
    mj.mj_step(model, data);
    if (data.qpos[2] < 0.6 * STAND_HEIGHT) this.fallen = true;
  }

  advance(seconds) {
    const n = Math.round(seconds / this.model.opt.timestep);
    for (let i = 0; i < n; i++) this.step();
  }

  // True if the duck touches anything but the floor (checked on demand).
  touching() {
    const d = this.data, contacts = d.contact;
    let hit = false;
    for (let c = 0; c < d.ncon && !hit; c++) {
      const con = contacts.get(c);
      const a = this.robotGeoms.has(con.geom1), b = this.robotGeoms.has(con.geom2);
      if (a !== b && con.geom1 !== this.ground && con.geom2 !== this.ground) hit = true;
      con.delete();
    }
    return hit;
  }

  position() {
    const q = this.data.qpos;
    return [q[0], q[1], q[2]];
  }
}
