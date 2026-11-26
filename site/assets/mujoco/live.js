// Live MuJoCo scene for the project pages: MuJoCo 4 (WebAssembly) simulates
// a robot in real time, three.js draws it, the mouse moves the camera.
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { Flight } from './flight.js';
import { load as loadWalk } from './walk.js';

const MUJOCO_URL = 'https://cdn.jsdelivr.net/npm/@mujoco/mujoco@4.0.1/mujoco.js';
const GEOM = { PLANE: 0, SPHERE: 2, CAPSULE: 3, ELLIPSOID: 4, CYLINDER: 5, BOX: 6, MESH: 7 };
const TRAIL = 6000;  // points of the path kept
const WALL_BASE = 0.12;  // m of a wall drawn solid; the rest is see-through

// What differs between the robots: the camera's start (behind the robot,
// looking at `look` above the ground), the speeds of "Drive", the texts.
export const ROBOTS = {
  lite3: {
    walks: true, camera: [-2.3, -2.9, 2.0], look: 0.25, near: 0.5,
    speed: { forward: 0.6, side: 0.3, turn: 0.8 },
    note: 'Blue: the path of the recorded mission. Orange: the path walked in your browser.',
  },
  t1: {
    walks: true, camera: [-2.8, -3.6, 2.7], look: 0.6, near: 0.8,
    speed: { forward: 0.5, side: 0.25, turn: 0.6 },
    note: 'Blue: the path of the recorded mission. Orange: the path walked in your browser. ' +
          'When the T1 falls, as it sometimes does on the steps, the mission starts again.',
  },
  quadrotor: {
    walks: false, camera: [-3.2, -4.4, 2.9], look: 1.0, near: 0.6,
    note: 'Blue: the planned path. Orange: the path flown in your browser.',
  },
};

let mujocoModule;  // one WebAssembly instance for every scene of the page

function checkerTexture() {
  const c = document.createElement('canvas');
  c.width = c.height = 64;
  const g = c.getContext('2d');
  g.fillStyle = '#c7ccc7'; g.fillRect(0, 0, 64, 64);
  g.fillStyle = '#b3b8b3'; g.fillRect(0, 0, 32, 32); g.fillRect(32, 32, 32, 32);
  const t = new THREE.CanvasTexture(c);
  t.wrapS = t.wrapT = THREE.RepeatWrapping;
  t.repeat.set(30, 30);
  t.magFilter = THREE.NearestFilter;
  t.colorSpace = THREE.SRGBColorSpace;
  return t;
}

function geometryFor(type, s) {
  switch (type) {
    case GEOM.SPHERE: return new THREE.SphereGeometry(s[0], 24, 16);
    case GEOM.ELLIPSOID: return new THREE.SphereGeometry(1, 24, 16).scale(s[0], s[1], s[2]);
    case GEOM.BOX: return new THREE.BoxGeometry(2 * s[0], 2 * s[1], 2 * s[2]);
    // three.js builds these along +y, MuJoCo along +z.
    case GEOM.CYLINDER: return new THREE.CylinderGeometry(s[0], s[0], 2 * s[1], 24).rotateX(Math.PI / 2);
    case GEOM.CAPSULE: return new THREE.CapsuleGeometry(s[0], 2 * s[1], 6, 12).rotateX(Math.PI / 2);
    default: return null;
  }
}

// The triangles of a mesh as MuJoCo compiled it (in the geom's frame).
function meshGeometry(model, id) {
  const v = model.mesh_vertadr[id], f = model.mesh_faceadr[id];
  const geo = new THREE.BufferGeometry();
  geo.setAttribute('position', new THREE.BufferAttribute(
      Float32Array.from(model.mesh_vert.subarray(3 * v, 3 * (v + model.mesh_vertnum[id]))), 3));
  geo.setIndex(new THREE.BufferAttribute(
      Uint32Array.from(model.mesh_face.subarray(3 * f, 3 * (f + model.mesh_facenum[id]))), 1));
  return geo;
}

function setPose(mesh, data, i) {
  const p = data.geom_xpos, r = data.geom_xmat, o = 9 * i;
  mesh.matrix.set(r[o], r[o + 1], r[o + 2], p[3 * i],
                  r[o + 3], r[o + 4], r[o + 5], p[3 * i + 1],
                  r[o + 6], r[o + 7], r[o + 8], p[3 * i + 2],
                  0, 0, 0, 1);
  mesh.matrixWorldNeedsUpdate = true;
}

function material(c, extra = {}) {
  return new THREE.MeshStandardMaterial({
    color: new THREE.Color(c[0], c[1], c[2]).convertSRGBToLinear(),
    transparent: c[3] < 1, opacity: c[3], roughness: 0.7, metalness: 0.05, ...extra,
  });
}

// A walking robot's colour per part (<robot>.json), by its visual geom's
// name: ".../FL_HIP_visual" is a hip.
function partColor(appearance, name) {
  if (!appearance || !name.endsWith('_visual')) return null;
  return appearance[name.slice(0, -'_visual'.length).split('_').pop().toLowerCase()] ?? null;
}

// One three.js mesh per MuJoCo geom that is drawn. Of a walking robot only
// the visual geoms are (its collision shapes are not); a wall is see-through
// above its base, so that the robot shows behind it.
function buildMeshes(model, data, scene, { walks, appearance }) {
  const dynamic = [], meshes = new Map();
  const types = model.geom_type, sizes = model.geom_size, rgba = model.geom_rgba;
  const matid = model.geom_matid, matRgba = model.mat_rgba, body = model.geom_bodyid;
  const contype = model.geom_contype, conaffinity = model.geom_conaffinity, dataid = model.geom_dataid;
  const walls = [];
  for (let i = 0; i < model.nmat; i++) if (model.mat(i).name === 'wall') walls.push(i);
  for (let i = 0; i < model.ngeom; i++) {
    const m = matid[i], robot = body[i] !== 0;
    if (types[i] === GEOM.PLANE) {
      // Behind what lies flat on it (the house's floor is 2 mm thick).
      const ground = new THREE.Mesh(new THREE.PlaneGeometry(60, 60), new THREE.MeshStandardMaterial({
        map: checkerTexture(), roughness: 1, polygonOffset: true, polygonOffsetFactor: 2, polygonOffsetUnits: 8 }));
      ground.receiveShadow = true;
      scene.add(ground);
      continue;
    }
    if (walks && robot && (contype[i] || conaffinity[i])) continue;
    const c = (walks && robot && partColor(appearance, model.geom(i).name)) ||
        (m >= 0 ? matRgba.slice(4 * m, 4 * m + 4) : rgba.slice(4 * i, 4 * i + 4));
    if (c[3] === 0) continue;
    const size = sizes.slice(3 * i, 3 * i + 3);
    let geo;
    if (types[i] === GEOM.MESH) {
      if (!meshes.has(dataid[i])) meshes.set(dataid[i], meshGeometry(model, dataid[i]));
      geo = meshes.get(dataid[i]);
    } else {
      geo = geometryFor(types[i], size);
    }
    if (!geo) continue;
    const wall = walls.includes(m) && types[i] === GEOM.BOX && size[2] > WALL_BASE;
    const mesh = new THREE.Mesh(geo, wall
      ? material([c[0], c[1], c[2], 0.22], { depthWrite: false })
      : material(c, { flatShading: types[i] === GEOM.MESH }));
    mesh.matrixAutoUpdate = false;
    mesh.castShadow = !wall && !(types[i] === GEOM.BOX && size[2] < 0.005);  // not the floor's sheet
    mesh.receiveShadow = c[3] === 1 && !wall;
    setPose(mesh, data, i);
    scene.add(mesh);
    if (wall) {
      const base = new THREE.Mesh(new THREE.BoxGeometry(2 * size[0], 2 * size[1], WALL_BASE), material(c));
      base.position.set(data.geom_xpos[3 * i], data.geom_xpos[3 * i + 1], WALL_BASE / 2);
      base.castShadow = base.receiveShadow = true;
      scene.add(base);
    }
    if (robot) dynamic.push([mesh, i]);
  }
  return dynamic;
}

// The path to follow; the path taken is drawn over it.
function pathLine(points, color, opacity) {
  const geo = new THREE.BufferGeometry().setFromPoints(points.map((p) => new THREE.Vector3(...p)));
  return new THREE.Line(geo, new THREE.LineBasicMaterial({ color, transparent: true, opacity, depthWrite: false }));
}

// The simulation of one robot, behind what the scene needs of it.
async function simulate(mujoco, name, base, say) {
  if (ROBOTS[name].walks) {
    const walk = await loadWalk(mujoco, name, {
      base, progress: (done, total) => say(`Loading the robot from its manufacturer's repository (${done} of ${total} files)…`),
    });
    const lift = 0.03 - walk.cfg.stand_height;  // the body's path, drawn on the ground
    return {
      sim: walk, appearance: walk.cfg.appearance,
      planned: walk.rows.map((r) => [r[1], r[2], r[3] + lift]),
      trailPoint: (p) => [p[0], p[1], Math.max(p[2] + lift, 0.03)],
      target: () => (walk.manual ? null : walk.target().pos),
    };
  }
  const fetchFrom = (file) => fetch(new URL(file, base));
  const [xml, setpoints] = await Promise.all([
    fetchFrom('forest.xml').then((r) => r.text()), fetchFrom('setpoints.json').then((r) => r.json())]);
  const flight = new Flight(mujoco, xml, setpoints);
  return {
    sim: flight,
    planned: setpoints.rows.map((r) => [r[1], r[2], r[3]]),
    trailPoint: (p) => p,
    target: () => flight.command().pos,
  };
}

// container: element the canvas goes into; ui: {status, time, play, follow,
// restart, drive, pad}; name: a key of ROBOTS. Returns {stop()}, which frees
// the scene so that another can start in the same container.
export async function start(container, ui, name = 'quadrotor') {
  const cfg = ROBOTS[name];
  const say = (s) => { if (ui.status) ui.status.textContent = s; };
  say('Loading MuJoCo (WebAssembly, ~11 MB)…');
  mujocoModule ??= import(MUJOCO_URL).then(({ default: loadMujoco }) => loadMujoco());
  const mujoco = await mujocoModule;
  const { sim, appearance, planned, trailPoint, target } =
      await simulate(mujoco, name, new URL('.', import.meta.url), say);
  const { model, data } = sim;

  const renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFSoftShadowMap;
  renderer.domElement.tabIndex = 0;  // takes the keys of "Drive"
  container.appendChild(renderer.domElement);

  const scene = new THREE.Scene();
  scene.background = new THREE.Color(0xdbe6f2);
  scene.fog = new THREE.Fog(0xdbe6f2, 18, 45);
  scene.add(new THREE.HemisphereLight(0xffffff, 0x8a9a8a, 1.6));
  const sun = new THREE.DirectionalLight(0xffffff, 2.2);
  sun.position.set(6, -8, 14);
  sun.castShadow = true;
  sun.shadow.mapSize.set(2048, 2048);
  Object.assign(sun.shadow.camera, { left: -12, right: 12, top: 12, bottom: -12, near: 1, far: 40 });
  sun.shadow.normalBias = 0.03;  // no stripes on the faces the sun grazes
  scene.add(sun);

  const camera = new THREE.PerspectiveCamera(45, 1, 0.05, 200);
  camera.up.set(0, 0, 1);
  camera.position.set(...cfg.camera);
  const controls = new OrbitControls(camera, renderer.domElement);
  controls.target.set(0, 0, cfg.look);
  controls.enableDamping = true;
  controls.maxPolarAngle = Math.PI * 0.495;
  controls.minDistance = cfg.near;
  controls.maxDistance = 40;
  controls.update();

  const dynamic = buildMeshes(model, data, scene, { walks: cfg.walks, appearance });
  scene.add(pathLine(planned, 0x2563eb, cfg.walks ? 0.8 : 0.45));
  const marker = new THREE.Mesh(new THREE.SphereGeometry(0.06, 16, 12), material([0.1, 0.6, 1, 0.6]));
  scene.add(marker);
  const trailPos = new Float32Array(3 * TRAIL);
  const trailGeo = new THREE.BufferGeometry();
  trailGeo.setAttribute('position', new THREE.BufferAttribute(trailPos, 3));
  trailGeo.setDrawRange(0, 0);
  const trail = new THREE.Line(trailGeo, new THREE.LineBasicMaterial({ color: 0xea580c, transparent: true }));
  trail.renderOrder = 1;
  trail.frustumCulled = false;  // its points change every frame
  scene.add(trail);
  let trailN = 0, lap = sim.laps, nextTrail = 0;

  const resize = () => {
    const w = container.clientWidth, h = container.clientHeight;
    renderer.setSize(w, h, false);
    camera.aspect = w / h;
    camera.updateProjectionMatrix();
  };
  const observer = new ResizeObserver(resize);
  observer.observe(container);
  resize();

  // Every listener goes with stop().
  const listeners = new AbortController();
  const on = (el, type, fn) => el?.addEventListener(type, fn, { signal: listeners.signal });
  let paused = false, follow = true;
  const last = new THREE.Vector3(...sim.position());
  if (ui.play) ui.play.textContent = 'Pause';
  ui.follow?.setAttribute('aria-pressed', 'true');
  on(ui.play, 'click', () => {
    paused = !paused;
    ui.play.textContent = paused ? 'Play' : 'Pause';
  });
  on(ui.follow, 'click', () => {
    follow = !follow;
    ui.follow.setAttribute('aria-pressed', String(follow));
  });
  on(ui.restart, 'click', () => sim.reset());

  // "Drive": the keys and the pad's buttons give the walking policy its
  // velocity command, in place of the recorded path.
  const held = new Set();
  let driving = false;
  const KEYS = { w: 'forward', arrowup: 'forward', s: 'back', arrowdown: 'back', a: 'left', arrowleft: 'left',
                 d: 'right', arrowright: 'right', q: 'strafe-left', e: 'strafe-right' };
  const command = () => {
    const h = (k) => (held.has(k) ? 1 : 0), v = cfg.speed;
    return [v.forward * (h('forward') - h('back')), v.side * (h('strafe-left') - h('strafe-right')),
            v.turn * (h('left') - h('right'))];
  };
  const setDriving = (value) => {
    driving = value;
    held.clear();
    sim.drive(driving ? [0, 0, 0] : null);
    ui.drive?.setAttribute('aria-pressed', String(driving));
    if (ui.pad) ui.pad.hidden = !driving;
    if (driving) renderer.domElement.focus({ preventScroll: true });
  };
  if (ui.drive) ui.drive.hidden = !cfg.walks;
  if (ui.pad) ui.pad.hidden = true;
  ui.drive?.setAttribute('aria-pressed', 'false');
  if (cfg.walks) {
    const hold = (key, down) => {
      if (!driving || !key) return;
      if (down) held.add(key); else held.delete(key);
      sim.drive(command());
    };
    on(ui.drive, 'click', () => setDriving(!driving));
    on(renderer.domElement, 'keydown', (e) => {
      const key = KEYS[e.key.toLowerCase()];
      if (driving && key) e.preventDefault();
      hold(key, true);
    });
    on(renderer.domElement, 'keyup', (e) => hold(KEYS[e.key.toLowerCase()], false));
    on(renderer.domElement, 'blur', () => { held.clear(); if (driving) sim.drive(command()); });
    ui.pad?.querySelectorAll('[data-key]').forEach((b) => {
      on(b, 'pointerdown', (e) => { e.preventDefault(); hold(b.dataset.key, true); });
      for (const type of ['pointerup', 'pointerleave', 'pointercancel']) on(b, type, () => hold(b.dataset.key, false));
    });
  }
  say('');

  const clock = new THREE.Clock();
  renderer.setAnimationLoop(() => {
    const dt = Math.min(clock.getDelta(), 1 / 20);
    if (!paused) sim.advance(dt);
    if (sim.laps !== lap) { lap = sim.laps; trailN = 0; nextTrail = 0; }
    for (const [mesh, i] of dynamic) setPose(mesh, data, i);

    const p = new THREE.Vector3(...sim.position());
    if (data.time >= nextTrail && trailN < TRAIL) {
      trailPos.set(trailPoint([p.x, p.y, p.z]), 3 * trailN++);
      trailGeo.setDrawRange(0, trailN);
      trailGeo.attributes.position.needsUpdate = true;
      nextTrail = data.time + 0.05;
    }
    const t = target();
    marker.visible = t !== null;
    if (t) marker.position.set(...t);
    if (follow) {
      // Keep the user's viewing angle and distance, move with the robot.
      const d = p.clone().sub(last);
      camera.position.add(d);
      controls.target.add(d);
    }
    last.copy(p);
    controls.update();
    renderer.render(scene, camera);
    if (ui.time) {
      const v = data.qvel;
      const fell = cfg.walks && sim.fallen !== null ? ' · fell, starting again' : '';
      ui.time.textContent = `t = ${data.time.toFixed(1)} s · ${Math.hypot(v[0], v[1], v[2]).toFixed(2)} m/s${fell}`;
    }
  });

  return {
    stop() {
      renderer.setAnimationLoop(null);
      listeners.abort();
      observer.disconnect();
      controls.dispose();
      scene.traverse((o) => {
        o.geometry?.dispose();
        o.material?.map?.dispose();
        o.material?.dispose();
      });
      renderer.dispose();
      renderer.forceContextLoss();
      renderer.domElement.remove();
      if (ui.time) ui.time.textContent = '';
      if (ui.pad) ui.pad.hidden = true;
      data.delete();
      model.delete();
    },
  };
}
