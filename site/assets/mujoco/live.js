// Live MuJoCo scene for the project page: MuJoCo 4 (WebAssembly) simulates
// the quadrotor or the Microduck in real time, three.js draws it, the mouse
// moves the camera.
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { Flight } from './flight.js';

const MUJOCO_URL = 'https://cdn.jsdelivr.net/npm/@mujoco/mujoco@4.0.1/mujoco.js';
const GEOM = { PLANE: 0, SPHERE: 2, CAPSULE: 3, ELLIPSOID: 4, CYLINDER: 5, BOX: 6, MESH: 7 };
const HIDDEN_GROUP = 3;  // collision geoms; groups below are drawn
const TRAIL = 4000;  // points of the travelled path kept
const SPEEDS = [1, 2, 4];

const base = new URL('.', import.meta.url);
const getText = (name) => fetch(new URL(name, base)).then((r) => r.text());
const getJson = (name) => fetch(new URL(name, base)).then((r) => r.json());

const ROBOTS = {
  drone: {
    follow: 'Follow drone',
    camera: [-3.2, -4.4, 2.9], target: [0, 0, 1.0], minDistance: 0.6,
    trailEvery: 0.05, speed: 1,
    async load(mujoco) {
      const [xml, setpoints] = await Promise.all([getText('forest.xml'), getJson('setpoints.json')]);
      return new Flight(mujoco, xml, setpoints);
    },
  },
  microduck: {
    follow: 'Follow duck',
    camera: [-0.9, -1.25, 0.75], target: [0, 0, 0.12], minDistance: 0.15,
    trailEvery: 0.2, speed: 2,
    async load(mujoco, say) {
      const walk = await import('./walk.js');
      const [xml, setpoints] = await Promise.all([getText('microduck.xml'), getJson('microduck_setpoints.json')]);
      const assets = await walk.fetchAssets(xml, (done, total) =>
        say(`Downloading the Microduck model and policy from Pollen Robotics (${done}/${total})…`));
      return new walk.Walk(mujoco, xml, assets, setpoints);
    },
  },
};

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

function meshGeometry(model, id) {
  const va = model.mesh_vertadr[id], vn = model.mesh_vertnum[id];
  const fa = model.mesh_faceadr[id], fn = model.mesh_facenum[id];
  const geo = new THREE.BufferGeometry();
  geo.setAttribute('position', new THREE.BufferAttribute(model.mesh_vert.slice(3 * va, 3 * (va + vn)), 3));
  geo.setIndex(new THREE.BufferAttribute(model.mesh_face.slice(3 * fa, 3 * (fa + fn)), 1));
  const flat = geo.toNonIndexed();  // faceted shading: CAD meshes have sharp edges
  flat.computeVertexNormals();
  return flat;
}

function geometryFor(model, i, s) {
  switch (model.geom_type[i]) {
    case GEOM.SPHERE: return new THREE.SphereGeometry(s[0], 24, 16);
    case GEOM.ELLIPSOID: return new THREE.SphereGeometry(1, 24, 16).scale(s[0], s[1], s[2]);
    case GEOM.BOX: return new THREE.BoxGeometry(2 * s[0], 2 * s[1], 2 * s[2]);
    // three.js builds these along +y, MuJoCo along +z.
    case GEOM.CYLINDER: return new THREE.CylinderGeometry(s[0], s[0], 2 * s[1], 24).rotateX(Math.PI / 2);
    case GEOM.CAPSULE: return new THREE.CapsuleGeometry(s[0], 2 * s[1], 6, 12).rotateX(Math.PI / 2);
    case GEOM.MESH: return meshGeometry(model, model.geom_dataid[i]);
    default: return null;
  }
}

function setPose(mesh, data, i) {
  const p = data.geom_xpos, r = data.geom_xmat, o = 9 * i;
  mesh.matrix.set(r[o], r[o + 1], r[o + 2], p[3 * i],
                  r[o + 3], r[o + 4], r[o + 5], p[3 * i + 1],
                  r[o + 6], r[o + 7], r[o + 8], p[3 * i + 2],
                  0, 0, 0, 1);
  mesh.matrixWorldNeedsUpdate = true;
}

function buildMeshes(model, data, group) {
  const dynamic = [];
  const sizes = model.geom_size, rgba = model.geom_rgba, groups = model.geom_group;
  const matid = model.geom_matid, matRgba = model.mat_rgba, body = model.geom_bodyid;
  for (let i = 0; i < model.ngeom; i++) {
    const m = matid[i];
    const c = m >= 0 ? matRgba.slice(4 * m, 4 * m + 4) : rgba.slice(4 * i, 4 * i + 4);
    if (model.geom_type[i] === GEOM.PLANE) {
      const ground = new THREE.Mesh(new THREE.PlaneGeometry(60, 60),
                                    new THREE.MeshStandardMaterial({ map: checkerTexture(), roughness: 1 }));
      ground.receiveShadow = true;
      group.add(ground);
      continue;
    }
    if (groups[i] >= HIDDEN_GROUP || c[3] === 0) continue;
    const geo = geometryFor(model, i, sizes.slice(3 * i, 3 * i + 3));
    if (!geo) continue;
    const mat = new THREE.MeshStandardMaterial({
      color: new THREE.Color(c[0], c[1], c[2]).convertSRGBToLinear(),
      transparent: c[3] < 1, opacity: c[3], roughness: 0.7, metalness: 0.05,
    });
    const mesh = new THREE.Mesh(geo, mat);
    mesh.matrixAutoUpdate = false;
    mesh.castShadow = true;
    mesh.receiveShadow = c[3] === 1;
    setPose(mesh, data, i);
    group.add(mesh);
    if (body[i] !== 0) dynamic.push([mesh, i]);
  }
  return dynamic;
}

function pathLine(points, color, opacity) {
  const geo = new THREE.BufferGeometry().setFromPoints(points.map((p) => new THREE.Vector3(...p)));
  return new THREE.Line(geo, new THREE.LineBasicMaterial({ color, transparent: opacity < 1, opacity }));
}

function dispose(group) {
  group.traverse((o) => {
    o.geometry?.dispose();
    o.material?.map?.dispose();
    o.material?.dispose();
  });
  group.clear();
}

// container: element the canvas goes into; ui: {status, time, play, follow,
// restart, speed, robots: buttons with data-robot}.
export async function start(container, ui) {
  const say = (s) => { if (ui.status) ui.status.textContent = s; };
  say('Loading MuJoCo (WebAssembly, ~11 MB)…');
  const { default: loadMujoco } = await import(MUJOCO_URL);
  const mujoco = await loadMujoco();

  const renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFSoftShadowMap;
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
  scene.add(sun);
  const world = new THREE.Group();
  scene.add(world);

  const camera = new THREE.PerspectiveCamera(45, 1, 0.02, 200);
  camera.up.set(0, 0, 1);
  const controls = new OrbitControls(camera, renderer.domElement);
  controls.enableDamping = true;
  controls.maxPolarAngle = Math.PI * 0.495;
  controls.maxDistance = 40;

  const resize = () => {
    const w = container.clientWidth, h = container.clientHeight;
    renderer.setSize(w, h, false);
    camera.aspect = w / h;
    camera.updateProjectionMatrix();
  };
  new ResizeObserver(resize).observe(container);
  resize();

  // The robot on stage; replaced by show().
  let sim = null, robot = null, dynamic = [], trail = null;
  let paused = false, follow = true, speed = 1;
  const last = new THREE.Vector3();
  const setSpeed = (s) => {
    speed = s;
    if (ui.speed) ui.speed.textContent = `${s}×`;
  };

  let loading = null;
  async function show(name) {
    if (loading || name === robot) return;
    const spec = ROBOTS[name];
    ui.robots?.forEach((b) => b.setAttribute('aria-pressed', String(b.dataset.robot === name)));
    loading = name;
    try {
      say(name === 'drone' ? 'Loading the forest…' : 'Loading the Microduck…');
      const next = await spec.load(mujoco, say);
      if (sim) { sim.data.delete(); sim.model.delete(); }
      dispose(world);
      sim = next;
      robot = name;
      dynamic = buildMeshes(sim.model, sim.data, world);
      world.add(pathLine(sim.rows.map((r) => [r[1], r[2], r[3]]), 0x2563eb, 0.45));
      const pos = new Float32Array(3 * TRAIL);
      const geo = new THREE.BufferGeometry();
      geo.setAttribute('position', new THREE.BufferAttribute(pos, 3));
      geo.setDrawRange(0, 0);
      world.add(new THREE.Line(geo, new THREE.LineBasicMaterial({ color: 0xea580c })));
      trail = { pos, geo, n: 0, next: 0, lap: sim.laps, every: spec.trailEvery };

      camera.position.set(...spec.camera);
      controls.target.set(...spec.target);
      controls.minDistance = spec.minDistance;
      controls.update();
      last.set(...sim.position());
      if (ui.follow) ui.follow.textContent = spec.follow;
      setSpeed(spec.speed);
      say('');
    } catch (e) {
      console.error(e);
      ui.robots?.forEach((b) => b.setAttribute('aria-pressed', String(b.dataset.robot === robot)));
      say(`Could not load the ${name} (${e.message}).`);
      if (sim) setTimeout(() => say(''), 6000);  // keep showing the previous robot
    } finally {
      loading = null;
    }
  }

  ui.play?.addEventListener('click', () => {
    paused = !paused;
    ui.play.textContent = paused ? 'Play' : 'Pause';
  });
  ui.follow?.addEventListener('click', () => {
    follow = !follow;
    ui.follow.setAttribute('aria-pressed', String(follow));
  });
  ui.restart?.addEventListener('click', () => sim?.reset());
  ui.speed?.addEventListener('click', () => setSpeed(SPEEDS[(SPEEDS.indexOf(speed) + 1) % SPEEDS.length]));
  ui.robots?.forEach((b) => b.addEventListener('click', () => show(b.dataset.robot)));

  await show('drone');

  const clock = new THREE.Clock();
  renderer.setAnimationLoop(() => {
    const dt = Math.min(clock.getDelta(), 1 / 20);
    if (!sim) return;
    const { data } = sim;
    if (!paused) sim.advance(dt * speed);
    if (sim.laps !== trail.lap) Object.assign(trail, { lap: sim.laps, n: 0, next: 0 });
    for (const [mesh, i] of dynamic) setPose(mesh, data, i);

    const p = new THREE.Vector3(...sim.position());
    if (data.time >= trail.next && trail.n < TRAIL) {
      trail.pos.set([p.x, p.y, p.z], 3 * trail.n++);
      trail.geo.setDrawRange(0, trail.n);
      trail.geo.attributes.position.needsUpdate = true;
      trail.next = data.time + trail.every;
    }
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
      ui.time.textContent = `t = ${data.time.toFixed(1)} s · ${Math.hypot(v[0], v[1], v[2]).toFixed(2)} m/s`;
    }
  });
}
