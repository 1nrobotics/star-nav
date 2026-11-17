// Live MuJoCo scene for the project page: MuJoCo 4 (WebAssembly) simulates
// the quadrotor in real time, three.js draws it, the mouse moves the camera.
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { Flight } from './flight.js';

const MUJOCO_URL = 'https://cdn.jsdelivr.net/npm/@mujoco/mujoco@4.0.1/mujoco.js';
const GEOM = { PLANE: 0, SPHERE: 2, CAPSULE: 3, ELLIPSOID: 4, CYLINDER: 5, BOX: 6 };
const TRAIL = 4000;  // points of flown path kept

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

function setPose(mesh, data, i) {
  const p = data.geom_xpos, r = data.geom_xmat, o = 9 * i;
  mesh.matrix.set(r[o], r[o + 1], r[o + 2], p[3 * i],
                  r[o + 3], r[o + 4], r[o + 5], p[3 * i + 1],
                  r[o + 6], r[o + 7], r[o + 8], p[3 * i + 2],
                  0, 0, 0, 1);
  mesh.matrixWorldNeedsUpdate = true;
}

function buildMeshes(model, data, scene) {
  const dynamic = [];
  const types = model.geom_type, sizes = model.geom_size, rgba = model.geom_rgba;
  const matid = model.geom_matid, matRgba = model.mat_rgba, body = model.geom_bodyid;
  for (let i = 0; i < model.ngeom; i++) {
    const m = matid[i];
    const c = m >= 0 ? matRgba.slice(4 * m, 4 * m + 4) : rgba.slice(4 * i, 4 * i + 4);
    if (types[i] === GEOM.PLANE) {
      const ground = new THREE.Mesh(new THREE.PlaneGeometry(60, 60),
                                    new THREE.MeshStandardMaterial({ map: checkerTexture(), roughness: 1 }));
      ground.receiveShadow = true;
      scene.add(ground);
      continue;
    }
    const geo = geometryFor(types[i], sizes.slice(3 * i, 3 * i + 3));
    if (!geo || c[3] === 0) continue;
    const mat = new THREE.MeshStandardMaterial({
      color: new THREE.Color(c[0], c[1], c[2]).convertSRGBToLinear(),
      transparent: c[3] < 1, opacity: c[3], roughness: 0.7, metalness: 0.05,
    });
    const mesh = new THREE.Mesh(geo, mat);
    mesh.matrixAutoUpdate = false;
    mesh.castShadow = true;
    mesh.receiveShadow = c[3] === 1;
    setPose(mesh, data, i);
    scene.add(mesh);
    if (body[i] !== 0) dynamic.push([mesh, i]);
  }
  return dynamic;
}

function pathLine(points, color, opacity) {
  const geo = new THREE.BufferGeometry().setFromPoints(points.map((p) => new THREE.Vector3(...p)));
  return new THREE.Line(geo, new THREE.LineBasicMaterial({ color, transparent: opacity < 1, opacity }));
}

// container: element the canvas goes into; ui: {status, time, play, follow, restart}.
export async function start(container, ui) {
  const say = (s) => { if (ui.status) ui.status.textContent = s; };
  say('Loading MuJoCo (WebAssembly, ~11 MB)…');
  const base = new URL('.', import.meta.url);
  const [{ default: loadMujoco }, xml, setpoints] = await Promise.all([
    import(MUJOCO_URL),
    fetch(new URL('forest.xml', base)).then((r) => r.text()),
    fetch(new URL('setpoints.json', base)).then((r) => r.json()),
  ]);
  const mujoco = await loadMujoco();
  const flight = new Flight(mujoco, xml, setpoints);
  const { model, data } = flight;

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

  const camera = new THREE.PerspectiveCamera(45, 1, 0.02, 200);
  camera.up.set(0, 0, 1);
  // Start behind the drone; the forest is visible beyond it.
  camera.position.set(-3.2, -4.4, 2.9);
  const controls = new OrbitControls(camera, renderer.domElement);
  controls.target.set(0, 0, 1.0);
  controls.enableDamping = true;
  controls.maxPolarAngle = Math.PI * 0.495;
  controls.minDistance = 0.6;
  controls.maxDistance = 40;
  controls.update();

  const dynamic = buildMeshes(model, data, scene);
  scene.add(pathLine(setpoints.rows.map((r) => [r[1], r[2], r[3]]), 0x2563eb, 0.45));
  const trailPos = new Float32Array(3 * TRAIL);
  const trailGeo = new THREE.BufferGeometry();
  trailGeo.setAttribute('position', new THREE.BufferAttribute(trailPos, 3));
  trailGeo.setDrawRange(0, 0);
  scene.add(new THREE.Line(trailGeo, new THREE.LineBasicMaterial({ color: 0xea580c })));
  let trailN = 0, lap = flight.laps, nextTrail = 0;

  const resize = () => {
    const w = container.clientWidth, h = container.clientHeight;
    renderer.setSize(w, h, false);
    camera.aspect = w / h;
    camera.updateProjectionMatrix();
  };
  new ResizeObserver(resize).observe(container);
  resize();

  let paused = false, follow = true;
  const last = new THREE.Vector3(...flight.position());
  ui.play?.addEventListener('click', () => {
    paused = !paused;
    ui.play.textContent = paused ? 'Play' : 'Pause';
  });
  ui.follow?.addEventListener('click', () => {
    follow = !follow;
    ui.follow.setAttribute('aria-pressed', String(follow));
  });
  ui.restart?.addEventListener('click', () => flight.reset());
  say('');

  const clock = new THREE.Clock();
  renderer.setAnimationLoop(() => {
    const dt = Math.min(clock.getDelta(), 1 / 20);
    if (!paused) flight.advance(dt);
    if (flight.laps !== lap) { lap = flight.laps; trailN = 0; nextTrail = 0; }
    for (const [mesh, i] of dynamic) setPose(mesh, data, i);

    const p = new THREE.Vector3(...flight.position());
    if (data.time >= nextTrail && trailN < TRAIL) {
      trailPos.set([p.x, p.y, p.z], 3 * trailN++);
      trailGeo.setDrawRange(0, trailN);
      trailGeo.attributes.position.needsUpdate = true;
      nextTrail = data.time + 0.05;
    }
    if (follow) {
      // Keep the user's viewing angle and distance, move with the drone.
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
