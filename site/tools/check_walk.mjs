// Checks the walking robots of the browser demo under Node: loads each one
// as the page does and walks the recorded mission once, in MuJoCo
// (WebAssembly), without drawing anything.
//
//   npm install --prefix /tmp/mujoco-web @mujoco/mujoco@4.0.1
//   node site/tools/check_walk.mjs /tmp/mujoco-web [lite3|t1 ...]
//
// The upstream files come from jsDelivr, as in the browser; with
// STAR_NAV_ASSETS set (e.g. ~/.cache/star_nav) the copies that
// "star-nav-mujoco --fetch" downloaded are read instead.
import fs from 'node:fs';
import path from 'node:path';
import { createRequire } from 'node:module';
import { fileURLToPath, pathToFileURL } from 'node:url';

const [prefix, ...names] = process.argv.slice(2);
if (!prefix) {
  console.error('usage: node site/tools/check_walk.mjs <npm prefix with @mujoco/mujoco> [robot ...]');
  process.exit(2);
}
const require = createRequire(path.resolve(prefix, 'package.json'));
const { default: loadMujoco } = await import(pathToFileURL(require.resolve('@mujoco/mujoco')).href);
const base = new URL('../assets/mujoco/', import.meta.url);
const { load } = await import(new URL('walk.js', base).href);

const cache = process.env.STAR_NAV_ASSETS;
// Where robots/<robot>.py keeps each upstream file.
const cached = (robot, url) => {
  const file = url.split('/').pop();
  if (robot === 'lite3') return file === 'policy.onnx' ? file : (file.endsWith('.xml') ? 'mjcf/' : 'meshes/') + file;
  return file === 'T1.pt' ? file : 'resources/T1/' + (file.endsWith('.xml') ? '' : 'meshes/') + file;
};

let failed = false;
for (const name of names.length ? names : ['lite3', 't1']) {
  const read = async (url) => {
    if (url.startsWith('file:')) return new Uint8Array(fs.readFileSync(fileURLToPath(url)));
    if (cache) return new Uint8Array(fs.readFileSync(path.join(cache, name, cached(name, url))));
    const r = await fetch(url);
    if (!r.ok) throw new Error(`${url}: HTTP ${r.status}`);
    return new Uint8Array(await r.arrayBuffer());
  };
  const mujoco = await loadMujoco();
  const walk = await load(mujoco, name, { base, read });
  const rows = walk.rows, end = rows[rows.length - 1];
  const started = performance.now();
  let worst = 0, peak = 0, waited = 0;
  // One mission: until the path's clock reaches its end, or a fall.
  while (walk.clock < end[0] && walk.fallen === null && walk.time < 3 * end[0]) {
    walk.step();
    const p = walk.position(), sp = walk.target();
    worst = Math.max(worst, Math.hypot(sp.pos[0] - p[0], sp.pos[1] - p[1]));
    peak = Math.max(peak, p[2]);
    if (walk.waiting) waited += walk.model.opt.timestep;
  }
  const p = walk.position();
  const off = Math.hypot(end[1] - p[0], end[2] - p[1]);
  const ok = walk.fallen === null && walk.clock >= end[0] && off < 0.3;
  failed ||= !ok;
  console.log(`${name}: ${ok ? 'PASS' : 'FAIL'}  path ${end[0].toFixed(1)} s, walked in ${walk.time.toFixed(1)} s ` +
              `(waited ${waited.toFixed(1)} s), ${walk.fallen === null ? 'no fall' : 'fell at ' + walk.fallen.toFixed(1) + ' s'}, ` +
              `farthest from the path ${worst.toFixed(2)} m, ${off.toFixed(2)} m from its end, ` +
              `highest ${peak.toFixed(2)} m, ${(walk.time / ((performance.now() - started) / 1000)).toFixed(1)}x real time`);
}
process.exit(failed ? 1 : 0);
