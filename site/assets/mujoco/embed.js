// Starts the live MuJoCo scene of a page. Each <div class="live"
// data-live="lite3 | t1 | quadrotor"> gets its scene when it is near (MuJoCo
// is ~11 MB of WebAssembly, a walking robot's meshes a few MB more); buttons
// with data-robot inside it switch the robot.
for (const root of document.querySelectorAll('.live[data-live]')) {
  const find = (name) => root.querySelector(`[data-live-${name}]`);
  const frame = root.querySelector('.live-frame');
  const ui = { status: find('status'), time: find('time'), play: find('play'), follow: find('follow'),
               restart: find('restart'), drive: find('drive'), pad: find('pad') };
  const note = find('note');
  const tabs = root.querySelectorAll('[data-robot]');
  let running = Promise.resolve(null);  // the scene's handle, once it runs

  const show = (name) => {
    tabs.forEach((t) => t.setAttribute('aria-pressed', String(t.dataset.robot === name)));
    // One scene at a time: stop the last one, then start this one.
    running = running.then(async (scene) => {
      scene?.stop();
      const live = await import('./live.js');
      if (note) note.textContent = live.ROBOTS[name].note;
      return live.start(frame, ui, name);
    }).catch((e) => {
      ui.status.textContent = `The live scene could not start in this browser (${e.message}).`;
      console.error(e);
      return null;
    });
  };

  let started = false;
  const go = (name = root.dataset.live) => {
    started = true;
    show(name);
  };
  tabs.forEach((t) => t.addEventListener('click', () => go(t.dataset.robot)));
  if ('IntersectionObserver' in window) {
    new IntersectionObserver((entries, obs) => {
      if (entries.some((e) => e.isIntersecting)) { obs.disconnect(); if (!started) go(); }
    }, { rootMargin: '300px' }).observe(frame);
  } else {
    go();
  }
}
