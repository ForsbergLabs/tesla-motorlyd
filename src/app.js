/**
 * app.js — UI glue for Tesla Motorlyd (improved).
 * All driving logic lives in controller.js / drive.js; all sound in engine-sim.js.
 */
import { EngineSim, VEHICLE_PRESETS } from './engine-sim.js';
import { DriveController } from './controller.js';

// All engines from the original app. `base` = engine profile, `turbo` = bolt-on turbo.
const ENGINES = [
  { id: 'v8cross', label: 'V8 Crossplane' },
  { id: 'v8flat', label: 'V8 Flat-plane' },
  { id: 'v8tt', label: 'V8 Twin-Turbo', turbo: true },
  { id: 'v12', label: 'V12' },
  { id: 'v10', label: 'V10' },
  { id: 'flat6', label: 'Flat-6' },
  { id: 'i6', label: 'Inline-6' },
  { id: 'i4', label: 'Inline-4' },
  { id: 'i4turbo', label: 'Inline-4 Turbo', turbo: true, base: 'i4' },
  { id: 'boxer4', label: 'Boxer-4' },
  { id: 'rotary2', label: 'Rotary' },
  { id: 'i5', label: 'Inline-5' },
  { id: 'vtwin', label: 'V-twin' },
  { id: 'v6tt', label: 'V6 Twin-Turbo', turbo: true, base: 'v6tt' },
];

// Turbo set-up: modern low-inertia turbo with boost from ~2200 rpm and a
// properly sized ATMOSPHERIC blow-off valve (clean "pssh", no compressor
// flutter / "stu-stu" — `surge: 0` switches the flutter off entirely).
const TURBO = { inertia: 0.32, maxBoost: 1.15, boostRpm: 1800, valve: 1, bov: 0.85, surge: 0, flutterHz: 13, screamer: 0, antiLag: 0 };

// Both drive modes use the same dual-clutch car; Normal shifts softer/slower,
// Sport faster. Shift points differ in GearSelector.
const VEHICLES = {
  normal: { ...VEHICLE_PRESETS.supercar, label: 'Normal (DCT)', shiftTimeMs: 150 },
  sport: { ...VEHICLE_PRESETS.supercar, label: 'Sport (DCT)', shiftTimeMs: 75 },
};

const $ = (id) => document.getElementById(id);
const el = {
  engines: $('engines'), rpm: $('rpm'), gear: $('gear'), gearNum: $('gearNum'), speed: $('speed'),
  speedSrc: $('speedSrc'), load: $('load'), status: $('status'), start: $('startBtn'),
  modeTest: $('modeTest'), modeGps: $('modeGps'), modeDemo: $('modeDemo'),
  modeNormal: $('modeNormal'), modeSport: $('modeSport'), pops: $('popsBtn'),
  target: $('targetSpeed'), targetVal: $('targetVal'), volume: $('volume'),
  gpsStatus: $('gpsStatus'), speedWrap: $('speedWrap'), tach: $('tachFill'),
};

let sim = null, ctl = null, running = false;
let source = 'test', driveMode = 'normal', currentEngine = 'v8cross', pops = false;
let wakeLock = null;

function buildEngineList() {
  el.engines.innerHTML = '';
  for (const e of ENGINES) {
    const b = document.createElement('button');
    b.className = 'eng-btn' + (e.id === currentEngine ? ' active' : '');
    b.textContent = e.label;
    b.onclick = () => selectEngine(e.id);
    el.engines.appendChild(b);
  }
}

function applyEngine() {
  const eng = ENGINES.find((e) => e.id === currentEngine);
  if (!sim) return eng;
  sim.setEngineType(eng.base || eng.id);
  Object.assign(sim.turboCfg, TURBO);
  sim.setTurbo(!!eng.turbo);
  if (ctl) ctl.refresh();
  return eng;
}

function selectEngine(id) {
  currentEngine = id;
  buildEngineList();
  const eng = applyEngine();
  el.status.textContent = 'Motor: ' + eng.label;
}

function applyDriveMode() {
  if (!sim) return;
  sim.setVehicleProfile(VEHICLES[driveMode]);
  sim.setInputs({ aggression: driveMode === 'sport' ? 0.85 : 0.35 });
  if (ctl) { ctl.setMode(driveMode); ctl.refresh(); }
}

function setDriveMode(m) {
  driveMode = m;
  el.modeNormal.classList.toggle('active', m === 'normal');
  el.modeSport.classList.toggle('active', m === 'sport');
  applyDriveMode();
}

function applyPops() {
  el.pops.textContent = 'Knatring: ' + (pops ? 'på' : 'av');
  el.pops.classList.toggle('active', pops);
  if (!sim) return;
  // Off: exhaust bangs/crackle and clunks kept as a faint texture only.
  sim.setMix({ transients: pops ? 0.5 : 0.14 });
  sim.setPopDepth(pops ? 1.1 : 0.35);
}

function setGpsStatus(s) {
  el.gpsStatus.className = 'gps-status' + (s.level ? ' ' + s.level : '');
  el.gpsStatus.textContent = s.text;
}

function setSource(m) {
  source = m;
  el.modeTest.classList.toggle('active', m === 'test');
  el.modeGps.classList.toggle('active', m === 'gps');
  el.modeDemo.classList.toggle('active', m === 'demo');
  el.speedWrap.style.display = m === 'test' ? '' : 'none';
  el.speedSrc.textContent = m === 'gps' ? 'km/h GPS' : m === 'demo' ? 'km/h demo' : 'km/h';
  if (m === 'test') setGpsStatus({ level: '', text: 'Testmodus — bruk fart-slideren' });
  if (m === 'gps') setGpsStatus({ level: '', text: running ? 'Venter på GPS…' : 'GPS-modus — trykk Start og tillat posisjon' });
  if (m === 'demo') setGpsStatus({ level: '', text: running ? 'Demo-tur starter…' : 'Demo-tur — trykk Start' });
  if (ctl && running) ctl.setSource(m);
}

async function requestWakeLock() {
  try {
    if ('wakeLock' in navigator && !wakeLock) {
      wakeLock = await navigator.wakeLock.request('screen');
      wakeLock.addEventListener('release', () => { wakeLock = null; });
    }
  } catch (e) { /* not allowed / unsupported — harmless */ }
}

async function start() {
  if (running) { stop(); return; }
  try {
    const eng = ENGINES.find((e) => e.id === currentEngine);
    if (!sim) {
      // Created inside the click handler: the only reliable way to get an
      // unlocked AudioContext in the Tesla (Chromium) browser.
      sim = new EngineSim(null, {
        engine: eng.base || eng.id, vehicle: 'supercar', volume: el.volume.value / 100,
        turbo: !!eng.turbo, latencyHint: 'playback',
      });
      Object.assign(sim.turboCfg, TURBO);
      sim.setTurbo(!!eng.turbo);
      sim.setVehicleProfile(VEHICLES[driveMode]);
      ctl = new DriveController(sim, { onGpsStatus: (s) => { if (source === 'gps') setGpsStatus(s); } });
      window.__motorlyd = { sim, ctl };
    }
    applyDriveMode();
    applyPops();
    await sim.start();
    if (sim.ctx.state !== 'running') await sim.ctx.resume().catch(() => {});
    running = true;
    el.start.textContent = 'Stopp motor';
    el.start.classList.add('running');
    el.status.textContent = 'Motor kjører — ' + eng.label;
    ctl.source = 'test';
    ctl.setSource(source);
    if (source === 'test') ctl.setTargetSpeedKmh(Number(el.target.value));
    requestWakeLock();
    lastFrame = performance.now();
    requestAnimationFrame(loop);
  } catch (err) {
    el.status.textContent = 'Feil: ' + err.message;
    console.error(err);
  }
}

function stop() {
  running = false;
  if (ctl) ctl.stop();
  if (sim) sim.stop();
  el.start.textContent = 'Start motor';
  el.start.classList.remove('running');
  el.status.textContent = 'Stoppet';
  if (wakeLock) { wakeLock.release().catch(() => {}); wakeLock = null; }
}

let lastFrame = 0, lastUi = 0;
function frame(now) {
  const dt = Math.min(0.05, Math.max(0, (now - lastFrame) / 1000));
  lastFrame = now;
  if (dt <= 0) return;
  ctl.frame(dt, now);
  sim.update(dt);
  if (now - lastUi > 66) {          // ~15 Hz DOM updates are plenty
    lastUi = now;
    const st = sim.getState();
    const rpm = Math.round(st.rpm || 0);
    el.rpm.innerHTML = rpm + ' <small>RPM</small>';
    const g = st.gear || 0;
    el.gear.textContent = g > 0 ? g : 'N';
    el.gearNum.textContent = g > 0 ? `${g}/${st.gearCount}` : '—';
    el.speed.textContent = Math.round(ctl.displayV * 3.6);
    el.load.textContent = Math.round((st.load || 0) * 100);
    el.tach.style.width = Math.min(100, (rpm / (st.redline || 7000)) * 100) + '%';
    if (source === 'demo') setGpsStatus({ level: 'ok', text: '' });
    if (source === 'demo') el.gpsStatus.innerHTML = `<span class="demo-tag">DEMO</span>${ctl.demoLabel} · ${(ctl.gps.rawSpeed != null ? Math.round(ctl.gps.rawSpeed * 3.6) : 0)} km/h målt`;
  }
}
function loop(now) {
  if (!running) return;
  frame(now);
  requestAnimationFrame(loop);
}
// rAF can be throttled when the pane is hidden/backgrounded; keep the engine
// model ticking at >= 10 Hz so the sound never freezes on one note.
setInterval(() => {
  if (running && performance.now() - lastFrame > 150) frame(performance.now());
}, 100);

// Re-unlock audio and wake lock when the page comes back / on any touch.
function resumeAudio() {
  if (running && sim && sim.ctx.state !== 'running') sim.ctx.resume().catch(() => {});
}
document.addEventListener('visibilitychange', () => { if (!document.hidden) { resumeAudio(); if (running) requestWakeLock(); } });
document.addEventListener('pointerdown', resumeAudio, { passive: true });

el.start.onclick = start;
el.modeTest.onclick = () => setSource('test');
el.modeGps.onclick = () => setSource('gps');
el.modeDemo.onclick = () => setSource('demo');
el.modeNormal.onclick = () => setDriveMode('normal');
el.modeSport.onclick = () => setDriveMode('sport');
el.pops.onclick = () => { pops = !pops; applyPops(); };
el.target.oninput = () => {
  el.targetVal.textContent = 'Mål: ' + el.target.value + ' km/h';
  if (ctl) ctl.setTargetSpeedKmh(Number(el.target.value));
};
el.volume.oninput = () => { if (sim) sim.setVolume(el.volume.value / 100); };

buildEngineList();
setSource('test');
applyPops();
el.status.textContent = 'Velg motor og trykk Start';
